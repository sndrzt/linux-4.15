#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/workqueue.h>
#include <linux/rtnetlink.h>
#include <linux/cache.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/idr.h>
#include <linux/rculist.h>
#include <linux/nsproxy.h>
#include <linux/fs.h>
#include <linux/proc_ns.h>
#include <linux/file.h>
#include <linux/export.h>
#include <linux/user_namespace.h>
#include <linux/net_namespace.h>
#include <linux/sched/task.h>
#include <linux/uidgid.h>

#include <net/sock.h>
#include <net/netlink.h>
#include <net/net_namespace.h>
#include <net/netns/generic.h>

/*
 *	Our network namespace constructor/destructor lists
 */

static LIST_HEAD(pernet_list);
static struct list_head *first_device = &pernet_list;
DEFINE_MUTEX(net_mutex);

LIST_HEAD(net_namespace_list);
EXPORT_SYMBOL_GPL(net_namespace_list);

struct net init_net = {
	.count		= ATOMIC_INIT(1),
	.dev_base_head	= LIST_HEAD_INIT(init_net.dev_base_head),
};
EXPORT_SYMBOL(init_net);

static bool init_net_initialized;

#define MIN_PERNET_OPS_ID	\
	((sizeof(struct net_generic) + sizeof(void *) - 1) / sizeof(void *))

#define INITIAL_NET_GEN_PTRS	13 /* +1 for len +2 for rcu_head */

static unsigned int max_gen_ptrs = INITIAL_NET_GEN_PTRS;

static struct net_generic *net_alloc_generic(void)
{
	struct net_generic *ng;
	unsigned int generic_size = offsetof(struct net_generic, ptr[max_gen_ptrs]);

	ng = kzalloc(generic_size, GFP_KERNEL);
	if (ng)
		ng->s.len = max_gen_ptrs;

	return ng;
}

static int net_assign_generic(struct net *net, unsigned int id, void *data)
{
	struct net_generic *ng, *old_ng;

	BUG_ON(!mutex_is_locked(&net_mutex));
	BUG_ON(id < MIN_PERNET_OPS_ID);

	old_ng = rcu_dereference_protected(net->gen,
					   lockdep_is_held(&net_mutex));
	if (old_ng->s.len > id) {
		old_ng->ptr[id] = data;
		return 0;
	}

	ng = net_alloc_generic();
	if (ng == NULL)
		return -ENOMEM;

	/*
	 * Some synchronisation notes:
	 *
	 * The net_generic explores the net->gen array inside rcu
	 * read section. Besides once set the net->gen->ptr[x]
	 * pointer never changes (see rules in netns/generic.h).
	 *
	 * That said, we simply duplicate this array and schedule
	 * the old copy for kfree after a grace period.
	 */

	memcpy(&ng->ptr[MIN_PERNET_OPS_ID], &old_ng->ptr[MIN_PERNET_OPS_ID],
	       (old_ng->s.len - MIN_PERNET_OPS_ID) * sizeof(void *));
	ng->ptr[id] = data;

	rcu_assign_pointer(net->gen, ng);
	kfree_rcu(old_ng, s.rcu);
	return 0;
}

static int ops_init(const struct pernet_operations *ops, struct net *net)
{
	struct net_generic *ng;
	int err = -ENOMEM;
	void *data = NULL;

	if (ops->id && ops->size) {
		data = kzalloc(ops->size, GFP_KERNEL);
		if (!data)
			goto out;

		err = net_assign_generic(net, *ops->id, data);
		if (err)
			goto cleanup;
	}
	err = 0;
	if (ops->init)
		err = ops->init(net);
	if (!err)
		return 0;

	if (ops->id && ops->size) {
		ng = rcu_dereference_protected(net->gen,
					       lockdep_is_held(&pernet_ops_rwsem));
		ng->ptr[*ops->id] = NULL;
	}

cleanup:
	kfree(data);

out:
	return err;
}

static void ops_free(const struct pernet_operations *ops, struct net *net)
{
	if (ops->id && ops->size) {
		kfree(net_generic(net, *ops->id));
	}
}

static void ops_exit_list(const struct pernet_operations *ops,
			  struct list_head *net_exit_list)
{
	struct net *net;
	if (ops->exit) {
		list_for_each_entry(net, net_exit_list, exit_list) {
			ops->exit(net);
			cond_resched();
		}
	}
	if (ops->exit_batch)
		ops->exit_batch(net_exit_list);
}

static void ops_free_list(const struct pernet_operations *ops,
			  struct list_head *net_exit_list)
{
	struct net *net;
	if (ops->size && ops->id) {
		list_for_each_entry(net, net_exit_list, exit_list)
			ops_free(ops, net);
	}
}

/* should be called with nsid_lock held */
static int alloc_netid(struct net *net, struct net *peer, int reqid)
{
	int min = 0, max = 0;

	if (reqid >= 0) {
		min = reqid;
		max = reqid + 1;
	}

	return idr_alloc(&net->netns_ids, peer, min, max, GFP_ATOMIC);
}

/* This function is used by idr_for_each(). If net is equal to peer, the
 * function returns the id so that idr_for_each() stops. Because we cannot
 * returns the id 0 (idr_for_each() will not stop), we return the magic value
 * NET_ID_ZERO (-1) for it.
 */
#define NET_ID_ZERO -1
static int net_eq_idr(int id, void *net, void *peer)
{
	if (net_eq(net, peer))
		return id ? : NET_ID_ZERO;
	return 0;
}

/* Must be called from RCU-critical section or with nsid_lock held. If
 * a new id is assigned, the bool alloc is set to true, thus the
 * caller knows that the new id must be notified via rtnl.
 */
static int __peernet2id_alloc(struct net *net, struct net *peer, bool *alloc)
{
	int id = idr_for_each(&net->netns_ids, net_eq_idr, peer);
	bool alloc_it = *alloc;

	*alloc = false;

	/* Magic value for id 0. */
	if (id == NET_ID_ZERO)
		return 0;
	if (id > 0)
		return id;

	if (alloc_it) {
		id = alloc_netid(net, peer, -1);
		*alloc = true;
		return id >= 0 ? id : NETNSA_NSID_NOT_ASSIGNED;
	}

	return NETNSA_NSID_NOT_ASSIGNED;
}

/* Must be called from RCU-critical section or with nsid_lock held */
static int __peernet2id(struct net *net, struct net *peer)
{
	bool no = false;

	return __peernet2id_alloc(net, peer, &no);
}

static void rtnl_net_notifyid(struct net *net, int cmd, int id, gfp_t gfp);
/* This function returns the id of a peer netns. If no id is assigned, one will
 * be allocated and returned.
 */
int peernet2id_alloc(struct net *net, struct net *peer, gfp_t gfp)
{
	bool alloc;
	int id;

	if (atomic_read(&net->count) == 0)
		return NETNSA_NSID_NOT_ASSIGNED;
	spin_lock_bh(&net->nsid_lock);
	alloc = atomic_read(&peer->count) == 0 ? false : true;
	id = __peernet2id_alloc(net, peer, &alloc);
	spin_unlock_bh(&net->nsid_lock);
	if (alloc && id >= 0)
		rtnl_net_notifyid(net, RTM_NEWNSID, id, gfp);
	return id;
}
EXPORT_SYMBOL_GPL(peernet2id_alloc);

/* This function returns, if assigned, the id of a peer netns. */
int peernet2id(struct net *net, struct net *peer)
{
	int id;

	rcu_read_lock();
	id = __peernet2id(net, peer);
	rcu_read_unlock();

	return id;
}
EXPORT_SYMBOL(peernet2id);

/* This function returns true is the peer netns has an id assigned into the
 * current netns.
 */
bool peernet_has_id(struct net *net, struct net *peer)
{
	return peernet2id(net, peer) >= 0;
}

struct net *get_net_ns_by_id(struct net *net, int id)
{
	struct net *peer;

	if (id < 0)
		return NULL;

	rcu_read_lock();
	spin_lock_bh(&net->nsid_lock);
	peer = idr_find(&net->netns_ids, id);
	if (peer)
		peer = maybe_get_net(peer);
	spin_unlock_bh(&net->nsid_lock);
	rcu_read_unlock();

	return peer;
}

/*
 * setup_net runs the initializers for the network namespace object.
 */
static __net_init int setup_net(struct net *net, struct user_namespace *user_ns)
{
	/* Must be called with net_mutex held */
	const struct pernet_operations *ops, *saved_ops;
	int error = 0;
	LIST_HEAD(net_exit_list);

	atomic_set(&net->count, 1);
	refcount_set(&net->passive, 1);
	get_random_bytes(&net->hash_mix, sizeof(u32));
	net->dev_base_seq = 1;
	net->user_ns = user_ns;
	idr_init(&net->netns_ids);
	spin_lock_init(&net->nsid_lock);

	list_for_each_entry(ops, &pernet_list, list) {
		error = ops_init(ops, net);
		if (error < 0)
			goto out_undo;
	}
out:
	return error;

out_undo:
	/* Walk through the list backwards calling the exit functions
	 * for the pernet modules whose init functions did not fail.
	 */
	list_add(&net->exit_list, &net_exit_list);
	saved_ops = ops;
	list_for_each_entry_continue_reverse(ops, &pernet_list, list)
		ops_exit_list(ops, &net_exit_list);

	ops = saved_ops;
	list_for_each_entry_continue_reverse(ops, &pernet_list, list)
		ops_free_list(ops, &net_exit_list);

	rcu_barrier();
	goto out;
}

static int __net_init net_defaults_init_net(struct net *net)
{
	net->core.sysctl_somaxconn = SOMAXCONN;
	return 0;
}

static struct pernet_operations net_defaults_ops = {
	.init = net_defaults_init_net,
};

static __init int net_defaults_init(void)
{
	if (register_pernet_subsys(&net_defaults_ops))
		panic("Cannot initialize net default settings");

	return 0;
}

core_initcall(net_defaults_init);

#ifdef CONFIG_NET_NS
static struct ucounts *inc_net_namespaces(struct user_namespace *ns)
{
	return inc_ucount(ns, current_euid(), UCOUNT_NET_NAMESPACES);
}

static void dec_net_namespaces(struct ucounts *ucounts)
{
	dec_ucount(ucounts, UCOUNT_NET_NAMESPACES);
}

static struct kmem_cache *net_cachep;
static struct workqueue_struct *netns_wq;

static struct net *net_alloc(void)
{
	struct net *net = NULL;
	struct net_generic *ng;

	ng = net_alloc_generic();
	if (!ng)
		goto out;

	net = kmem_cache_zalloc(net_cachep, GFP_KERNEL);
	if (!net)
		goto out_free;

	rcu_assign_pointer(net->gen, ng);
out:
	return net;

out_free:
	kfree(ng);
	goto out;
}

static void net_free(struct net *net)
{
	kfree(rcu_access_pointer(net->gen));
	kmem_cache_free(net_cachep, net);
}

void net_drop_ns(void *p)
{
	struct net *ns = p;
	if (ns && refcount_dec_and_test(&ns->passive))
		net_free(ns);
}

struct net *copy_net_ns(unsigned long flags,
			struct user_namespace *user_ns, struct net *old_net)
{
	struct ucounts *ucounts;
	struct net *net;
	int rv;

	if (!(flags & CLONE_NEWNET))
		return get_net(old_net);

	ucounts = inc_net_namespaces(user_ns);
	if (!ucounts)
		return ERR_PTR(-ENOSPC);

	net = net_alloc();
	if (!net) {
		dec_net_namespaces(ucounts);
		return ERR_PTR(-ENOMEM);
	}

	get_user_ns(user_ns);

	rv = mutex_lock_killable(&net_mutex);
	if (rv < 0) {
		net_free(net);
		dec_net_namespaces(ucounts);
		put_user_ns(user_ns);
		return ERR_PTR(rv);
	}

	net->ucounts = ucounts;
	rv = setup_net(net, user_ns);
	if (rv == 0) {
		rtnl_lock();
		list_add_tail_rcu(&net->list, &net_namespace_list);
		rtnl_unlock();
	}
	mutex_unlock(&net_mutex);
	if (rv < 0) {
		dec_net_namespaces(ucounts);
		put_user_ns(user_ns);
		net_drop_ns(net);
		return ERR_PTR(rv);
	}
	return net;
}

/**
 * net_ns_get_ownership - get sysfs ownership data for @net
 * @net: network namespace in question (can be NULL)
 * @uid: kernel user ID for sysfs objects
 * @gid: kernel group ID for sysfs objects
 *
 * Returns the uid/gid pair of root in the user namespace associated with the
 * given network namespace.
 */
void net_ns_get_ownership(const struct net *net, kuid_t *uid, kgid_t *gid)
{
	if (net) {
		kuid_t ns_root_uid = make_kuid(net->user_ns, 0);
		kgid_t ns_root_gid = make_kgid(net->user_ns, 0);

		if (uid_valid(ns_root_uid))
			*uid = ns_root_uid;

		if (gid_valid(ns_root_gid))
			*gid = ns_root_gid;
	} else {
		*uid = GLOBAL_ROOT_UID;
		*gid = GLOBAL_ROOT_GID;
	}
}
EXPORT_SYMBOL_GPL(net_ns_get_ownership);

static DEFINE_SPINLOCK(cleanup_list_lock);
static LIST_HEAD(cleanup_list);  /* Must hold cleanup_list_lock to touch */

static void cleanup_net(struct work_struct *work)
{
	const struct pernet_operations *ops;
	struct net *net, *tmp;
	struct list_head net_kill_list;
	LIST_HEAD(net_exit_list);

	/* Atomically snapshot the list of namespaces to cleanup */
	spin_lock_irq(&cleanup_list_lock);
	list_replace_init(&cleanup_list, &net_kill_list);
	spin_unlock_irq(&cleanup_list_lock);

	mutex_lock(&net_mutex);

	/* Don't let anyone else find us. */
	rtnl_lock();
	list_for_each_entry(net, &net_kill_list, cleanup_list) {
		list_del_rcu(&net->list);
		list_add_tail(&net->exit_list, &net_exit_list);
		for_each_net(tmp) {
			int id;

			spin_lock_bh(&tmp->nsid_lock);
			id = __peernet2id(tmp, net);
			if (id >= 0)
				idr_remove(&tmp->netns_ids, id);
			spin_unlock_bh(&tmp->nsid_lock);
			if (id >= 0)
				rtnl_net_notifyid(tmp, RTM_DELNSID, id,
						  GFP_KERNEL);
		}
		spin_lock_bh(&net->nsid_lock);
		idr_destroy(&net->netns_ids);
		spin_unlock_bh(&net->nsid_lock);

	}
	rtnl_unlock();

	/*
	 * Another CPU might be rcu-iterating the list, wait for it.
	 * This needs to be before calling the exit() notifiers, so
	 * the rcu_barrier() below isn't sufficient alone.
	 */
	synchronize_rcu();

	/* Run all of the network namespace exit methods */
	list_for_each_entry_reverse(ops, &pernet_list, list)
		ops_exit_list(ops, &net_exit_list);

	/* Free the net generic variables */
	list_for_each_entry_reverse(ops, &pernet_list, list)
		ops_free_list(ops, &net_exit_list);

	mutex_unlock(&net_mutex);

	/* Ensure there are no outstanding rcu callbacks using this
	 * network namespace.
	 */
	rcu_barrier();

	/* Finally it is safe to free my network namespace structure */
	list_for_each_entry_safe(net, tmp, &net_exit_list, exit_list) {
		list_del_init(&net->exit_list);
		dec_net_namespaces(net->ucounts);
		put_user_ns(net->user_ns);
		net_drop_ns(net);
	}
}

/**
 * net_ns_barrier - wait until concurrent net_cleanup_work is done
 *
 * cleanup_net runs from work queue and will first remove namespaces
 * from the global list, then run net exit functions.
 *
 * Call this in module exit path to make sure that all netns
 * ->exit ops have been invoked before the function is removed.
 */
void net_ns_barrier(void)
{
	mutex_lock(&net_mutex);
	mutex_unlock(&net_mutex);
}
EXPORT_SYMBOL(net_ns_barrier);

static DECLARE_WORK(net_cleanup_work, cleanup_net);

void __put_net(struct net *net)
{
	/* Cleanup the network namespace in process context */
	unsigned long flags;

	spin_lock_irqsave(&cleanup_list_lock, flags);
	list_add(&net->cleanup_list, &cleanup_list);
	spin_unlock_irqrestore(&cleanup_list_lock, flags);

	queue_work(netns_wq, &net_cleanup_work);
}
EXPORT_SYMBOL_GPL(__put_net);

/**
 * get_net_ns - increment the refcount of the network namespace
 * @ns: common namespace (net)
 *
 * Returns the net's common namespace.
 */
struct ns_common *get_net_ns(struct ns_common *ns)
{
	return &get_net(container_of(ns, struct net, ns))->ns;
}
EXPORT_SYMBOL_GPL(get_net_ns);

struct net *get_net_ns_by_fd(int fd)
{
	struct file *file;
	struct ns_common *ns;
	struct net *net;

	file = proc_ns_fget(fd);
	if (IS_ERR(file))
		return ERR_CAST(file);

	ns = get_proc_ns(file_inode(file));
	if (ns->ops == &netns_operations)
		net = get_net(container_of(ns, struct net, ns));
	else
		net = ERR_PTR(-EINVAL);

	fput(file);
	return net;
}

#else
struct net *get_net_ns_by_fd(int fd)
{
	return ERR_PTR(-EINVAL);
}
#endif
EXPORT_SYMBOL_GPL(get_net_ns_by_fd);

struct net *get_net_ns_by_pid(pid_t pid)
{
	struct task_struct *tsk;
	struct net *net;

	/* Lookup the network namespace */
	net = ERR_PTR(-ESRCH);
	rcu_read_lock();
	tsk = find_task_by_vpid(pid);
	if (tsk) {
		struct nsproxy *nsproxy;
		task_lock(tsk);
		nsproxy = tsk->nsproxy;
		if (nsproxy)
			net = get_net(nsproxy->net_ns);
		task_unlock(tsk);
	}
	rcu_read_unlock();
	return net;
}
EXPORT_SYMBOL_GPL(get_net_ns_by_pid);

static __net_init int net_ns_net_init(struct net *net)
{
#ifdef CONFIG_NET_NS
	net->ns.ops = &netns_operations;
#endif
	return ns_alloc_inum(&net->ns);
}

static __net_exit void net_ns_net_exit(struct net *net)
{
	ns_free_inum(&net->ns);
}

static struct pernet_operations __net_initdata net_ns_ops = {
	.init = net_ns_net_init,
	.exit = net_ns_net_exit,
};

static const struct nla_policy rtnl_net_policy[NETNSA_MAX + 1] = {
	[NETNSA_NONE]		= { .type = NLA_UNSPEC },
	[NETNSA_NSID]		= { .type = NLA_S32 },
	[NETNSA_PID]		= { .type = NLA_U32 },
	[NETNSA_FD]		= { .type = NLA_U32 },
};

static int rtnl_net_newid(struct sk_buff *skb, struct nlmsghdr *nlh,
			  struct netlink_ext_ack *extack)
{
	struct net *net = sock_net(skb->sk);
	struct nlattr *tb[NETNSA_MAX + 1];
	struct nlattr *nla;
	struct net *peer;
	int nsid, err;

	err = nlmsg_parse(nlh, sizeof(struct rtgenmsg), tb, NETNSA_MAX,
			  rtnl_net_policy, extack);
	if (err < 0)
		return err;
	if (!tb[NETNSA_NSID]) {
		NL_SET_ERR_MSG(extack, "nsid is missing");
		return -EINVAL;
	}
	nsid = nla_get_s32(tb[NETNSA_NSID]);

	if (tb[NETNSA_PID]) {
		peer = get_net_ns_by_pid(nla_get_u32(tb[NETNSA_PID]));
		nla = tb[NETNSA_PID];
	} else if (tb[NETNSA_FD]) {
		peer = get_net_ns_by_fd(nla_get_u32(tb[NETNSA_FD]));
		nla = tb[NETNSA_FD];
	} else {
		NL_SET_ERR_MSG(extack, "Peer netns reference is missing");
		return -EINVAL;
	}
	if (IS_ERR(peer)) {
		NL_SET_BAD_ATTR(extack, nla);
		NL_SET_ERR_MSG(extack, "Peer netns reference is invalid");
		return PTR_ERR(peer);
	}

	spin_lock_bh(&net->nsid_lock);
	if (__peernet2id(net, peer) >= 0) {
		spin_unlock_bh(&net->nsid_lock);
		err = -EEXIST;
		NL_SET_BAD_ATTR(extack, nla);
		NL_SET_ERR_MSG(extack,
			       "Peer netns already has a nsid assigned");
		goto out;
	}

	err = alloc_netid(net, peer, nsid);
	spin_unlock_bh(&net->nsid_lock);
	if (err >= 0) {
		rtnl_net_notifyid(net, RTM_NEWNSID, err, GFP_KERNEL);
		err = 0;
	} else if (err == -ENOSPC && nsid >= 0) {
		err = -EEXIST;
		NL_SET_BAD_ATTR(extack, tb[NETNSA_NSID]);
		NL_SET_ERR_MSG(extack, "The specified nsid is already used");
	}
out:
	put_net(peer);
	return err;
}

static int rtnl_net_get_size(void)
{
	return NLMSG_ALIGN(sizeof(struct rtgenmsg))
	       + nla_total_size(sizeof(s32)) /* NETNSA_NSID */
	       ;
}

static int rtnl_net_fill(struct sk_buff *skb, u32 portid, u32 seq, int flags,
			 int cmd, struct net *net, int nsid)
{
	struct nlmsghdr *nlh;
	struct rtgenmsg *rth;

	nlh = nlmsg_put(skb, portid, seq, cmd, sizeof(*rth), flags);
	if (!nlh)
		return -EMSGSIZE;

	rth = nlmsg_data(nlh);
	rth->rtgen_family = AF_UNSPEC;

	if (nla_put_s32(skb, NETNSA_NSID, nsid))
		goto nla_put_failure;

	nlmsg_end(skb, nlh);
	return 0;

nla_put_failure:
	nlmsg_cancel(skb, nlh);
	return -EMSGSIZE;
}

static int rtnl_net_getid(struct sk_buff *skb, struct nlmsghdr *nlh,
			  struct netlink_ext_ack *extack)
{
	struct net *net = sock_net(skb->sk);
	struct nlattr *tb[NETNSA_MAX + 1];
	struct nlattr *nla;
	struct sk_buff *msg;
	struct net *peer;
	int err, id;

	err = nlmsg_parse(nlh, sizeof(struct rtgenmsg), tb, NETNSA_MAX,
			  rtnl_net_policy, extack);
	if (err < 0)
		return err;
	if (tb[NETNSA_PID]) {
		peer = get_net_ns_by_pid(nla_get_u32(tb[NETNSA_PID]));
		nla = tb[NETNSA_PID];
	} else if (tb[NETNSA_FD]) {
		peer = get_net_ns_by_fd(nla_get_u32(tb[NETNSA_FD]));
		nla = tb[NETNSA_FD];
	} else {
		NL_SET_ERR_MSG(extack, "Peer netns reference is missing");
		return -EINVAL;
	}

	if (IS_ERR(peer)) {
		NL_SET_BAD_ATTR(extack, nla);
		NL_SET_ERR_MSG(extack, "Peer netns reference is invalid");
		return PTR_ERR(peer);
	}

	msg = nlmsg_new(rtnl_net_get_size(), GFP_KERNEL);
	if (!msg) {
		err = -ENOMEM;
		goto out;
	}

	id = peernet2id(net, peer);
	err = rtnl_net_fill(msg, NETLINK_CB(skb).portid, nlh->nlmsg_seq, 0,
			    RTM_NEWNSID, net, id);
	if (err < 0)
		goto err_out;

	err = rtnl_unicast(msg, net, NETLINK_CB(skb).portid);
	goto out;

err_out:
	nlmsg_free(msg);
out:
	put_net(peer);
	return err;
}

struct rtnl_net_dump_cb {
	struct net *net;
	struct sk_buff *skb;
	struct netlink_callback *cb;
	int idx;
	int s_idx;
};

/* Runs in RCU-critical section. */
static int rtnl_net_dumpid_one(int id, void *peer, void *data)
{
	struct rtnl_net_dump_cb *net_cb = (struct rtnl_net_dump_cb *)data;
	int ret;

	if (net_cb->idx < net_cb->s_idx)
		goto cont;

	ret = rtnl_net_fill(net_cb->skb, NETLINK_CB(net_cb->cb->skb).portid,
			    net_cb->cb->nlh->nlmsg_seq, NLM_F_MULTI,
			    RTM_NEWNSID, net_cb->net, id);
	if (ret < 0)
		return ret;

cont:
	net_cb->idx++;
	return 0;
}

static int rtnl_net_dumpid(struct sk_buff *skb, struct netlink_callback *cb)
{
	struct net *net = sock_net(skb->sk);
	struct rtnl_net_dump_cb net_cb = {
		.net = net,
		.skb = skb,
		.cb = cb,
		.idx = 0,
		.s_idx = cb->args[0],
	};

	rcu_read_lock();
	idr_for_each(&net->netns_ids, rtnl_net_dumpid_one, &net_cb);
	rcu_read_unlock();

	cb->args[0] = net_cb.idx;
	return skb->len;
}

static void rtnl_net_notifyid(struct net *net, int cmd, int id, gfp_t gfp)
{
	struct sk_buff *msg;
	int err = -ENOMEM;

	msg = nlmsg_new(rtnl_net_get_size(), gfp);
	if (!msg)
		goto out;

	err = rtnl_net_fill(msg, 0, 0, 0, cmd, net, id);
	if (err < 0)
		goto err_out;

	rtnl_notify(msg, net, 0, RTNLGRP_NSID, NULL, gfp);
	return;

err_out:
	nlmsg_free(msg);
out:
	rtnl_set_sk_err(net, RTNLGRP_NSID, err);
}

static int __init net_ns_init(void)
{
	struct net_generic *ng;

#ifdef CONFIG_NET_NS
	net_cachep = kmem_cache_create("net_namespace", sizeof(struct net),
					SMP_CACHE_BYTES,
					SLAB_PANIC, NULL);

	/* Create workqueue for cleanup */
	netns_wq = create_singlethread_workqueue("netns");
	if (!netns_wq)
		panic("Could not create netns workq");
#endif

	ng = net_alloc_generic();
	if (!ng)
		panic("Could not allocate generic netns");

	rcu_assign_pointer(init_net.gen, ng);

	mutex_lock(&net_mutex);
	if (setup_net(&init_net, &init_user_ns))
		panic("Could not setup the initial network namespace");

	init_net_initialized = true;

	rtnl_lock();
	list_add_tail_rcu(&init_net.list, &net_namespace_list);
	rtnl_unlock();

	mutex_unlock(&net_mutex);

	if (register_pernet_subsys(&net_ns_ops))
		panic("Could not register network namespace subsystems");

	rtnl_register(PF_UNSPEC, RTM_NEWNSID, rtnl_net_newid, NULL,
		      RTNL_FLAG_DOIT_UNLOCKED);
	rtnl_register(PF_UNSPEC, RTM_GETNSID, rtnl_net_getid, rtnl_net_dumpid,
		      RTNL_FLAG_DOIT_UNLOCKED);

	return 0;
}

pure_initcall(net_ns_init);

#ifdef CONFIG_NET_NS
static int __register_pernet_operations(struct list_head *list,
					struct pernet_operations *ops)
{
	struct net *net;
	int error;
	LIST_HEAD(net_exit_list);

	list_add_tail(&ops->list, list);
	if (ops->init || (ops->id && ops->size)) {
		for_each_net(net) {
			error = ops_init(ops, net);
			if (error)
				goto out_undo;
			list_add_tail(&net->exit_list, &net_exit_list);
		}
	}
	return 0;

out_undo:
	/* If I have an error cleanup all namespaces I initialized */
	list_del(&ops->list);
	ops_exit_list(ops, &net_exit_list);
	ops_free_list(ops, &net_exit_list);
	return error;
}

static void __unregister_pernet_operations(struct pernet_operations *ops)
{
	struct net *net;
	LIST_HEAD(net_exit_list);

	list_del(&ops->list);
	for_each_net(net)
		list_add_tail(&net->exit_list, &net_exit_list);
	ops_exit_list(ops, &net_exit_list);
	ops_free_list(ops, &net_exit_list);
}

#else

static int __register_pernet_operations(struct list_head *list,
					struct pernet_operations *ops)
{
	if (!init_net_initialized) {
		list_add_tail(&ops->list, list);
		return 0;
	}

	return ops_init(ops, &init_net);
}

static void __unregister_pernet_operations(struct pernet_operations *ops)
{
	if (!init_net_initialized) {
		list_del(&ops->list);
	} else {
		LIST_HEAD(net_exit_list);
		list_add(&init_net.exit_list, &net_exit_list);
		ops_exit_list(ops, &net_exit_list);
		ops_free_list(ops, &net_exit_list);
	}
}

#endif /* CONFIG_NET_NS */

static DEFINE_IDA(net_generic_ids);

static int register_pernet_operations(struct list_head *list,
				      struct pernet_operations *ops)
{
	int error;

	if (ops->id) {
again:
		error = ida_get_new_above(&net_generic_ids, MIN_PERNET_OPS_ID, ops->id);
		if (error < 0) {
			if (error == -EAGAIN) {
				ida_pre_get(&net_generic_ids, GFP_KERNEL);
				goto again;
			}
			return error;
		}
		max_gen_ptrs = max(max_gen_ptrs, *ops->id + 1);
	}
	error = __register_pernet_operations(list, ops);
	if (error) {
		rcu_barrier();
		if (ops->id)
			ida_remove(&net_generic_ids, *ops->id);
	}

	return error;
}

static void unregister_pernet_operations(struct pernet_operations *ops)
{
	
	__unregister_pernet_operations(ops);
	rcu_barrier();
	if (ops->id)
		ida_remove(&net_generic_ids, *ops->id);
}

/**
 *      register_pernet_subsys - register a network namespace subsystem
 *	@ops:  pernet operations structure for the subsystem
 *
 *	Register a subsystem which has init and exit functions
 *	that are called when network namespaces are created and
 *	destroyed respectively.
 *
 *	When registered all network namespace init functions are
 *	called for every existing network namespace.  Allowing kernel
 *	modules to have a race free view of the set of network namespaces.
 *
 *	When a new network namespace is created all of the init
 *	methods are called in the order in which they were registered.
 *
 *	When a network namespace is destroyed all of the exit methods
 *	are called in the reverse of the order with which they were
 *	registered.
 */
int register_pernet_subsys(struct pernet_operations *ops)
{
	int error;
	mutex_lock(&net_mutex);
	error =  register_pernet_operations(first_device, ops);
	mutex_unlock(&net_mutex);
	return error;
}
EXPORT_SYMBOL_GPL(register_pernet_subsys);

/**
 *      unregister_pernet_subsys - unregister a network namespace subsystem
 *	@ops: pernet operations structure to manipulate
 *
 *	Remove the pernet operations structure from the list to be
 *	used when network namespaces are created or destroyed.  In
 *	addition run the exit method for all existing network
 *	namespaces.
 */
void unregister_pernet_subsys(struct pernet_operations *ops)
{
	mutex_lock(&net_mutex);
	unregister_pernet_operations(ops);
	mutex_unlock(&net_mutex);
}
EXPORT_SYMBOL_GPL(unregister_pernet_subsys);

/**
 *      register_pernet_device - register a network namespace device
 *	@ops:  pernet operations structure for the subsystem
 *
 *	Register a device which has init and exit functions
 *	that are called when network namespaces are created and
 *	destroyed respectively.
 *
 *	When registered all network namespace init functions are
 *	called for every existing network namespace.  Allowing kernel
 *	modules to have a race free view of the set of network namespaces.
 *
 *	When a new network namespace is created all of the init
 *	methods are called in the order in which they were registered.
 *
 *	When a network namespace is destroyed all of the exit methods
 *	are called in the reverse of the order with which they were
 *	registered.
 */
int register_pernet_device(struct pernet_operations *ops)
{
	int error;
	mutex_lock(&net_mutex);
	error = register_pernet_operations(&pernet_list, ops);
	if (!error && (first_device == &pernet_list))
		first_device = &ops->list;
	mutex_unlock(&net_mutex);
	return error;
}
EXPORT_SYMBOL_GPL(register_pernet_device);

/**
 *      unregister_pernet_device - unregister a network namespace netdevice
 *	@ops: pernet operations structure to manipulate
 *
 *	Remove the pernet operations structure from the list to be
 *	used when network namespaces are created or destroyed.  In
 *	addition run the exit method for all existing network
 *	namespaces.
 */
void unregister_pernet_device(struct pernet_operations *ops)
{
	mutex_lock(&net_mutex);
	if (&ops->list == first_device)
		first_device = first_device->next;
	unregister_pernet_operations(ops);
	mutex_unlock(&net_mutex);
}
EXPORT_SYMBOL_GPL(unregister_pernet_device);

#ifdef CONFIG_NET_NS
static struct ns_common *netns_get(struct task_struct *task)
{
	struct net *net = NULL;
	struct nsproxy *nsproxy;

	task_lock(task);
	nsproxy = task->nsproxy;
	if (nsproxy)
		net = get_net(nsproxy->net_ns);
	task_unlock(task);

	return net ? &net->ns : NULL;
}

static inline struct net *to_net_ns(struct ns_common *ns)
{
	return container_of(ns, struct net, ns);
}

static void netns_put(struct ns_common *ns)
{
	put_net(to_net_ns(ns));
}

static int netns_install(struct nsproxy *nsproxy, struct ns_common *ns)
{
	struct net *net = to_net_ns(ns);

	if (!ns_capable(net->user_ns, CAP_SYS_ADMIN) ||
	    !ns_capable(current_user_ns(), CAP_SYS_ADMIN))
		return -EPERM;

	put_net(nsproxy->net_ns);
	nsproxy->net_ns = get_net(net);
	return 0;
}

static struct user_namespace *netns_owner(struct ns_common *ns)
{
	return to_net_ns(ns)->user_ns;
}

const struct proc_ns_operations netns_operations = {
	.name		= "net",
	.type		= CLONE_NEWNET,
	.get		= netns_get,
	.put		= netns_put,
	.install	= netns_install,
	.owner		= netns_owner,
};
#endif
