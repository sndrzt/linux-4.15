#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/poll.h>

struct scull_dev {
        wait_queue_head_t iwq, owq;       /* read and write queues */
        char *buff, *tail;                /* begin of buf, tail of buf */
        int size;                    /* used in pointer arithmetic */
        char *rp, *wp;                     /* where to read, where to write */
        int r_cnt, w_cnt;            /* number of openings for r/w */
        struct fasync_struct *asynq; /* asynchronous readers */
        struct semaphore sem;              /* mutual exclusion semaphore */
        struct cdev devt;                  /* Char device structure */
};

int major =   0;
int nr_devs = 4;	/* number of pipe devices */
int nr_bfsize =  4000;	/* buff size */

module_param(major, int, S_IRUGO);
module_param(nr_devs, int, 0);
module_param(nr_bfsize, int, 0);

int scull_fasync(int fd, struct file *filp, int mode)
{
	struct scull_dev *pdev = filp->private_data;
	return fasync_helper(fd, filp, mode, &pdev->asynq);
}

int scull_open(struct inode *inode, struct file *filp)
{
	struct scull_dev *pdev;

	pdev = container_of(inode->i_cdev, struct scull_dev, devt);
	filp->private_data = pdev;

	if (down_interruptible(&pdev->sem))
		return -ERESTARTSYS;

	if (!pdev->buff) {
		pdev->buff = kmalloc(nr_bfsize, GFP_KERNEL);
		if (!pdev->buff) {
			up(&pdev->sem);
			return -ENOMEM;
		}
	}

	pdev->size = nr_bfsize;
	pdev->tail = pdev->buff + pdev->size;
	pdev->rp = pdev->wp = pdev->buff; /* rd and wr from the beginning */

	if (filp->f_mode & FMODE_READ) /* use f_mode,not  f_flags: it's cleaner (fs/open.c tells why) */
		pdev->r_cnt++;
	if (filp->f_mode & FMODE_WRITE)
		pdev->w_cnt++;

	up(&pdev->sem);

	return nonseekable_open(inode, filp);
}

int scull_release(struct inode *inode, struct file *filp)
{
	struct scull_dev *pdev = filp->private_data;

	scull_fasync(-1, filp, 0); /* remove this filp from the asynchronously notified filp's */

	down(&pdev->sem);

	if (filp->f_mode & FMODE_READ)
		pdev->r_cnt--;
	if (filp->f_mode & FMODE_WRITE)
		pdev->w_cnt--;
	if (pdev->r_cnt + pdev->w_cnt == 0) {
		kfree(pdev->buff);
		pdev->buff = NULL; /* the other fields are not checked on open */
	}

	up(&pdev->sem);

	return 0;
}

ssize_t scull_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct scull_dev *pdev = filp->private_data;

	if (down_interruptible(&pdev->sem))
		return -ERESTARTSYS;

	while (pdev->rp == pdev->wp) { /* nothing to read */
		up(&pdev->sem); /* release the lock */

		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		printk("\"%s\" reading: going to sleep\n", current->comm);

		if (wait_event_interruptible(pdev->iwq, (pdev->rp != pdev->wp)))
			return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
		if (down_interruptible(&pdev->sem)) /* otherwise loop, but first reacquire the lock */
			return -ERESTARTSYS;
	}

	if (pdev->wp > pdev->rp) /* ok, data is there, return something */
		count = min(count, (size_t)(pdev->wp - pdev->rp));
	else /* the write pointer has wrapped, return data up to pdev->tail */
		count = min(count, (size_t)(pdev->tail - pdev->rp));

	if (copy_to_user(buf, pdev->rp, count)) {
		up (&pdev->sem);
		return -EFAULT;
	}

	pdev->rp += count;

	if (pdev->rp == pdev->tail)
		pdev->rp = pdev->buff; /* wrapped */

	up (&pdev->sem);

	wake_up_interruptible(&pdev->owq); /* finally, awake any writers and return */
	printk("\"%s\" did read %li bytes\n", current->comm, (long)count);

	return count;
}

int spacefree(struct scull_dev *pdev)
{
	if (pdev->rp == pdev->wp)
		return pdev->size - 1;

	return ((pdev->rp + pdev->size - pdev->wp) % pdev->size) - 1;
}

/* Wait for space for writing; caller must hold device semaphore.  On
 * error the semaphore will be released before returning. */
int scull_getwritespace(struct scull_dev *pdev, struct file *filp)
{
	while (spacefree(pdev) == 0) { /* full */
		DEFINE_WAIT(wait);
		
		up(&pdev->sem);

		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		printk("\"%s\" writing: going to sleep\n", current->comm);
		prepare_to_wait(&pdev->owq, &wait, TASK_INTERRUPTIBLE);

		if (spacefree(pdev) == 0)
			schedule();

		finish_wait(&pdev->owq, &wait);

		if (signal_pending(current))
			return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
		if (down_interruptible(&pdev->sem))
			return -ERESTARTSYS;
	}
	return 0;
}	

ssize_t scull_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	struct scull_dev *pdev = filp->private_data;
	int result;

	if (down_interruptible(&pdev->sem))
		return -ERESTARTSYS;

	result = scull_getwritespace(pdev, filp); /* Make sure there's space to write */
	if (result)
		return result; /* scull_getwritespace called up(&pdev->sem) */

	count = min(count, (size_t)spacefree(pdev)); /* ok, space is there, accept something */

	if (pdev->wp >= pdev->rp)
		count = min(count, (size_t)(pdev->tail - pdev->wp)); /* to tail-of-buf */
	else /* the write pointer has wrapped, fill up to rp-1 */
		count = min(count, (size_t)(pdev->rp - pdev->wp - 1));

	printk("Going to accept %li bytes to %p from %p\n", (long)count, pdev->wp, buf);

	if (copy_from_user(pdev->wp, buf, count)) {
		up (&pdev->sem);

		return -EFAULT;
	}

	pdev->wp += count;

	if (pdev->wp == pdev->tail)
		pdev->wp = pdev->buff; /* wrapped */

	up(&pdev->sem);

	wake_up_interruptible(&pdev->iwq);  /* finally, awake any reader, blocked in read() and select() */

	if (pdev->asynq) /* and signal asynchronous readers, explained late in chapter 5 */
		kill_fasync(&pdev->asynq, SIGIO, POLL_IN);

	printk("\"%s\" did write %li bytes\n", current->comm, (long)count);

	return count;
}

unsigned int scull_poll(struct file *filp, poll_table *wait)
{
	struct scull_dev *pdev = filp->private_data;
	unsigned int mask = 0;

	down(&pdev->sem);
	poll_wait(filp, &pdev->iwq,  wait);
	poll_wait(filp, &pdev->owq, wait);

	if (pdev->rp != pdev->wp)
		mask |= POLLIN | POLLRDNORM;	/* readable */
	if (spacefree(pdev))
		mask |= POLLOUT | POLLWRNORM;	/* writable */

	up(&pdev->sem);

	return mask;
}

struct file_operations scull_dev_fops = {
	.owner =	THIS_MODULE,
	.read =		scull_read,
	.write =	scull_write,
	.poll =		scull_poll,
	.open =		scull_open,
	.release =	scull_release,
	.fasync =	scull_fasync,
};

dev_t devno;
struct scull_dev *devlist;
void scull_cleanup(void)
{
	int i;

	if (!devlist)
		return; /* nothing else to release */

	for (i = 0; i < nr_devs; i++) {
		cdev_del(&devlist[i].devt);
		kfree(devlist[i].buff);
	}

	kfree(devlist);
	unregister_chrdev_region(devno, nr_devs);
	devlist = NULL; /* pedantic */
}

int scull_init(void)
{
	int result, i;
	int err;

	devno = MKDEV(major, nr_devs);

	result = register_chrdev_region(devno, nr_devs, "scullp");
	if (result < 0) {
		printk(KERN_NOTICE "Unable to get scullp region, error %d\n", result);
		return 0;
	}

	devlist = kmalloc(nr_devs * sizeof(struct scull_dev), GFP_KERNEL);

	if (devlist == NULL) {
		unregister_chrdev_region(devno, nr_devs);
		return 0;
	}
	memset(devlist, 0, nr_devs * sizeof(struct scull_dev));
	for (i = 0; i < nr_devs; i++) {
		init_waitqueue_head(&(devlist[i].iwq));
		init_waitqueue_head(&(devlist[i].owq));
		init_MUTEX(&devlist[i].sem);
    
		cdev_init(&(devlist+i)->devt, &scull_dev_fops);
		(devlist+i)->devt.owner = THIS_MODULE;
		err = cdev_add (&(devlist+i)->devt, devno + i, 1);

		if (err)
			printk(KERN_NOTICE "Error %d adding scullpipe%d", err, i);
	}

	return 0;
}

module_init(scull_init);
module_exit(scull_cleanup);

MODULE_AUTHOR("Alessandro Rubini, Jonathan Corbet");
MODULE_LICENSE("Dual BSD/GPL");

