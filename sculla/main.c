#include <linux/config.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/cdev.h>
#include <asm/uaccess.h>	/* copy_*_user */
#include <linux/tty.h>

struct scull_qset {
        void **data;
        struct scull_qset *next;
};

struct scull_dev {
        struct scull_qset *data;           /* Pointer to first quantum set */
        int quantum;                       /* the current quantum size */
        int qset;                          /* the current array size */
        unsigned long size;                /* amount of data stored here */
        struct semaphore sem;              /* mutual exclusion semaphore */
        struct cdev cdev;                  /* Char device structure */
        unsigned int access_key;           /* used by sculluid and scullpriv */
};

#define TYPE(minor)	(((minor) >> 4) & 0xf)	/* high nibble */
#define NUM(minor)	((minor) & 0xf)		/* low  nibble */

#define SCULL_IOC_MAGIC  'k'

#define SCULL_IOCRESET    _IO(SCULL_IOC_MAGIC, 0)

/*
 * S means "Set" through a ptr,
 * T means "Tell" directly with the argument value
 * G means "Get": reply by setting through a pointer
 * Q means "Query": response is on the return value
 * X means "eXchange": switch G and S atomically
 * H means "sHift": switch T and Q atomically
 */
#define SCULL_IOCSQUANTUM _IOW(SCULL_IOC_MAGIC,  1, int)
#define SCULL_IOCSQSET    _IOW(SCULL_IOC_MAGIC,  2, int)
#define SCULL_IOCTQUANTUM _IO(SCULL_IOC_MAGIC,   3)
#define SCULL_IOCTQSET    _IO(SCULL_IOC_MAGIC,   4)
#define SCULL_IOCGQUANTUM _IOR(SCULL_IOC_MAGIC,  5, int)
#define SCULL_IOCGQSET    _IOR(SCULL_IOC_MAGIC,  6, int)
#define SCULL_IOCQQUANTUM _IO(SCULL_IOC_MAGIC,   7)
#define SCULL_IOCQQSET    _IO(SCULL_IOC_MAGIC,   8)
#define SCULL_IOCXQUANTUM _IOWR(SCULL_IOC_MAGIC, 9, int)
#define SCULL_IOCXQSET    _IOWR(SCULL_IOC_MAGIC,10, int)
#define SCULL_IOCHQUANTUM _IO(SCULL_IOC_MAGIC,  11)
#define SCULL_IOCHQSET    _IO(SCULL_IOC_MAGIC,  12)

/*
 * The other entities only have "Tell" and "Query", because they're
 * not printed in the book, and there's no need to have all six.
 * (The previous stuff was only there to show different ways to do it.
 */
#define SCULL_P_IOCTSIZE _IO(SCULL_IOC_MAGIC,   13)
#define SCULL_P_IOCQSIZE _IO(SCULL_IOC_MAGIC,   14)

#define SCULL_IOC_MAXNR 14

int scull_major =   0;
int scull_minor =   0;
int scull_nr_devs = 4;	/* number of bare scull devices */
int scull_quantum = 4000;
int scull_qset =    1000;

module_param(scull_major, int, S_IRUGO);
module_param(scull_minor, int, S_IRUGO);
module_param(scull_nr_devs, int, S_IRUGO);
module_param(scull_quantum, int, S_IRUGO);
module_param(scull_qset, int, S_IRUGO);

struct scull_dev *scull_devices;

int scull_trim(struct scull_dev *dev)
{
	struct scull_qset *next, *dptr;
	int qset = dev->qset;   /* "dev" is not-null */
	int i;

	for (dptr = dev->data; dptr; dptr = next) { /* all the list items */
		if (dptr->data) {
			for (i = 0; i < qset; i++)
				kfree(dptr->data[i]);
			kfree(dptr->data);
			dptr->data = NULL;
		}
		next = dptr->next;
		kfree(dptr);
	}
	dev->size = 0;
	dev->quantum = scull_quantum;
	dev->qset = scull_qset;
	dev->data = NULL;

	return 0;
}

struct scull_qset *scull_follow(struct scull_dev *dev, int n)
{
	struct scull_qset *qs = dev->data;

	if (! qs) { /* Allocate first qset explicitly if need be */
		qs = dev->data = kmalloc(sizeof(struct scull_qset), GFP_KERNEL);
		if (qs == NULL)
			return NULL;  /* Never mind */
		memset(qs, 0, sizeof(struct scull_qset));
	}

	while (n--) { /* Then follow the list */
		if (!qs->next) {
			qs->next = kmalloc(sizeof(struct scull_qset), GFP_KERNEL);
			if (qs->next == NULL)
				return NULL;  /* Never mind */
			memset(qs->next, 0, sizeof(struct scull_qset));
		}
		qs = qs->next;
		continue;
	}
	return qs;
}

ssize_t scull_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct scull_dev *dev = filp->private_data; 
	struct scull_qset *dptr;	/* the first listitem */
	int quantum = dev->quantum, qset = dev->qset;
	int itemsize = quantum * qset; /* how many bytes in the listitem */
	int item, s_pos, q_pos, rest;
	ssize_t retval = 0;

	if (down_interruptible(&dev->sem))
		return -ERESTARTSYS;

	if (*f_pos >= dev->size)
		goto out;

	if (*f_pos + count > dev->size)
		count = dev->size - *f_pos;

	item = (long)*f_pos / itemsize; /* find item, qset index and offset in the quantum */
	rest = (long)*f_pos % itemsize;
	s_pos = rest / quantum; q_pos = rest % quantum;

	dptr = scull_follow(dev, item); /* follow the list up to the right position */

	if (dptr == NULL || !dptr->data || ! dptr->data[s_pos])
		goto out; /* don't fill holes */

	if (count > quantum - q_pos) /* read only up to the end of this quantum */
		count = quantum - q_pos;

	if (copy_to_user(buf, dptr->data[s_pos] + q_pos, count)) {
		retval = -EFAULT;
		goto out;
	}
	*f_pos += count;
	retval = count;

  out:
	up(&dev->sem);
	return retval;
}

ssize_t scull_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	struct scull_dev *dev = filp->private_data;
	struct scull_qset *dptr;
	int quantum = dev->quantum, qset = dev->qset;
	int itemsize = quantum * qset;
	int item, s_pos, q_pos, rest;
	ssize_t retval = -ENOMEM; /* value used in "goto out" statements */

	if (down_interruptible(&dev->sem))
		return -ERESTARTSYS;

	item = (long)*f_pos / itemsize; /* find listitem, qset index and offset in the quantum */
	rest = (long)*f_pos % itemsize;
	s_pos = rest / quantum; q_pos = rest % quantum;

	dptr = scull_follow(dev, item); /* follow the list up to the right position */
	if (dptr == NULL)
		goto out;

	if (!dptr->data) {
		dptr->data = kmalloc(qset * sizeof(char *), GFP_KERNEL);
		if (!dptr->data)
			goto out;
		memset(dptr->data, 0, qset * sizeof(char *));
	}
	if (!dptr->data[s_pos]) {
		dptr->data[s_pos] = kmalloc(quantum, GFP_KERNEL);
		if (!dptr->data[s_pos])
			goto out;
	}

	if (count > quantum - q_pos) /* write only up to the end of this quantum */
		count = quantum - q_pos;

	if (copy_from_user(dptr->data[s_pos]+q_pos, buf, count)) {
		retval = -EFAULT;
		goto out;
	}
	*f_pos += count;
	retval = count;

	if (dev->size < *f_pos)
		dev->size = *f_pos;

  out:
	up(&dev->sem);
	return retval;
}

int scull_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0, tmp;
	int retval = 0;
    
	if (_IOC_TYPE(cmd) != SCULL_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > SCULL_IOC_MAXNR) return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;

	switch(cmd) {

	  case SCULL_IOCRESET:
		scull_quantum = 4000;
		scull_qset = 1000;
		break;
        
	  case SCULL_IOCSQUANTUM: /* Set: arg points to the value */
		if (! capable (CAP_SYS_ADMIN))
			return -EPERM;
		retval = __get_user(scull_quantum, (int __user *)arg);
		break;

	  case SCULL_IOCTQUANTUM: /* Tell: arg is the value */
		if (! capable (CAP_SYS_ADMIN))
			return -EPERM;
		scull_quantum = arg;
		break;

	  case SCULL_IOCGQUANTUM: /* Get: arg is pointer to result */
		retval = __put_user(scull_quantum, (int __user *)arg);
		break;

	  case SCULL_IOCQQUANTUM: /* Query: return it (it's positive) */
		return scull_quantum;

	  case SCULL_IOCXQUANTUM: /* eXchange: use arg as pointer */
		if (! capable (CAP_SYS_ADMIN))
			return -EPERM;
		tmp = scull_quantum;
		retval = __get_user(scull_quantum, (int __user *)arg);
		if (retval == 0)
			retval = __put_user(tmp, (int __user *)arg);
		break;

	  case SCULL_IOCHQUANTUM: /* sHift: like Tell + Query */
		if (! capable (CAP_SYS_ADMIN))
			return -EPERM;
		tmp = scull_quantum;
		scull_quantum = arg;
		return tmp;
        
	  case SCULL_IOCSQSET:
		if (! capable (CAP_SYS_ADMIN))
			return -EPERM;
		retval = __get_user(scull_qset, (int __user *)arg);
		break;

	  case SCULL_IOCTQSET:
		if (! capable (CAP_SYS_ADMIN))
			return -EPERM;
		scull_qset = arg;
		break;

	  case SCULL_IOCGQSET:
		retval = __put_user(scull_qset, (int __user *)arg);
		break;

	  case SCULL_IOCQQSET:
		return scull_qset;

	  case SCULL_IOCXQSET:
		if (! capable (CAP_SYS_ADMIN))
			return -EPERM;
		tmp = scull_qset;
		retval = __get_user(scull_qset, (int __user *)arg);
		if (retval == 0)
			retval = put_user(tmp, (int __user *)arg);
		break;

	  case SCULL_IOCHQSET:
		if (! capable (CAP_SYS_ADMIN))
			return -EPERM;
		tmp = scull_qset;
		scull_qset = arg;
		return tmp;

	  default:  /* redundant, as cmd was checked against MAXNR */
		return -ENOTTY;
	}

	return retval;
}

loff_t scull_llseek(struct file *filp, loff_t off, int whence)
{
	struct scull_dev *dev = filp->private_data;
	loff_t newpos;

	switch(whence) {
	  case 0: /* SEEK_SET */
		newpos = off;
		break;

	  case 1: /* SEEK_CUR */
		newpos = filp->f_pos + off;
		break;

	  case 2: /* SEEK_END */
		newpos = dev->size + off;
		break;

	  default: /* can't happen */
		return -EINVAL;
	}
	if (newpos < 0) return -EINVAL;
	filp->f_pos = newpos;
	return newpos;
}

static dev_t scull_a_firstdev;  /* Where our range begins */
static struct scull_dev scull_s_device;
static atomic_t scull_s_available = ATOMIC_INIT(1);

static int scull_s_open(struct inode *inode, struct file *filp)
{
	struct scull_dev *dev = &scull_s_device; /* device information */

	if (! atomic_dec_and_test (&scull_s_available)) {
		atomic_inc(&scull_s_available);
		return -EBUSY; /* already open */
	}

	if ( (filp->f_flags & O_ACCMODE) == O_WRONLY) /* then, everything else is copied from the bare scull device */
		scull_trim(dev);
	filp->private_data = dev;

	return 0;
}

static int scull_s_release(struct inode *inode, struct file *filp)
{
	atomic_inc(&scull_s_available); /* release the device */
	return 0;
}

struct file_operations scull_sngl_fops = {
	.owner =	THIS_MODULE,
	.llseek =     	scull_llseek,
	.read =       	scull_read,
	.write =      	scull_write,
	.ioctl =      	scull_ioctl,
	.open =       	scull_s_open,
	.release =    	scull_s_release,
};

static struct scull_dev scull_u_device;
static int scull_u_count;	/* initialized to 0 by default */
static uid_t scull_u_owner;	/* initialized to 0 by default */
static spinlock_t scull_u_lock = SPIN_LOCK_UNLOCKED;

static int scull_u_open(struct inode *inode, struct file *filp)
{
	struct scull_dev *dev = &scull_u_device; /* device information */

	spin_lock(&scull_u_lock);
	if (scull_u_count && 
			(scull_u_owner != current->uid) &&  /* allow user */
			(scull_u_owner != current->euid) && /* allow whoever did su */
			!capable(CAP_DAC_OVERRIDE)) { /* still allow root */
		spin_unlock(&scull_u_lock);
		return -EBUSY;   /* -EPERM would confuse the user */
	}

	if (scull_u_count == 0)
		scull_u_owner = current->uid; /* grab it */

	scull_u_count++;
	spin_unlock(&scull_u_lock);

	if ((filp->f_flags & O_ACCMODE) == O_WRONLY) /* then, everything else is copied from the bare scull device */
		scull_trim(dev);

	filp->private_data = dev;

	return 0;
}

static int scull_u_release(struct inode *inode, struct file *filp)
{
	spin_lock(&scull_u_lock);
	scull_u_count--; /* nothing else */
	spin_unlock(&scull_u_lock);
	return 0;
}

struct file_operations scull_user_fops = {
	.owner =      THIS_MODULE,
	.llseek =     scull_llseek,
	.read =       scull_read,
	.write =      scull_write,
	.ioctl =      scull_ioctl,
	.open =       scull_u_open,
	.release =    scull_u_release,
};

static struct scull_dev scull_w_device;
static int scull_w_count;	/* initialized to 0 by default */
static uid_t scull_w_owner;	/* initialized to 0 by default */
static DECLARE_WAIT_QUEUE_HEAD(scull_w_wait);
static spinlock_t scull_w_lock = SPIN_LOCK_UNLOCKED;

static inline int scull_w_available(void)
{
	return scull_w_count == 0 ||
		scull_w_owner == current->uid ||
		scull_w_owner == current->euid ||
		capable(CAP_DAC_OVERRIDE);
}

static int scull_w_open(struct inode *inode, struct file *filp)
{
	struct scull_dev *dev = &scull_w_device; /* device information */

	spin_lock(&scull_w_lock);
	while (! scull_w_available()) {
		spin_unlock(&scull_w_lock);
		if (filp->f_flags & O_NONBLOCK) return -EAGAIN;
		if (wait_event_interruptible (scull_w_wait, scull_w_available()))
			return -ERESTARTSYS; /* tell the fs layer to handle it */
		spin_lock(&scull_w_lock);
	}
	if (scull_w_count == 0)
		scull_w_owner = current->uid; /* grab it */
	scull_w_count++;
	spin_unlock(&scull_w_lock);

	if ((filp->f_flags & O_ACCMODE) == O_WRONLY) /* then, everything else is copied from the bare scull device */
		scull_trim(dev);
	filp->private_data = dev;

	return 0;
}

static int scull_w_release(struct inode *inode, struct file *filp)
{
	int temp;

	spin_lock(&scull_w_lock);
	scull_w_count--;
	temp = scull_w_count;
	spin_unlock(&scull_w_lock);

	if (temp == 0)
		wake_up_interruptible_sync(&scull_w_wait); /* awake other uid's */
	return 0;
}

struct file_operations scull_wusr_fops = {
	.owner =      THIS_MODULE,
	.llseek =     scull_llseek,
	.read =       scull_read,
	.write =      scull_write,
	.ioctl =      scull_ioctl,
	.open =       scull_w_open,
	.release =    scull_w_release,
};

struct scull_listitem {
	struct scull_dev device;
	dev_t key;
	struct list_head list;
};

static LIST_HEAD(scull_c_list); /* The list of devices, and a lock to protect it */
static spinlock_t scull_c_lock = SPIN_LOCK_UNLOCKED;

static struct scull_dev scull_c_device;   /* A placeholder scull_dev which really just holds the cdev stuff. */

static struct scull_dev *scull_c_lookfor_device(dev_t key) /* Look for a device or create one if missing */
{
	struct scull_listitem *lptr;

	list_for_each_entry(lptr, &scull_c_list, list) {
		if (lptr->key == key)
			return &(lptr->device);
	}

	lptr = kmalloc(sizeof(struct scull_listitem), GFP_KERNEL); /* not found */
	if (!lptr)
		return NULL;

	memset(lptr, 0, sizeof(struct scull_listitem)); /* initialize the device */
	lptr->key = key;
	scull_trim(&(lptr->device)); /* initialize it */
	init_MUTEX(&(lptr->device.sem));

	list_add(&lptr->list, &scull_c_list); /* place it in the list */

	return &(lptr->device);
}

static int scull_c_open(struct inode *inode, struct file *filp)
{
	struct scull_dev *dev;
	dev_t key;
 
	if (!current->signal->tty) { 
		printk("Process \"%s\" has no ctl tty\n", current->comm);
		return -EINVAL;
	}
	key = tty_devnum(current->signal->tty);

	spin_lock(&scull_c_lock); /* look for a scullc device in the list */
	dev = scull_c_lookfor_device(key);
	spin_unlock(&scull_c_lock);

	if (!dev)
		return -ENOMEM;

	if ( (filp->f_flags & O_ACCMODE) == O_WRONLY) /* then, everything else is copied from the bare scull device */
		scull_trim(dev);
	filp->private_data = dev;
	return 0;
}

static int scull_c_release(struct inode *inode, struct file *filp)
{
	return 0; /* Nothing to do, because the device is persistent. A `real' cloned device should be freed on last close */
}

struct file_operations scull_priv_fops = {
	.owner =    THIS_MODULE,
	.read =     scull_read,
	.write =    scull_write,
	.open =     scull_c_open,
	.release =  scull_c_release,
	.llseek =   scull_llseek,
	.ioctl =    scull_ioctl,
};

static struct scull_adev_info {
	char *name;
	struct scull_dev *sculldev;
	struct file_operations *fops;
} scull_access_devs[] = {
	{ "scullsingle", &scull_s_device, &scull_sngl_fops },
	{ "sculluid", &scull_u_device, &scull_user_fops },
	{ "scullwuid", &scull_w_device, &scull_wusr_fops },
	{ "sullpriv", &scull_c_device, &scull_priv_fops }
};

#define SCULL_N_ADEVS 4

static void scull_access_setup (dev_t devno, struct scull_adev_info *devinfo)
{
	struct scull_dev *dev = devinfo->sculldev;
	int err;

	dev->quantum = scull_quantum; /* Initialize the device structure */
	dev->qset = scull_qset;
	init_MUTEX(&dev->sem);

	cdev_init(&dev->cdev, devinfo->fops); /* Do the cdev stuff. */
	kobject_set_name(&dev->cdev.kobj, devinfo->name);
	dev->cdev.owner = THIS_MODULE;
	err = cdev_add (&dev->cdev, devno, 1);
 
	if (err) { /* Fail gracefully if need be */
		printk(KERN_NOTICE "Error %d adding %s\n", err, devinfo->name);
		kobject_put(&dev->cdev.kobj);
	} else
		printk(KERN_NOTICE "%s registered at %x\n", devinfo->name, devno);
}

int scull_access_init(dev_t firstdev)
{
	int result, i;

	result = register_chrdev_region (firstdev, SCULL_N_ADEVS, "sculla"); /* Get our number space */
	if (result < 0) {
		printk(KERN_WARNING "sculla: device number registration failed\n");
		return 0;
	}
	scull_a_firstdev = firstdev;

	for (i = 0; i < SCULL_N_ADEVS; i++) /* Set up each device. */
		scull_access_setup (firstdev + i, scull_access_devs + i);

	return SCULL_N_ADEVS;
}

void scull_cleanup(void)
{
	struct scull_listitem *lptr, *next;
	int i;

	for (i = 0; i < SCULL_N_ADEVS; i++) { /* Clean up the static devs */
		struct scull_dev *dev = scull_access_devs[i].sculldev;
		cdev_del(&dev->cdev);
		scull_trim(scull_access_devs[i].sculldev);
	}

	list_for_each_entry_safe(lptr, next, &scull_c_list, list) { /* And all the cloned devices */
		list_del(&lptr->list);
		scull_trim(&(lptr->device));
		kfree(lptr);
	}

	unregister_chrdev_region(scull_a_firstdev, SCULL_N_ADEVS); /* Free up our number space */

	return;
}

int scull_init(void)
{
	int result;
	dev_t dev = 0;

	if (scull_major) {
		dev = MKDEV(scull_major, scull_minor);
		result = register_chrdev_region(dev, scull_nr_devs, "scull");
	} else {
		result = alloc_chrdev_region(&dev, scull_minor, scull_nr_devs, "scull");
		scull_major = MAJOR(dev);
	}

	if (result < 0) {
		printk(KERN_WARNING "scull: can't get major %d\n", scull_major);

		return result;
	}

	dev = MKDEV(scull_major, scull_minor + scull_nr_devs);
	dev += scull_access_init(dev);

	return 0;
}

module_init(scull_init);
module_exit(scull_cleanup);

MODULE_AUTHOR("Alessandro Rubini, Jonathan Corbet");
MODULE_LICENSE("Dual BSD/GPL");


