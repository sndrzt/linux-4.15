#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* nr_bfsize_t */
#include <linux/cdev.h>
#include <asm/uaccess.h>	/* copy_*_user */
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/aio.h>
#include <linux/ioctl.h>

struct ldd_driver {
	char *version;
	struct module *module;
	struct device_driver driver;
	struct driver_attribute version_attr;
};

struct ldd_device {
	char *name;
	struct ldd_driver *driver;
	struct device dev;
};

extern int register_ldd_device(struct ldd_device *);
extern void unregister_ldd_device(struct ldd_device *);
extern int register_ldd_driver(struct ldd_driver *);
extern void unregister_ldd_driver(struct ldd_driver *);

struct scull_dev {
	void **data;
	struct scull_dev *lst;  /* lst listitem */
	int vmas;                 /* active mappings */
	int cnt_segs;                /* the current allocation cnt_segs */
	int bufsize;                 /* the current array size */
	size_t fpos;              /* 32-bit will suffice */
	struct semaphore sem;     /* Mutual exclusion */
	struct cdev devt;
	char devname[20];
	struct ldd_device ldev;
};

int major = 0;
int nr_devs = 4;	/* number of bare scull devices */
int nr_bfsize = 4;
int nr_segs = 3;

module_param(major, int, 0);
module_param(nr_devs, int, 0);
module_param(nr_bfsize, int, 0);
module_param(nr_segs, int, 0);

struct scull_dev *devlist;

int scull_trim(struct scull_dev *pdev)
{
	struct scull_dev *lst, *dptr;
	int bufsize = pdev->bufsize;   /* "dev" is not-null */
	int i;

	if (pdev->vmas) /* don't trim: there are active mappings */
		return -EBUSY;

	for (dptr = pdev; dptr; dptr = lst) { /* all the list items */
		if (dptr->data) {
			for (i = 0; i < bufsize; i++)
				if (dptr->data[i])
					free_pages((unsigned long)(dptr->data[i]), dptr->cnt_segs); /* This code frees a whole quantum-set */

			kfree(dptr->data);
			dptr->data=NULL;
		}
		lst=dptr->lst;
		if (dptr != pdev) kfree(dptr); /* all of them but the first */
	}
	pdev->fpos = 0;
	pdev->bufsize = nr_bfsize;
	pdev->cnt_segs = nr_segs;
	pdev->lst = NULL;

	return 0;
}

int scull_release(struct inode *inode, struct file *filp)
{
	return 0;
}

int scull_open(struct inode *inode, struct file *filp)
{
	struct scull_dev *pdev;

	pdev = container_of(inode->i_cdev, struct scull_dev, devt);
	filp->private_data = pdev;

	/* now trim to 0 the length of the device if open was write-only */
	if ( (filp->f_flags & O_ACCMODE) == O_WRONLY) {
		if (down_interruptible(&pdev->sem))
			return -ERESTARTSYS;

		scull_trim(pdev); /* ignore errors */

		up(&pdev->sem);
	}

	return 0;          /* success */
}

struct scull_dev *getsegbyidx(struct scull_dev *pdev, int n)
{
	while (n--) {
		if (!pdev->lst) {
			pdev->lst = kmalloc(sizeof(struct scull_dev), GFP_KERNEL);
			memset(pdev->lst, 0, sizeof(struct scull_dev));
		}
		pdev = pdev->lst;
		continue;
	}
	return pdev;
}

ssize_t scull_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct scull_dev *pdev = filp->private_data; 
	struct scull_dev *dptr;
	int cnt_segs = PAGE_SIZE << pdev->cnt_segs;
	int bufsize = pdev->bufsize;
	int itemsize = cnt_segs * bufsize; /* how many bytes in the listitem */
	int item, s_pos, q_pos, rest;
	ssize_t retval = 0;

	if (down_interruptible(&pdev->sem))
		return -ERESTARTSYS;
	if (*f_pos > pdev->fpos) 
		goto nothing;
	if (*f_pos + count > pdev->fpos)
		count = pdev->fpos - *f_pos;

	item = ((long) *f_pos) / itemsize; /* find listitem, bufsize index, and offset in the quantum */
	rest = ((long) *f_pos) % itemsize;
	s_pos = rest / cnt_segs; q_pos = rest % cnt_segs;

    	/* follow the list up to the right position (defined elsewhere) */
	dptr = getsegbyidx(pdev, item);

	if (!dptr->data)
		goto nothing; /* don't fill holes */
	if (!dptr->data[s_pos])
		goto nothing;
	if (count > cnt_segs - q_pos)
		count = cnt_segs - q_pos; /* read only up to the end of this quantum */

	if (copy_to_user (buf, dptr->data[s_pos]+q_pos, count)) {
		retval = -EFAULT;
		goto nothing;
	}
	up (&pdev->sem);

	*f_pos += count;
	return count;

  nothing:
	up(&pdev->sem);
	return retval;
}

ssize_t scull_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	struct scull_dev *pdev = filp->private_data;
	struct scull_dev *dptr;
	int cnt_segs = PAGE_SIZE << pdev->cnt_segs;
	int bufsize = pdev->bufsize;
	int itemsize = cnt_segs * bufsize;
	int item, s_pos, q_pos, rest;
	ssize_t retval = -ENOMEM; /* our most likely error */

	if (down_interruptible (&pdev->sem))
		return -ERESTARTSYS;

	item = ((long) *f_pos) / itemsize; /* find listitem, bufsize index and offset in the cnt_segs */
	rest = ((long) *f_pos) % itemsize;
	s_pos = rest / cnt_segs; q_pos = rest % cnt_segs;

	dptr = getsegbyidx(pdev, item); /* follow the list up to the right position */
	if (!dptr->data) {
		dptr->data = kmalloc(bufsize * sizeof(void *), GFP_KERNEL);
		if (!dptr->data)
			goto out;
		memset(dptr->data, 0, bufsize * sizeof(char *));
	}
	if (!dptr->data[s_pos]) {
		dptr->data[s_pos] = (void *)__get_free_pages(GFP_KERNEL, dptr->cnt_segs); /* allocation of a single quantum */
		if (!dptr->data[s_pos])
			goto out;
		memset(dptr->data[s_pos], 0, PAGE_SIZE << dptr->cnt_segs);
	}
	if (count > cnt_segs - q_pos)
		count = cnt_segs - q_pos; /* write only up to the end of this quantum */
	if (copy_from_user (dptr->data[s_pos]+q_pos, buf, count)) {
		retval = -EFAULT;
		goto out;
	}
	*f_pos += count;
 
	if (pdev->fpos < *f_pos) /* update the size */
		pdev->fpos = *f_pos;
	up (&pdev->sem);

	return count;
  out:
	up(&pdev->sem);

	return retval;
}

struct file_operations scull_fops = {
	.owner =    THIS_MODULE,
	.read =     scull_read,
	.write =    scull_write,
	.open =     scull_open,
	.release =  scull_release,
};

dev_t devno;

static void scull_setup_cdev(struct scull_dev *pdev, int index)
{
	int err, devno = MKDEV(major, index);
    
	cdev_init(&pdev->devt, &scull_fops);
	pdev->devt.owner = THIS_MODULE;
	pdev->devt.ops = &scull_fops;
	err = cdev_add (&pdev->devt, devno, 1);
	/* Fail gracefully if need be */
	if (err)
		printk(KERN_NOTICE "Error %d adding scull%d", err, index);
}

static ssize_t scull_show_dev(struct device *ddev, char *buf)
{
	struct scull_dev *dev = dev_get_drvdata(ddev);

	return print_dev_t(buf, dev->devt.dev);
}

static DEVICE_ATTR(dev, S_IRUGO, scull_show_dev, NULL);

static struct ldd_driver scull_driver = { /* Device model stuff */
	.version = "$Revision: 1.21 $",
	.module = THIS_MODULE,
	.driver = {
		.name = "scull",
	},
};

static void scull_register_dev(struct scull_dev *dev, int index)
{
	sprintf(dev->devname, "scull%d", index);
	dev->ldev.name = dev->devname;
	dev->ldev.driver = &scull_driver;
	dev_set_drvdata(&dev->ldev.dev, dev);
	register_ldd_device(&dev->ldev);
	device_create_file(&dev->ldev.dev, &dev_attr_dev);
}

void scull_cleanup(void)
{
	int i;
	devno = MKDEV(major, 0);

	for (i = 0; i < nr_devs; i++) {
		unregister_ldd_device(&devlist[i].ldev);
		cdev_del(&devlist[i].devt);
		scull_trim(devlist + i);
	}
	kfree(devlist);
	unregister_ldd_driver(&scull_driver);
	unregister_chrdev_region(devno, nr_devs);
}

int scull_init(void)
{
	int result, i;
	dev_t dv = 0;

	if (major) {
		dv = MKDEV(major, 0);
		result = register_chrdev_region(dv, nr_devs, "scull");
	} else {
		result = alloc_chrdev_region(&dv, 0, nr_devs, "scull");
		major = MAJOR(dv);
	}

	if (result < 0) {
		printk(KERN_WARNING "scull: can't get major %d\n", major);

		return result;
	}

	register_ldd_driver(&scull_driver);
	
	devlist = kmalloc(nr_devs * sizeof(struct scull_dev), GFP_KERNEL);
	if (!devlist) {
		result = -ENOMEM;
		goto fail;
	}

	memset(devlist, 0, nr_devs * sizeof(struct scull_dev));

	for (i = 0; i < nr_devs; i++) {
		devlist[i].bufsize = nr_bfsize;
		devlist[i].cnt_segs = nr_segs;
		sema_init (&devlist[i].sem, 1);
		scull_setup_cdev(devlist + i, i);
		scull_register_dev(devlist + i, i);
	}

	return 0;
  fail:
	unregister_chrdev_region(dv, nr_devs);
	return result;
}

module_init(scull_init);
module_exit(scull_cleanup);

MODULE_AUTHOR("Alessandro Rubini, Jonathan Corbet");
MODULE_LICENSE("Dual BSD/GPL");

