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

struct scull_seg {
        void **buff;
        struct scull_seg *next;
};

struct scull_dev {
        struct scull_seg *lst;             /* Pointer to first segment */
        int cnt_segs;                      /* the current array nr_bfsize */
        int bufsize;                       /* the current bufsize nr_bfsize */
        unsigned long fpos;                /* amount of lst stored here */
        struct semaphore sem;              /* mutual exclusion semaphore */
        struct cdev devt;                  /* Char device structure */
        char devname[20];
        struct ldd_device ldev;
};

int major = 0;
int minor = 0;
int nr_devs = 4;	/* number of bare scull devices */
int nr_bfsize = 4;
int nr_segs = 3;

module_param(major, int, S_IRUGO);
module_param(minor, int, S_IRUGO);
module_param(nr_devs, int, S_IRUGO);
module_param(nr_bfsize, int, S_IRUGO);
module_param(nr_segs, int, S_IRUGO);

struct scull_dev *devlist;

int scull_trim(struct scull_dev *pdev)
{
	struct scull_seg *pseg;
	int i;

	for (pseg = pdev->lst; pseg; pseg = pseg->next) { /* all the list lst */
		if (pseg->buff) {
			for (i = 0; i < pdev->cnt_segs; i++)
				kfree(pseg->buff[i]);

			kfree(pseg->buff);
			pseg->buff = NULL;
		}
		kfree(pseg);
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

struct scull_seg *getsegbyidx(struct scull_dev *pdev, int n)
{
	struct scull_seg *pseg = pdev->lst;

	if (! pseg) { /* Allocate first size explicitly if need be */
		printk(KERN_ALERT "scull: kmalloc seg struct for seg %d\n", n);
		pseg = pdev->lst = kmalloc(sizeof(struct scull_seg), GFP_KERNEL);

		if (pseg == NULL)
			return NULL;  /* Never mind */

		memset(pseg, 0, sizeof(struct scull_seg));
	}

	while (n--) { /* Then follow the list */
		if (!pseg->next) {
			printk(KERN_ALERT "scull: kmalloc seg struct for seg %d\n", nr_segs - n);
			pseg->next = kmalloc(sizeof(struct scull_seg), GFP_KERNEL);

			if (pseg->next == NULL)
				return NULL;  /* Never mind */

			memset(pseg->next, 0, sizeof(struct scull_seg));
		}
		pseg = pseg->next;
		continue;
	}

	return pseg;
}

ssize_t scull_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct scull_dev *pdev = filp->private_data; 
	struct scull_seg *pseg;
	int buffsize = pdev->bufsize;
	int total_size = buffsize * pdev->cnt_segs;
	int seg_idx, curr_bytes, quotient, remainder;
	size_t retval = 0;

	if (down_interruptible(&pdev->sem))
		return -ERESTARTSYS;
	if (*f_pos >= pdev->fpos)
		goto out;
	if (*f_pos + count > pdev->fpos)
		count = pdev->fpos - *f_pos;

	seg_idx = (long)*f_pos / total_size; /* find item, size index and offset in the buffsize */
	curr_bytes = (long)*f_pos % total_size;
	quotient = curr_bytes / buffsize;
	remainder = curr_bytes % buffsize;

	printk(KERN_ALERT "scull: read seg_idx=%d, curr_bytes=%d, quotient=%d, remainder=%d, count=%d\n", seg_idx, curr_bytes, quotient, remainder, count);
	pseg = getsegbyidx(pdev, seg_idx);

	if (pseg == NULL || !pseg->buff || ! pseg->buff[quotient])
		goto out; /* don't fill holes */

	if (count > buffsize - remainder) /* read only up to the end of this buffsize */
		count = buffsize - remainder;

	if (copy_to_user(buf, pseg->buff[quotient] + remainder, count)) {
		retval = -EFAULT;
		goto out;
	}
	*f_pos += count;
	retval = count;
	printk(KERN_ALERT "scull: read count=%d, f_pos=%d, retval=%d\n", count, *f_pos, retval);

  out:
	up(&pdev->sem);
	return retval;
}

ssize_t scull_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	struct scull_dev *pdev = filp->private_data;
	struct scull_seg *pseg;
	int buffsize = pdev->bufsize;
	int total_size = buffsize * pdev->cnt_segs;
	int seg_idx, curr_bytes, quotient, remainder;
	size_t retval = -ENOMEM; /* value used in "goto out" statements */

	if (down_interruptible(&pdev->sem))
		return -ERESTARTSYS;

	seg_idx = (long)*f_pos / total_size; /* find listitem, size index and offset in the buffsize */
	curr_bytes = (long)*f_pos % total_size;
	quotient = curr_bytes / buffsize;
	remainder = curr_bytes % buffsize;
	printk(KERN_ALERT "scull: seg_idx=%d, curr_bytes=%d, quotient=%d, remainder=%d\n", seg_idx, curr_bytes, quotient, remainder);

	pseg = getsegbyidx(pdev, seg_idx);
	if (pseg == NULL)
		goto out;

	if (!pseg->buff) {
		printk(KERN_ALERT "scull: kmalloc %d buff memory for seg %d\n", pdev->cnt_segs, seg_idx);
		pseg->buff = kmalloc(pdev->cnt_segs * sizeof(char *), GFP_KERNEL);
		if (!pseg->buff)
			goto out;

		memset(pseg->buff, 0, pdev->cnt_segs * sizeof(char *));
	}

	if (!pseg->buff[quotient]) {
		printk(KERN_ALERT "scull: kmalloc %d bytes real buffer for seg %d, count=%d\n", buffsize, seg_idx, count);
		pseg->buff[quotient] = kmalloc(buffsize, GFP_KERNEL);
		if (!pseg->buff[quotient])
			goto out;
	}

	if (count > buffsize - remainder) /* write only up to the end of this buffsize */
		count = buffsize - remainder;

	if (copy_from_user(pseg->buff[quotient]+remainder, buf, count)) {
		retval = -EFAULT;
		goto out;
	}
	*f_pos += count;
	retval = count;

	if (pdev->fpos < *f_pos)
		pdev->fpos = *f_pos;

	printk(KERN_ALERT "scull: count=%d, f_pos=%d, retval=%d\n", count, *f_pos, retval);
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
	devno = MKDEV(major, minor);

	if (!devlist)
		return; /* nothing else to release */

	for (i = 0; i < nr_devs; i++) {
		scull_trim(devlist + i);
		unregister_ldd_device(&devlist[i].ldev);
		cdev_del(&devlist[i].devt);
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
		dv = MKDEV(major, minor);
		result = register_chrdev_region(dv, nr_devs, "scull");
	} else {
		result = alloc_chrdev_region(&dv, minor, nr_devs, "scull");
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
		init_MUTEX(&devlist[i].sem);
		int err, devno = MKDEV(major, minor + i);
    
		cdev_init(&devlist[i].devt, &scull_fops);
		devlist[i].devt.owner = THIS_MODULE;
		devlist[i].devt.ops = &scull_fops;
		err = cdev_add (&devlist[i].devt, devno, 1);
		if (err)
			printk(KERN_NOTICE "Error %d adding scull%d", err, i);

		scull_register_dev(devlist + i, i);
	}

	return 0;
  fail:
	scull_cleanup();
	return result;
}

module_init(scull_init);
module_exit(scull_cleanup);

MODULE_AUTHOR("Alessandro Rubini, Jonathan Corbet");
MODULE_LICENSE("Dual BSD/GPL");

