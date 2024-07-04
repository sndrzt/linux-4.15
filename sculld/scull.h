#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include "lddbus.h"

/*
 * Macros to help debugging
 */
#undef PDEBUG             /* undef it, just in case */
#ifdef SCULL_DEBUG
#  ifdef __KERNEL__
     /* This one if debugging is on, and kernel space */
#    define PDEBUG(fmt, args...) printk( KERN_DEBUG "scull: " fmt, ## args)
#  else
     /* This one for user space */
#    define PDEBUG(fmt, args...) fprintf(stderr, fmt, ## args)
#  endif
#else
#  define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif

#undef PDEBUGG
#define PDEBUGG(fmt, args...) /* nothing: it's a placeholder */

#define SCULL_MAJOR 0   /* dynamic major by default */

#define SCULL_DEVS 4    /* scull0 through scull3 */

/*
 * The bare device is a variable-length region of memory.
 * Use a linked list of indirect blocks.
 *
 * "scull_dev->data" points to an array of pointers, each
 * pointer refers to a memory page.
 *
 * The array (quantum-set) is SCULL_QSET long.
 */
#define SCULL_ORDER    0 /* one page at a time */
#define SCULL_QSET     500

struct scull_dev {
	void **data;
	struct scull_dev *next;  /* next listitem */
	int vmas;                 /* active mappings */
	int order;                /* the current allocation order */
	int qset;                 /* the current array size */
	size_t size;              /* 32-bit will suffice */
	struct semaphore sem;     /* Mutual exclusion */
	struct cdev cdev;
	char devname[20];
	struct ldd_device ldev;
};

extern struct scull_dev *scull_devices;

extern struct file_operations scull_fops;

/*
 * The different configurable parameters
 */
extern int scull_major;     /* main.c */
extern int scull_devs;
extern int scull_order;
extern int scull_qset;

/*
 * Prototypes for shared functions
 */
int scull_trim(struct scull_dev *dev);
struct scull_dev *scull_follow(struct scull_dev *dev, int n);

#ifdef SCULL_DEBUG
#  define SCULL_USE_PROC
#endif

/*
 * Ioctl definitions
 */

/* Use 'K' as magic number */
#define SCULL_IOC_MAGIC  'K'

#define SCULL_IOCRESET    _IO(SCULL_IOC_MAGIC, 0)

/*
 * S means "Set" through a ptr,
 * T means "Tell" directly
 * G means "Get" (to a pointed var)
 * Q means "Query", response is on the return value
 * X means "eXchange": G and S atomically
 * H means "sHift": T and Q atomically
 */
#define SCULL_IOCSORDER   _IOW(SCULL_IOC_MAGIC,  1, int)
#define SCULL_IOCTORDER   _IO(SCULL_IOC_MAGIC,   2)
#define SCULL_IOCGORDER   _IOR(SCULL_IOC_MAGIC,  3, int)
#define SCULL_IOCQORDER   _IO(SCULL_IOC_MAGIC,   4)
#define SCULL_IOCXORDER   _IOWR(SCULL_IOC_MAGIC, 5, int)
#define SCULL_IOCHORDER   _IO(SCULL_IOC_MAGIC,   6)
#define SCULL_IOCSQSET    _IOW(SCULL_IOC_MAGIC,  7, int)
#define SCULL_IOCTQSET    _IO(SCULL_IOC_MAGIC,   8)
#define SCULL_IOCGQSET    _IOR(SCULL_IOC_MAGIC,  9, int)
#define SCULL_IOCQQSET    _IO(SCULL_IOC_MAGIC,  10)
#define SCULL_IOCXQSET    _IOWR(SCULL_IOC_MAGIC,11, int)
#define SCULL_IOCHQSET    _IO(SCULL_IOC_MAGIC,  12)

#define SCULL_IOC_MAXNR 12

