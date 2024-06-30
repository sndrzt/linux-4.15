#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include "lddbus.h"

static char *Version = "$Revision: 1.9 $";

static int ldd_hotplug(struct device *dev, char **envp, int num_envp, char *buffer, int buffer_size) /* Respond to hotplug events. */
{
	envp[0] = buffer;
	if (snprintf(buffer, buffer_size, "LDDBUS_VERSION=%s", Version) >= buffer_size)
		return -ENOMEM;
	envp[1] = NULL;

	return 0;
}

static int ldd_uevent(struct device *dev, struct kobj_uevent_env *env) {
	env->envp[0] = env->buf;
	if (snprintf(env->buf, env->buflen, "LDDBUS_VERSION=%s", Version) >= env->buflen)
		return -ENOMEM;

	env->envp[1] = NULL;

	return 0;
}

static int ldd_match(struct device *dev, struct device_driver *driver) /* Match LDD devices to drivers.  Just do a simple name test. */
{
	return !strncmp(dev->init_name, driver->name, strlen(driver->name));
}

static void ldd_bus_release(struct device *dev) /* The LDD bus device. */
{
	printk(KERN_DEBUG "lddbus release\n");
}
	
struct device ldd_bus = {
	.init_name   = "ldd0",
	.release  = ldd_bus_release
};

struct bus_type ldd_bus_type = { /* And the bus type. */
	.name = "ldd",
	.match = ldd_match,
	.uevent = ldd_uevent,
};

static ssize_t show_bus_version(struct bus_type *bus, char *buf) /* Export a simple attribute. */
{
	return snprintf(buf, PAGE_SIZE, "%s\n", Version);
}

static BUS_ATTR(version, S_IRUGO, show_bus_version, NULL);

/* For now, no references to LDDbus devices go out which are not
 * tracked via the module reference count, so we use a no-op release function. */
static void ldd_dev_release(struct device *dev)
{
}

int register_ldd_device(struct ldd_device *ldddev)
{
	ldddev->dev.bus = &ldd_bus_type;
	ldddev->dev.parent = &ldd_bus;
	ldddev->dev.release = ldd_dev_release;
	strncpy(ldddev->dev.init_name, ldddev->name, 20);

	return device_register(&ldddev->dev);
}
EXPORT_SYMBOL(register_ldd_device);

void unregister_ldd_device(struct ldd_device *ldddev)
{
	device_unregister(&ldddev->dev);
}
EXPORT_SYMBOL(unregister_ldd_device);

static ssize_t show_version(struct device_driver *driver, char *buf) /* Crude driver interface. */
{
	struct ldd_driver *ldriver = to_ldd_driver(driver);

	sprintf(buf, "%s\n", ldriver->version);

	return strlen(buf);
}
		
int register_ldd_driver(struct ldd_driver *driver)
{
	int ret;
	
	driver->driver.bus = &ldd_bus_type;
	ret = driver_register(&driver->driver);

	if (ret)
		return ret;

	driver->version_attr.attr.name = "version";
	driver->version_attr.attr.mode = driver->module;
	driver->version_attr.attr.mode = S_IRUGO;
	driver->version_attr.show = show_version;
	driver->version_attr.store = NULL;
	return driver_create_file(&driver->driver, &driver->version_attr);
}
EXPORT_SYMBOL(register_ldd_driver);

void unregister_ldd_driver(struct ldd_driver *driver)
{
	driver_unregister(&driver->driver);
}
EXPORT_SYMBOL(unregister_ldd_driver);

static int __init ldd_bus_init(void)
{
	int ret;

	ret = bus_register(&ldd_bus_type);
	if (ret)
		return ret;

	if (bus_create_file(&ldd_bus_type, &bus_attr_version))
		printk(KERN_NOTICE "Unable to create version attribute\n");

	ret = device_register(&ldd_bus);
	if (ret)
		printk(KERN_NOTICE "Unable to register ldd0\n");

	return ret;
}

static void ldd_bus_exit(void)
{
	device_unregister(&ldd_bus);
	bus_unregister(&ldd_bus_type);
}

module_init(ldd_bus_init);
module_exit(ldd_bus_exit);

MODULE_AUTHOR("Jonathan Corbet");
MODULE_LICENSE("Dual BSD/GPL");

