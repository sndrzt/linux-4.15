# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the i2c bus drivers.
#

obj-$(CONFIG_I2C_DESIGNWARE_CORE)	+= i2c-designware-core.o
i2c-designware-core-objs := i2c-designware-common.o i2c-designware-master.o
obj-$(CONFIG_I2C_DESIGNWARE_PLATFORM)	+= i2c-designware-platform.o
i2c-designware-platform-objs := i2c-designware-platdrv.o
i2c-designware-platform-$(CONFIG_I2C_DESIGNWARE_BAYTRAIL) += i2c-designware-baytrail.o
