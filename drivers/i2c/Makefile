# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the i2c core.
#

obj-$(CONFIG_I2C_BOARDINFO)	+= i2c-boardinfo.o
obj-$(CONFIG_I2C)		+= i2c-core.o
i2c-core-objs 			:= i2c-core-base.o i2c-core-smbus.o
i2c-core-$(CONFIG_ACPI)		+= i2c-core-acpi.o

obj-$(CONFIG_I2C_CHARDEV)	+= i2c-dev.o
obj-y				+= algos/ busses/

ccflags-$(CONFIG_I2C_DEBUG_CORE) := -DDEBUG
CFLAGS_i2c-core-base.o := -Wno-deprecated-declarations
