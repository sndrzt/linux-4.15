# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the industrial I/O core.
#

obj-$(CONFIG_IIO) += industrialio.o
industrialio-y := industrialio-core.o
industrialio-$(CONFIG_IIO_BUFFER) += industrialio-buffer.o
#industrialio-$(CONFIG_IIO_TRIGGER) += industrialio-trigger.o

#obj-$(CONFIG_IIO_CONFIGFS) += industrialio-configfs.o
#obj-$(CONFIG_IIO_SW_DEVICE) += industrialio-sw-device.o
#obj-$(CONFIG_IIO_SW_TRIGGER) += industrialio-sw-trigger.o
#obj-$(CONFIG_IIO_TRIGGERED_EVENT) += industrialio-triggered-event.o

obj-y += accel/
