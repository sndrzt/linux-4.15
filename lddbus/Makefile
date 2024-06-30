all:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules
	cp Module.symvers ../sculld/

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions modules.order Module.symvers

load:
	lsmod | grep lddbus && sudo rmmod lddbus || echo lddbus unloaded

	sudo /sbin/insmod ./lddbus.ko || exit 1

unload:
	sudo rmmod lddbus

obj-m := lddbus.o

