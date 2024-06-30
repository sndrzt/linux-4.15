all:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

load:
	lsmod | grep scull && sudo rmmod scull || echo scull unloaded
	sudo /sbin/insmod ./scull.ko major=254 || exit 1

	major=$(awk "\$2==\"scull\" {print \$1}" /proc/devices)

	echo "major=${major}"

	sudo rm -f /dev/sculls
	sudo mknod /dev/sculls c 254 1
	sudo chgrp work /dev/sculls
	sudo chmod 664  /dev/sculls

	echo abcdefg > /dev/sculls
	cat /dev/sculls

unload:
	sudo rmmod scull
	sudo rm -rf /dev/sculls

scull-objs := main.o

obj-m := scull.o

