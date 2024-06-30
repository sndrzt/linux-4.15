all:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

load:
	lsmod | grep scull && sudo rmmod scull || echo scull unloaded
	sudo /sbin/insmod ./scull.ko major=254 || exit 1

	major=$(awk "\$2==\"scull\" {print \$1}" /proc/devices)

	echo "major=${major}"

	sudo rm -f /dev/scullu
	sudo mknod /dev/scullu c 254 1
	sudo chgrp work /dev/scullu
	sudo chmod 664  /dev/scullu

	echo abcdefg > /dev/scullu
	cat /dev/scullu

unload:
	sudo rmmod scull
	sudo rm -rf /dev/scullu

scull-objs := main.o

obj-m := scull.o

