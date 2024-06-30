all:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

load:
	lsmod | grep scull && sudo rmmod scull || echo scull unloaded
	sudo /sbin/insmod ./scull.ko major=254 || exit 1

	major=$(awk "\$2==\"scull\" {print \$1}" /proc/devices)

	echo "major=${major}"

	sudo rm -f /dev/scull[0-3]
	sudo mknod /dev/scull0 c 254 0
	sudo mknod /dev/scull1 c 254 1
	sudo mknod /dev/scull2 c 254 2
	sudo mknod /dev/scull3 c 254 3
	sudo ln -sf scull0 /dev/scull
	sudo chgrp work /dev/scull[0-3]
	sudo chmod 664  /dev/scull[0-3]

	echo abcdefg > /dev/scull0
	cat /dev/scull0

unload:
	sudo rmmod scull

scull-objs := main.o

obj-m := scull.o

