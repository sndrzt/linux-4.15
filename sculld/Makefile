all:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions modules.order

load:
	lsmod | grep scull && sudo rmmod scull || echo scull unloaded

	major=$(awk "\$2==\"scull\" {print \$1}" /proc/devices)

	echo "major=${major}"

	sudo rm -f /dev/scull[0-3]
	sudo mknod /dev/scull0 c 250 0
	sudo mknod /dev/scull1 c 250 1
	sudo mknod /dev/scull2 c 250 2
	sudo mknod /dev/scull3 c 250 3
	sudo ln -sf scull0 /dev/scull
	sudo chgrp work /dev/scull[0-3]
	sudo chmod 664  /dev/scull[0-3]

	sudo /sbin/insmod ./scull.ko || echo "insmod scull.ko failed"

test:
	echo abcdefg > /dev/scull0
	cat /dev/scull0

unload:
	sudo rmmod scull
	sudo rm -rf /dev/scull[0-3]

scull-objs := main.o

obj-m := scull.o

