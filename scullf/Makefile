all:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

load:
	lsmod | grep scull && sudo rmmod scull || echo scull unloaded
	sudo /sbin/insmod ./scull.ko major=254 || exit 1

	major=$(awk "\$2==\"scull\" {print \$1}" /proc/devices)

	echo "major=${major}"

	sudo rm -f /dev/scullpipe[0-3]
	sudo mknod /dev/scullpipe0 c 254 4
	sudo mknod /dev/scullpipe1 c 254 5
	sudo mknod /dev/scullpipe2 c 254 6
	sudo mknod /dev/scullpipe3 c 254 7
	sudo ln -sf scullpipe0 /dev/scullpipe
	sudo chgrp work /dev/scullpipe[0-3]
	sudo chmod 664  /dev/scullpipe[0-3]

	cat /dev/scullpipe0
	echo abc > /dev/scullpipe0

unload:
	sudo rmmod scull
	sudo rm -rf /dev/scullpipe[0-3]

scull-objs := main.o

obj-m := scull.o

