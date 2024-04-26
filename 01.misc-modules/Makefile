obj := hello.o
all:
	make -C /home/work/linux-2.6.11 M=/home/work/hello modules

load:
	sudo insmod hello.ko

unload:
	sudo rmmod hello

check:
	sudo dmesg | tail -n 20
