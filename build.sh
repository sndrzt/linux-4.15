#!/usr/bin/env bash

sudo apt-get update
sudo apt-get install -y net-tools openssh-server vim dpkg-dev build-essential bison flex libncursesw5-dev libssl-dev git tig
sudo apt-get build-dep -y linux-source-4.15.0

make menuconfig

date > a.txt
make bzImage -j 4
make modules -j 4

date > b.txt

sudo make modules_install
sudo make install

udevadm info /dev/iio:device0
echo -e "sensor:modalias:acpi:BOSC0200*:dmi:*\n ACCEL_MOUNT_MATRIX=0,1,0;-1,0,0;0,0,-1" > 61-sensor-local.hwdb
sudo cp 61-sensor-local.hwdb /etc/udev/hwdb.d/
sudo systemd-hwdb update

