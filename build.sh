#!/usr/bin/env bash

make menuconfig

date > a.txt
make bzImage -j 4
make modules -j 4

date > b.txt

sudo make modules_install
sudo make install

