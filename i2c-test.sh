#!/usr/bin/bash

sudo modprobe i2c-dev
sudo modprobe i2c-stub chip_addr=0x69
i2cdetect -y -l
sudo i2cdetect -y 1
sudo i2cset -y 1 0x69 0x75 0xfd
sudo i2cdump -y -r 0x00-0x7f 1 0x69