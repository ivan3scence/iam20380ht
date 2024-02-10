# IAM20380HT

This project contains a kernel driver for gyroscope IAM20380HT.

In this project only I2C interface is implemented.

Used for 3.0.25 IMX Linux kernel.

To compile fow newer kernel (>5), use **NEW_KERNEL** macro(see Makefile).

## Description

Driver connects to sensor via I2C, allows to get gyro values with:

* /sys/class/input/input?/value
* IAM20380HT_IO_GDATA - ioctl

Driver supports sensor FIFO usage. In this case
every frame marks with its approximate timestamp.

Usage:
1. enable driver to store gyro values from FIFO:

    - /sys/class/input/input?/enable
    - IAM20380HT_IO_STOREENABLE - ioctl

2. read array of values:

    - IAM20380HT_IO_QUEUEDATAFRAME - ioctl

## Docs

If you prefer, you can use Doxygen to get project
documentation:

    sudo [apt install] doxygen      # install doxygen
    cd docs
    doxygen
    google-chrome html/index.html