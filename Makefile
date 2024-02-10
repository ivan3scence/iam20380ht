CONFIG_MODULE_SIG=n

obj-m += iam20380ht_drv.o 

iam20380ht_drv-y := iam20380ht_driver.o \
					iam20380ht.o \
					iam20380ht_i2c.o

KERNELDIR ?= $(KERNEL_SRC)

PWD := $(shell pwd)

CFLAGS = $(EXTCFLAGS)

# DEBUG_I2C_STUB: for debuging on host machine with `sudo modprobe i2c-stub chip_addr=0x69`
MY_CFLAGS = -g -D IAM_DEBUG -D DEBUG_I2C_STUB -D NEW_KERNEL 

ccflags-y := -DIAM_USE_FIFO -DIAM_USE_BASIC_I2C_FUNC -D__KERNEL__ \
				$(MY_CFLAGS)

#				-Wall -Wextra

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

# for build on host machine
debug:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules EXTRA_CFLAGS="$(MY_CFLAGS)"

install:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules_install

dclean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

modules_install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install
	mkdir -p $(DEST_DIR)/usr/include
	cp  iam20380ht-ioctl.h $(DEST_DIR)/usr/include
	cp  iam20380ht_defs.h $(DEST_DIR)/usr/include

clean:
	rm -rf *.o *.ko .*.cmd  *.cmd *.mod* *order Module.symvers .tmp_versions

fclean: clean
	sudo rmmod iam20380ht_drv

re: clean debug

%.i: %.c
	$(MAKE) -C $(KERNELDIR) M=$(PWD) $@
