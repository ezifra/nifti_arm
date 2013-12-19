##########################
# where your kernel sources are
KERNELDIR  = /lib/modules/`uname -r`/build

GCC    = gcc
DEPMOD = /sbin/depmod

MANPATH= /usr/share/man

# UDEV - only relevant if you are using udev system
# Path to your configuration directory for udev
UDEV_CONF_PATH = /etc/udev

# enable this if you don't use devfs nor udev
CREATE_INODES=false

OUTPUT_FILE = /dev/null

# end configuration part
##########################

PWD := $(shell pwd)

KERNEL_SRC ?= /usr/src/linux-headers-$(shell uname -r)
KERNEL_MODS ?= /lib/modules/$(shell uname -r)

EXTRA_CFLAGS = $(CMODFLAGS)

obj-m := cpc-usb.o

cpc-usb-objs := cpc_usb.o sja2m16c.o

all: module

module: cpc_usb.c
	@$(MAKE) -C $(KERNEL_SRC) SUBDIRS=$(PWD) M=$(PWD) modules

install: module-install

module-install: cpc-usb.ko
	install -c -o root -g root -m 644 cpc-usb.ko \
	$(KERNEL_MODS)/kernel/drivers/usb/misc
	depmod -ae
	install -c -o root -g root -m 644 udev.rules \
	/etc/udev/rules.d/41-cpc-usb.rules
	udevadm control --reload-rules

clean:
	rm -rf *.o *.ko *.mod.c .*.cmd .tmp_versions *~ *.a *.so *.d
	rm -rf Module.symvers Module.markers modules.order
