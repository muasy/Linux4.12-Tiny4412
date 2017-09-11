# Linux modules compile
# Author : SY
# Time   : 2017-8-8 21:30:31
#############################################################################

TARGET      := tiny4412_fb
obj-m       := $(TARGET).o
CROSS       := arm-none-linux-gnueabi-
KDIR        := /opt/fs/rootfs/rootfs/lib/modules/4.12.0+/build
PWD         := $(shell pwd)
INSTALL_DIR := /opt/fs/rootfs/rootfs

all:
	$(MAKE) -C $(KDIR) M=$(PWD)

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install INSTALL_MOD_PATH=$(INSTALL_DIR)
	chmod 755 $(TARGET).ko
	cp $(TARGET).ko /opt/fs/rootfs/rootfs/opt/app
	
	
clean:
	rm -rf *.o *.ko *.mod.c *.temp_versions *.symvers *.order .built* .tmp* .$(TARGET)*
