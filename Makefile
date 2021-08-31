# Load module: sudo insmod pci_driver.ko
#
CONFIG_MODULE_SIG=n
BINARY		:= pci_driver
KERNEL		:= /lib/modules/$(shell uname -r)/build
ARCH		:= x86
C_FLAGS		:= -Wall
KMOD_DIR	:= $(shell pwd)
TARGET_PATH := /lib/modules/$(shell uname -r)/kernel/drivers/char
OBJECTS		:= xil_pci_driver.o

ccflags-y += $(C_FLAGS)

obj-m += $(BINARY).o

$(BINARY)-y := $(OBJECTS)

$(BINARY).ko:
	make -C $(KERNEL) M=$(KMOD_DIR) modules

install:
	cp $(BINARY).ko $(TARGET_PATH)
	depmod -a

clean:
	rm -rf *.o
	rm -rf *.order
	rm -rf *.ko
	rm -rf .*.*.cmd
	rm -rf .*.*.*.cmd
	rm -rf *.mod.c
	rm -rf *.symvers