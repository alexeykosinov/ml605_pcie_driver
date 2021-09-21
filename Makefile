# Read perform	: cat /dev/ML605_PCIe
# Write perform	: cat > /dev/ML605_PCIe
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

remove:
	rm $(TARGET_PATH)/$(BINARY).ko 

clean:
	rm -rf *.o
	rm -rf *.order
	rm -rf *.ko
	rm -rf .*.*.cmd
	rm -rf .*.*.*.cmd
	rm -rf *.mod.c
	rm -rf *.symvers
