ARCH=$(shell gcc -print-multiarch)

ifeq ($(ARCH), arm-linux-gnueabihf)

# For on-board compiling (32-bit ARM)
MAKEARCH = $(MAKE)

else

ifeq ($(ARCH), aarch64-linux-gnu)

# For on-board compiling (64-bit ARM)
MAKEARCH = $(MAKE)

else

#ARCH ?= arm
ARCH ?= arm64
#CROSS_COMPILE ?= arm-linux-gnueabihf-
CROSS_COMPILE ?= aarch64-linux-gnu-
MAKEARCH = $(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE)

endif

endif

BUILD_DIR := $(shell pwd)
VERBOSE = 0
PWD = $(shell pwd)
MOD = dmp_fb
KERNELDIR ?= /lib/modules/`uname -r`/build

obj-m := $(MOD).o 
$(MOD)-objs  := ./src/dv-fb.o 
$(MOD)-objs  += ./src/pdc.o 

all: 
	$(MAKEARCH) -C $(KERNELDIR) M=$(PWD) modules

.SILENT:	install

install:
	echo Copying $(MOD).ko to /lib/modules/`uname -r`/
	cp $(MOD).ko /lib/modules/`uname -r`/
	echo depmod -a
	depmod -a
	echo To reload the module, execute: sudo rmmod $(MOD) \&\& sudo modprobe $(MOD)

clean: 
	rm -rf *.ko *.o *.order src/*.o .tmp_vers* *.symvers *.mod.c .dmp* src/.*.cmd
