ifeq ($(BOARD), arria10)
  BDDEF = DMP_ARRIA10
else
  BDDEF = DMP_ZC706
endif

KERNELDIR ?= /lib/modules/`uname -r`/build
CROSS_COMPILE ?= arm-linux-gnueabihf-
BUILD_DIR := $(shell pwd)
VERBOSE = 0
PWD = $(shell pwd)
MOD = dmp_fb
ARCH = arm

MAKEARCH = $(MAKE) ARCH=$(ARCH) KCPPFLAGS="-D$(BDDEF)" CROSS_COMPILE=$(CROSS_COMPILE)

obj-m := $(MOD).o 
$(MOD)-objs  := ./src/dv-fb.o 
$(MOD)-objs  += ./src/pdc.o 

all: 
	$(MAKEARCH) -C $(KERNELDIR) M=$(PWD) modules

clean: 
	rm -rf *.ko *.o *.order src/*.o .tmp_vers* *.symvers *.mod.c .dmp* src/.*.cmd
