DRIVER_NAME=axi-ad7616-dma

obj-m := $(DRIVER_NAME).o

$(DRIVER_NAME)-objs=axi-ad7616-dma.o ad7616.o  spi_engine_linux.o

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers