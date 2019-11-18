DRIVER_NAME=axi-ad7616-dma

LINUXDIR := /home/johnny/IRIM/kernel-source

obj-m := $(DRIVER_NAME).o

$(DRIVER_NAME)-objs=ad7616.o  spi_engine_linux.o

SRC := $(shell pwd)

all:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C $(LINUXDIR) M=$(shell pwd)

clean:
	make -C $(LINUXDIR) M=$(shell pwd) clean