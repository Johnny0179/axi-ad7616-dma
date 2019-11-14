SUMMARY = "Recipe for  build an external axi-ad7616-dma Linux kernel module"
SECTION = "PETALINUX/modules"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=12f884d2ae1ff87c09e5b7ccc2c4ca7e"

inherit module

SRC_URI = "file://Makefile \
           file://axi-ad7616-dma.c \
           file://ad7616_core.c \
           file://spi_engine_linux.c \
           file://ad7616.c \
           file://ad7616_core.h \
           file://ad7616.h \
           file://axi-ad7616-dma.h \
           file://spi_engine_linux.h \
           file://COPYING \
          "

S = "${WORKDIR}"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.
