#ifndef AXI_AD7616_DMA_H
#define AXI_AD7616_DMA_H

#include <linux/delay.h>
#include <linux/kernel.h>
#include <stdint.h>
#include <unistd.h>
#include "ad7616.h"
#include <linux/gpio/consumer.h>
// #include "ad7616_core.h"
#include "spi_engine_linux.h"

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/spi/spi.h>

#define MODULE_NAME "axi_ad7616_dma"
#define AXI_ADC_MINOR_START 0
#define AXI_ADC_MINOR_COUNT 16
#define AXI_ADC_CALLBACK_TIMEOUTMSEC 10000
#define SUCCESS 0
#define FAILURE -1
#define MAX_BUF_COUNT 8
/* IOCTL defines */
#define AXI_ADC_IOCTL_BASE 'W'
#define AXI_ADC_SET_SAMPLE_NUM _IO(AXI_ADC_IOCTL_BASE, 0)
#define AXI_ADC_SET_DMA_LEN_BYTES _IO(AXI_ADC_IOCTL_BASE, 1)
#define AXI_ADC_DMA_INIT _IO(AXI_ADC_IOCTL_BASE, 2)
#define AXI_ADC_DMA_START _IO(AXI_ADC_IOCTL_BASE, 3)
#define AXI_ADC_DMA_DEINIT _IO(AXI_ADC_IOCTL_BASE, 4)
#define AXI_AD7616_CORE_SETUP _IO(AXI_ADC_IOCTL_BASE, 5)
#define AXI_AD7616_SETUP _IO(AXI_ADC_IOCTL_BASE, 6)
#define AXI_AD7616_CAPTURE_SERIAL _IO(AXI_ADC_IOCTL_BASE, 7)



/* adc core */
#define AD7616_REG_PCORE_VERSION 0x400
#define AD7616_REG_ID 0x404
#define AD7616_REG_UP_SCRATCH 0x408
#define AD7616_REG_UP_IF_TYPE 0x40C
#define AD7616_REG_UP_CTRL 0x440
#define AD7616_REG_UP_CONV_RATE 0x444
#define AD7616_REG_UP_READ_DATA 0x448
#define AD7616_REG_UP_WRITE_DATA 0x44C

/* AD7616_REG_UP_CTRL */
#define AD7616_CTRL_RESETN (1 << 0)
#define AD7616_CTRL_CNVST_EN (1 << 1)

#define ADC_DMAC_REG_IRQ_MASK 0x80
#define ADC_DMAC_REG_IRQ_PENDING 0x84
#define ADC_DMAC_REG_IRQ_SOURCE 0x88

#define ADC_DMAC_REG_CTRL 0x400
#define ADC_DMAC_REG_TRANSFER_ID 0x404
#define ADC_DMAC_REG_START_TRANSFER 0x408
#define ADC_DMAC_REG_FLAGS 0x40c
#define ADC_DMAC_REG_DEST_ADDRESS 0x410
#define ADC_DMAC_REG_SRC_ADDRESS 0x414
#define ADC_DMAC_REG_X_LENGTH 0x418
#define ADC_DMAC_REG_Y_LENGTH 0x41c
#define ADC_DMAC_REG_DEST_STRIDE 0x420
#define ADC_DMAC_REG_SRC_STRIDE 0x424
#define ADC_DMAC_REG_TRANSFER_DONE 0x428
#define ADC_DMAC_REG_ACTIVE_TRANSFER_ID 0x42c
#define ADC_DMAC_REG_STATUS 0x430
#define ADC_DMAC_REG_CURRENT_DEST_ADDR 0x434
#define ADC_DMAC_REG_CURRENT_SRC_ADDR 0x438
#define ADC_DMAC_REG_DBG0 0x43c
#define ADC_DMAC_REG_DBG1 0x440

#define ADC_DMAC_CTRL_ENABLE (1 << 0)
#define ADC_DMAC_CTRL_PAUSE (1 << 1)

#define ADC_DMAC_IRQ_SOT (1 << 0)
#define ADC_DMAC_IRQ_EOT (1 << 1)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
struct adc_core {
  uint32_t adc_baseaddr;
  uint32_t dmac_baseaddr;
  uint8_t no_of_channels;
  uint8_t resolution;
};



// ad7616 device structure
struct axi_adc_dev {
  struct mutex mutex;
  struct platform_device *pdev;
  /* ADC Hardware device constants */
  void *adc_virtaddr;
  /* DMA stuff */
  struct dma_chan *rx_chan;
  dma_cookie_t rx_cookie;
  struct completion rx_cmp;
  unsigned long rx_tmo;
  int bd_cnt;
  struct scatterlist *rx_sg;
  /*DMA address of buffer */
  dma_addr_t *dma_dsts;
  u8 **dsts;
  int adc_sample_num;
  int dma_len_bytes;
  /* spi engine */
  struct spi_engine *spi_engine;
  struct spi_master *master;

  /* Device Settings */
  struct ad7616_dev *ad7616_dev;
};

int axi_ad7616_dma_init();
// void ad7616_capture_serial();

#endif
