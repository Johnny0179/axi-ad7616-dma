/*  axi-ad7616-dma.c - The simplest kernel module.

* Copyright (C) 2013 - 2016 Xilinx, Inc
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   (at your option) any later version.

*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License along
*   with this program. If not, see <http://www.gnu.org/licenses/>.

*/

#include "axi-ad7616-dma.h"
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dma/xilinx_dma.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>



/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
struct ad7616_init_param default_init_param = {
    /* SPI */
    SPI_AD7616_CS,  // spi_chip_select
    SPI_MODE_2,     // spi_mode
    SPI_ENGINE,     // spi_type
    -1,             // spi_device_id
    /* GPIO */
    PS7_GPIO,          // gpio_type
    GPIO_DEVICE_ID,    // gpio_device_id
    -1,                // gpio_hw_rngsel0
    -1,                // gpio_hw_rngsel1
    GPIO_ADC_RESET_N,  // gpio_reset
    -1,                // gpio_os0
    -1,                // gpio_os1
    -1,                // gpio_os2
    /* Device Settings */
    AD7616_SW,  // mode
    {AD7616_10V, AD7616_10V, AD7616_10V, AD7616_10V, AD7616_10V, AD7616_10V,
     AD7616_10V, AD7616_10V},  // va[8]
    {AD7616_10V, AD7616_10V, AD7616_10V, AD7616_10V, AD7616_10V, AD7616_10V,
     AD7616_10V, AD7616_10V},  // vb[8]
    AD7616_OSR_0,              // osr
};

/*ADC channel name */
static const char adc_channels[][20] = {
    {"adc0"},  {"adc1"},  {"adc2"},  {"adc3"},  {"adc4"},  {"adc5"},
    {"adc6"},  {"adc7"},  {"adc8"},  {"adc9"},  {"adc10"}, {"adc11"},
    {"adc12"}, {"adc13"}, {"adc14"}, {"adc15"},
};

/* ad7616 device */
// struct ad7616_dev dev;

static struct axi_adc_dev *axi_adc_dev;
static int dev_index = 0;
static dev_t devno;
static struct cdev adc_cdev;
static struct class *axi_adc_class;
static void dma_slave_rx_callback(void *completion) { complete(completion); }

/* request gpios */
static int ad7616_request_gpios(struct axi_adc_dev *axi_adc_dev) {
  struct device *dev = axi_adc_dev->pdev->dev;

  // reset pin
  axi_adc_dev->ad7616_dev->gpio_reset =
      devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
  if (IS_ERR(axi_adc_dev->ad7616_dev->gpio_reset))
    return PTR_ERR(axi_adc_dev->ad7616_dev->gpio_reset);
}

// ad7616 rst
static int ad7616_reset(struct axi_adc_dev *axi_adc_dev) {
  if (axi_adc_dev->ad7616_dev->gpio_reset) {
    gpiod_set_value(axi_adc_dev->ad7616_dev->gpio_reset, 1);
    ndelay(100); /* t_reset >= 100ns */
    gpiod_set_value(axi_adc_dev->ad7616_dev->gpio_reset, 0);
    return 0;
  }
  return -ENODEV;
}

/* ad7616 core setup */
static void ad7616_core_setup() {
  unsigned int type;
  // axi_adc_dev->ad7616_core->adc_baseaddr = &axi_adc_dev->adc_virtaddr;
  axi_adc_dev->ad7616_dev->core->no_of_channels = 2;
  axi_adc_dev->ad7616_dev->core->resolution = 16;

  writel(0x00, axi_adc_dev->adc_virtaddr + AD7616_REG_UP_CTRL);
  mdelay(10);
  writel(AD7616_CTRL_RESETN, axi_adc_dev->adc_virtaddr + AD7616_REG_UP_CTRL);
  writel(100, axi_adc_dev->adc_virtaddr + AD7616_REG_UP_CONV_RATE);
  writel(AD7616_CTRL_RESETN | AD7616_CTRL_CNVST_EN,
         axi_adc_dev->adc_virtaddr + AD7616_REG_UP_CTRL);
  mdelay(10);

  writel(AD7616_CTRL_RESETN, axi_adc_dev->adc_virtaddr + AD7616_REG_UP_CTRL);
  type = readl(axi_adc_dev->adc_virtaddr + AD7616_REG_UP_IF_TYPE);

  printk("AD7616 IP Core (%s interface) successfully initialized\n",
         type ? "parallel" : "serial");
}

static void ad7616_capture_serial() {
  mdelay(10);
  // spi_engine_write(SPI_ENGINE_REG_OFFLOAD_CTRL(0), 0x0);
  writel_relaxed(0x0,
                 axi_adc_dev->adc_virtaddr + SPI_ENGINE_REG_OFFLOAD_CTRL(0));
  mdelay(10);
  // spi_engine_write(SPI_ENGINE_REG_OFFLOAD_RESET(0), 0x1);
  writel_relaxed(0x1,
                 axi_adc_dev->adc_virtaddr + SPI_ENGINE_REG_OFFLOAD_RESET(0));
  // spi_engine_write(SPI_ENGINE_REG_OFFLOAD_SDO_MEM(0), 0x00);
  writel_relaxed(0x00,
                 axi_adc_dev->adc_virtaddr + SPI_ENGINE_REG_OFFLOAD_SDO_MEM(0));
  // spi_engine_write(SPI_ENGINE_REG_OFFLOAD_CMD_MEM(0), 0x2103);
  writel_relaxed(0x2103,
                 axi_adc_dev->adc_virtaddr + SPI_ENGINE_REG_OFFLOAD_CMD_MEM(0));
  // spi_engine_write(SPI_ENGINE_REG_OFFLOAD_CMD_MEM(0), 0x2000);
  writel_relaxed(0x2000,
                 axi_adc_dev->adc_virtaddr + SPI_ENGINE_REG_OFFLOAD_CMD_MEM(0));
  // spi_engine_write(SPI_ENGINE_REG_OFFLOAD_CMD_MEM(0), 0x11fe);
  writel_relaxed(0x11fe,
                 axi_adc_dev->adc_virtaddr + SPI_ENGINE_REG_OFFLOAD_CMD_MEM(0));
  // spi_engine_write(SPI_ENGINE_REG_OFFLOAD_CMD_MEM(0), 0x0201);
  writel_relaxed(0x0201,
                 axi_adc_dev->adc_virtaddr + SPI_ENGINE_REG_OFFLOAD_CMD_MEM(0));
  // spi_engine_write(SPI_ENGINE_REG_OFFLOAD_CMD_MEM(0), 0x3000);
  writel_relaxed(0x3000,
                 axi_adc_dev->adc_virtaddr + SPI_ENGINE_REG_OFFLOAD_CMD_MEM(0));
  // spi_engine_write(SPI_ENGINE_REG_OFFLOAD_CMD_MEM(0), 0x11ff);
  writel_relaxed(0x11ff,
                 axi_adc_dev->adc_virtaddr + SPI_ENGINE_REG_OFFLOAD_CMD_MEM(0));
  // spi_engine_write(SPI_ENGINE_REG_OFFLOAD_CTRL(0), 0x1);
  writel_relaxed(0x1,
                 axi_adc_dev->adc_virtaddr + SPI_ENGINE_REG_OFFLOAD_CTRL(0));

  // ad7616_core_write(core, AD7616_REG_UP_CTRL,
  //                   AD7616_CTRL_RESETN | AD7616_CTRL_CNVST_EN);

  writel(AD7616_CTRL_RESETN | AD7616_CTRL_CNVST_EN,
         axi_adc_dev->adc_virtaddr + AD7616_REG_UP_CTRL);

  // DMA length
  // length = no_of_samples * core.no_of_channels * ((core.resolution + 7) / 8);
}

/* ad7616 device setup */
void ad7616_setup(struct axi_adc_dev *axi_adc_dev,
                  struct ad7616_init_param init_param) {
  int ret = 0;
  u8 i = 0;

  // set device mode
  axi_adc_dev->ad7616_dev->mode = init_param.mode;
  for (i = 0; i <= AD7616_VA7; i++) {
    axi_adc_dev->va[i] = init_param.va[i];
    axi_adc_dev->vb[i] = init_param.vb[i];
  }
  ret |= ad7616_set_mode(axi_adc_dev->master, axi_adc_dev->ad7616_dev,
                         axi_adc_dev->ad7616_dev->mode);

  // set oversampling ratio
  axi_adc_dev->ad7616_dev->osr = init_param.osr;

  ret |= ad7616_set_oversampling_ratio(axi_adc_dev->master,
                                       axi_adc_dev->ad7616_dev,
                                       axi_adc_dev->ad7616_dev->mode);

  if (!ret) printk("AD7616 successfully initialized\n");
}

/* File operations */
int axi_adc_dma_open(struct inode *inode, struct file *filp) {
  unsigned int mn;
  mn = iminor(inode);
  /*Assign minor number for later reference */
  filp->private_data = (void *)mn;
  return SUCCESS;
}

int axi_adc_dma_release(struct inode *inode, struct file *filp) {
  return SUCCESS;
}

ssize_t axi_adc_dma_read(struct file *filep, char __user *buf, size_t count,
                         loff_t *f_pos) {
  int minor = 0, rx_tmo = 0, status = 0;

  struct dma_device *rx_dev = axi_adc_dev[minor]->rx_chan->device;
  /* Query minor number.
   * @To be extended for multiple channel support
   */
  minor = (int)filep->private_data;

  /* Validation for read size */
  if (count > axi_adc_dev[minor]->dma_len_bytes) {
    dev_err(&axi_adc_dev[minor]->pdev->dev, "improper buffer size \n");
    return EINVAL;
  }

  rx_tmo = wait_for_completion_timeout(&axi_adc_dev[minor]->rx_cmp,
                                       axi_adc_dev[minor]->rx_tmo);
  /* Check the status of DMA channel */
  status = dma_async_is_tx_complete(axi_adc_dev[minor]->rx_chan,
                                    axi_adc_dev[minor]->rx_cookie, NULL, NULL);
  if (rx_tmo == 0) {
    dev_err(&axi_adc_dev[minor]->pdev->dev, "RX test timed out\n");
    return -EAGAIN;
  }
  dma_unmap_single(rx_dev->dev, axi_adc_dev[minor]->dma_dsts[0],
                   axi_adc_dev[minor]->dma_len_bytes, DMA_DEV_TO_MEM);
  copy_to_user(buf, axi_adc_dev[minor]->dsts[0], count);
  return count;
}

/* IOCTL calls provide interface to configure ,start and stop
   DMA engine */
static long axi_adc_dma_ioctl(struct file *file, unsigned int cmd,
                              unsigned long arg) {
  enum dma_status status;
  enum dma_ctrl_flags flags;

  int ret;
  int i;
  int minor = (int)file->private_data;
  struct adc_dma_cfg *cfg;
  struct dma_device *rx_dev = axi_adc_dev[minor]->rx_chan->device;
  struct dma_async_tx_descriptor *rxd = NULL;
  struct scatterlist rx_sg[MAX_BUF_COUNT];
  // dma_addr_t dma_dsts[bd_cnt];
  switch (cmd) {
    case AXI_ADC_SET_SAMPLE_NUM:
      axi_adc_dev[minor]->adc_sample_num = arg;
      break;

    case AXI_ADC_SET_DMA_LEN_BYTES:
      axi_adc_dev[minor]->dma_len_bytes = arg;
      break;

    case AXI_ADC_DMA_INIT:
      axi_adc_dev[minor]->bd_cnt = 1;
      axi_adc_dev[minor]->dsts =
          kcalloc(axi_adc_dev[minor]->bd_cnt + 1, sizeof(u8 *), GFP_KERNEL);

      if (!axi_adc_dev[minor]->dsts) return ret;

      for (i = 0; i < axi_adc_dev[minor]->bd_cnt; i++) {
        axi_adc_dev[minor]->dsts[i] =
            kmalloc(axi_adc_dev[minor]->dma_len_bytes, GFP_KERNEL);
      }
      axi_adc_dev[minor]->dsts[i] = NULL;
      axi_adc_dev[minor]->dma_dsts = kcalloc(axi_adc_dev[minor]->bd_cnt + 1,
                                             sizeof(dma_addr_t), GFP_KERNEL);
      flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
      for (i = 0; i < axi_adc_dev[minor]->bd_cnt; i++) {
        /*Configures DMA transaction */
        axi_adc_dev[minor]->dma_dsts[i] =
            dma_map_single(rx_dev->dev, axi_adc_dev[minor]->dsts[i],
                           axi_adc_dev[minor]->dma_len_bytes, DMA_MEM_TO_DEV);
      }

      break;

      /* Start the DMA transaction */
    case AXI_ADC_DMA_START:

      /* Preparing mapped scatter-gather list */
      sg_init_table(rx_sg, axi_adc_dev[minor]->bd_cnt);
      for (i = 0; i < axi_adc_dev[minor]->bd_cnt; i++) {
        sg_dma_address(&rx_sg[i]) = axi_adc_dev[minor]->dma_dsts[i];
        /* Configures S2MM data length */
        sg_dma_len(&rx_sg[i]) = axi_adc_dev[minor]->dma_len_bytes;
      }
      rxd = rx_dev->device_prep_slave_sg(axi_adc_dev[minor]->rx_chan, rx_sg,
                                         axi_adc_dev[minor]->bd_cnt,
                                         DMA_DEV_TO_MEM, flags, NULL);
      if (!rxd) {
        dev_err(&axi_adc_dev[minor]->pdev->dev, "rxd is NULL\n");
        for (i = 0; i < axi_adc_dev[minor]->bd_cnt; i++)
          dma_unmap_single(rx_dev->dev, axi_adc_dev[minor]->dma_dsts[i],
                           axi_adc_dev[minor]->dma_len_bytes, DMA_DEV_TO_MEM);
      }
      init_completion(&axi_adc_dev[minor]->rx_cmp);
      /* Added callback information */
      rxd->callback = dma_slave_rx_callback;
      rxd->callback_param = &axi_adc_dev[minor]->rx_cmp;

      /*Place transaction to DMA engine pending queue */
      axi_adc_dev[minor]->rx_cookie = rxd->tx_submit(rxd);

      /* Check for dma submit errors */
      if (dma_submit_error(axi_adc_dev[minor]->rx_cookie)) {
        dev_err(&axi_adc_dev[minor]->pdev->dev, "dma submit error\n");
      }
      axi_adc_dev[minor]->rx_tmo =
          msecs_to_jiffies(AXI_ADC_CALLBACK_TIMEOUTMSEC); /* RX takes longer */
      dma_async_issue_pending(axi_adc_dev[minor]->rx_chan);

      /* writel()
       *往内存映射的 I/O 空间上写数据，wirtel()  I/O 上写入 32 位数据(4字节)。
       */
      // writel(axi_adc_dev[minor]->adc_sample_num,
      //        axi_adc_dev[minor]->adc_virtaddr + 4);
      // writel(1, axi_adc_dev[minor]->adc_virtaddr);
      break;

    case AXI_ADC_DMA_DEINIT:
      break;

      // ad7616 core setup
    case AXI_AD7616_CORE_SETUP:

      /* ad7616 setup */
      // spi init

      break;

    default:

      return -EOPNOTSUPP;
  }

  return SUCCESS;
}

struct file_operations axi_adc_fops = {.owner = THIS_MODULE,
                                       .read = axi_adc_dma_read,
                                       .open = axi_adc_dma_open,
                                       .unlocked_ioctl = axi_adc_dma_ioctl,
                                       .release = axi_adc_dma_release};

static int axi_adc_remove(struct platform_device *pdev) {
  int i;
  for (i = 0; i < dev_index; i++) {
    device_destroy(axi_adc_class, MKDEV(MAJOR(devno), i));

    /* Free up the DMA channel */
    dma_release_channel(axi_adc_dev[i]->rx_chan);

    /* Unmap the adc I/O memory */
    if (axi_adc_dev[i]->adc_virtaddr) iounmap(axi_adc_dev[i]->adc_virtaddr);

    if (axi_adc_dev[i]) {
      kfree(axi_adc_dev[i]);
    }
    dev_info(&pdev->dev, "adc DMA Unload :: Success \n");
  }
  class_destroy(axi_adc_class);
  cdev_del(&adc_cdev);
  unregister_chrdev_region(devno, AXI_ADC_MINOR_COUNT);
  return SUCCESS;
}

static int axi_adc_probe(struct platform_device *pdev) {
  int ret = 0, i = 0;
  struct device_node *node = NULL;
  struct spi_master *master;
  struct resource *res;

  /* spi engine interrupt */
  int irq;

  /*Allocate device node */
  node = pdev->dev.of_node;

  // get an IRQ for the spi engine
  irq = platform_get_irq(pdev, 0);
  if (irq <= 0) return -ENXIO;

  /* Allocate a private structure to manage this device
   * 一般情况下，内存只有在要被 DMA
   * 访问的时候才需要物理上连续，但为了性能上的考虑，内核中一般使用 kmalloc()
   */
  axi_adc_dev = kmalloc(sizeof(struct axi_adc_dev), GFP_KERNEL);
  if (axi_adc_dev == NULL) {
    dev_err(&pdev->dev, "unable to allocate device structure\n");
    return -ENOMEM;
  }

  //  allocate SPI master controller
  master = spi_alloc_master(&pdev->dev, 0);
  if (!master) return -ENOMEM;

  spi_master_set_devdata(master, axi_adc_dev->spi_engine);

  /*Memset
   *用来对一段内存空间全部设置为某个字符，一般用在对定义的字符串进行初始化为‘
   *’或‘/0’； 例:char a[100];memset(a, ‘/0’, sizeof(a));
   */
  memset(axi_adc_dev, 0, sizeof(struct axi_adc_dev));

  // 向dmaengine系统申请DMA通道
  axi_adc_dev->rx_chan = dma_request_slave_channel(&pdev->dev, "axidma1");

  if (IS_ERR(axi_adc_dev->rx_chan)) {
    dev_err(&pdev->dev, "No DMA Rx channel\n");
    goto free_rx;
  }

  if (axi_adc_dev->rx_chan == NULL) {
    dev_err(&pdev->dev, "No DMA Rx channel\n");
    goto fail1;
  }

  /* IOMAP adc
   *registers,ioremap将一个IO地址空间映射到内核的虚拟地址空间上去，便于访问。
   *void __iomem *of_iomap(struct device_node *node, int index);
   *通过设备结点直接进行设备内存区间的ioremap()，index是内存段的索引。
   *若设备结点的reg属性有多段，可通过index标示要ioremap的是哪一段，只有1段的情况，index为0。
   *采用DeviceTree后，大量的设备驱动通过of_iomap()进行映射，而不再通过传统的ioremap。
   */
  axi_adc_dev->adc_virtaddr = of_iomap(node, 0);
  if (!axi_adc_dev->adc_virtaddr) {
    dev_err(&pdev->dev, "unable to IOMAP adc registers\n");
    ret = -ENOMEM;
    goto fail1;
  }

  axi_adc_dev->pdev = pdev;

  /* Initialize our device mutex */
  // mutex_init(&axi_adc_dev->mutex);

  // if (dev_index == 0) {
  ret = alloc_chrdev_region(&devno, 0, AXI_ADC_MINOR_COUNT, MODULE_NAME);
  if (ret < 0) {
    dev_err(&pdev->dev, "unable to alloc chrdev \n");
    goto fail2;
  }

  /* Register with the kernel as a character device */
  cdev_init(&adc_cdev, &axi_adc_fops);
  adc_cdev.owner = THIS_MODULE;
  adc_cdev.ops = &axi_adc_fops;

  mutex_init(&axi_adc_dev->mutex);

  ret = cdev_add(&adc_cdev, devno, AXI_ADC_MINOR_COUNT);

  axi_adc_class = class_create(THIS_MODULE, MODULE_NAME);
  // }

  // Creating device node for each ADC channel
  device_create(axi_adc_class, NULL, MKDEV(MAJOR(devno), AXI_ADC_MINOR_START),
                NULL, adc_channels[i]);

  dev_info(&pdev->dev, "PL ad7616 added successfully\n");
  // dev_index++;

  // Initializing AXI DMA
  axi_ad7616_dma_init();

  /* ad7616 core setup */
  ad7616_core_setup();

  /* spi engine setup */
  spi_engine_setup(irq, axi_adc_dev, axi_adc_dev->spi_engine,
                   axi_adc_dev->master);
  dev_info(&pdev->dev, "SPI Engine successfully initialized\n");

  /* ad7616 device setup */
  ad7616_setup(axi_adc_dev, default_init_param);

  dev_info(&pdev->dev, "AXI AD7616 configured!\n");

  return SUCCESS;

fail2:
  iounmap(axi_adc_dev->adc_virtaddr);
free_rx:
  dma_release_channel(axi_adc_dev->rx_chan);
fail1:
  kfree(axi_adc_dev);
}

static const struct of_device_id axi_ad7616_dma_of_ids[] = {
    {
        .compatible = "airs,axi-ad7616-dma",
    },
};

static struct platform_driver axi_ad7616_dma_of_driver = {
    .driver =
        {
            .name = MODULE_NAME,
            .owner = THIS_MODULE,
            .of_match_table = axi_ad7616_dma_of_ids,
        },
    .probe = axi_adc_probe,
    .remove = axi_adc_remove,
};

module_platform_driver(axi_ad7616_dma_of_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AIRS AXI AD7616 DMA driver");
MODULE_AUTHOR("AIRS, Inc.");
MODULE_VERSION("1.00a");
