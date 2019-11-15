#include "spi_engine_linux.h"

int spi_engine_transfer_one_message(struct spi_master *master,
                                    struct spi_message *msg) {
  struct spi_engine_program p_dry, *p;
  struct spi_engine *spi_engine = spi_master_get_devdata(master);
  unsigned int int_enable = 0;
  unsigned long flags;
  size_t size;

  p_dry.length = 0;
  spi_engine_compile_message(spi_engine, msg, true, &p_dry);

  size = sizeof(*p->instructions) * (p_dry.length + 1);
  p = kzalloc(sizeof(*p) + size, GFP_KERNEL);
  if (!p) return -ENOMEM;
  spi_engine_compile_message(spi_engine, msg, false, p);

  spin_lock_irqsave(&spi_engine->lock, flags);
  spi_engine->sync_id = (spi_engine->sync_id + 1) & 0xff;
  spi_engine_program_add_cmd(p, false,
                             SPI_ENGINE_CMD_SYNC(spi_engine->sync_id));

  spi_engine->msg = msg;
  spi_engine->p = p;

  spi_engine->cmd_buf = p->instructions;
  spi_engine->cmd_length = p->length;
  if (spi_engine_write_cmd_fifo(spi_engine))
    int_enable |= SPI_ENGINE_INT_CMD_ALMOST_EMPTY;

  spi_engine_tx_next(spi_engine);
  if (spi_engine_write_tx_fifo(spi_engine))
    int_enable |= SPI_ENGINE_INT_SDO_ALMOST_EMPTY;

  spi_engine_rx_next(spi_engine);
  if (spi_engine->rx_length != 0) int_enable |= SPI_ENGINE_INT_SDI_ALMOST_FULL;

  int_enable |= SPI_ENGINE_INT_SYNC;

  writel_relaxed(int_enable, spi_engine->base + SPI_ENGINE_REG_INT_ENABLE);
  spi_engine->int_enable = int_enable;
  spin_unlock_irqrestore(&spi_engine->lock, flags);

  return 0;
}

irqreturn_t spi_engine_irq(int irq, void *devid) {
  struct spi_master *master = devid;
  struct spi_engine *spi_engine = spi_master_get_devdata(master);
  unsigned int disable_int = 0;
  unsigned int pending;

  pending = readl_relaxed(spi_engine->base + SPI_ENGINE_REG_INT_PENDING);

  if (pending & SPI_ENGINE_INT_SYNC) {
    writel_relaxed(SPI_ENGINE_INT_SYNC,
                   spi_engine->base + SPI_ENGINE_REG_INT_PENDING);
    spi_engine->completed_id =
        readl_relaxed(spi_engine->base + SPI_ENGINE_REG_SYNC_ID);
  }

  spin_lock(&spi_engine->lock);

  if (pending & SPI_ENGINE_INT_CMD_ALMOST_EMPTY) {
    if (!spi_engine_write_cmd_fifo(spi_engine))
      disable_int |= SPI_ENGINE_INT_CMD_ALMOST_EMPTY;
  }

  if (pending & SPI_ENGINE_INT_SDO_ALMOST_EMPTY) {
    if (!spi_engine_write_tx_fifo(spi_engine))
      disable_int |= SPI_ENGINE_INT_SDO_ALMOST_EMPTY;
  }

  if (pending & (SPI_ENGINE_INT_SDI_ALMOST_FULL | SPI_ENGINE_INT_SYNC)) {
    if (!spi_engine_read_rx_fifo(spi_engine))
      disable_int |= SPI_ENGINE_INT_SDI_ALMOST_FULL;
  }

  if (pending & SPI_ENGINE_INT_SYNC) {
    if (spi_engine->msg && spi_engine->completed_id == spi_engine->sync_id) {
      struct spi_message *msg = spi_engine->msg;

      kfree(spi_engine->p);
      msg->status = 0;
      msg->actual_length = msg->frame_length;
      spi_engine->msg = NULL;
      spi_finalize_current_message(master);
      disable_int |= SPI_ENGINE_INT_SYNC;
    }
  }

  if (disable_int) {
    spi_engine->int_enable &= ~disable_int;
    writel_relaxed(spi_engine->int_enable,
                   spi_engine->base + SPI_ENGINE_REG_INT_ENABLE);
  }

  spin_unlock(&spi_engine->lock);

  return IRQ_HANDLED;
}

void spi_engine_program_add_cmd(struct spi_engine_program *p, bool dry,
                                uint16_t cmd) {
  if (!dry) p->instructions[p->length] = cmd;
  p->length++;
}

unsigned int spi_engine_get_config(struct spi_device *spi) {
  unsigned int config = 0;

  if (spi->mode & SPI_CPOL) config |= SPI_ENGINE_CONFIG_CPOL;
  if (spi->mode & SPI_CPHA) config |= SPI_ENGINE_CONFIG_CPHA;
  if (spi->mode & SPI_3WIRE) config |= SPI_ENGINE_CONFIG_3WIRE;

  return config;
}

unsigned int spi_engine_get_clk_div(struct spi_engine *spi_engine,
                                    struct spi_device *spi,
                                    struct spi_transfer *xfer) {
  unsigned int clk_div;

  clk_div = DIV_ROUND_UP(clk_get_rate(spi_engine->ref_clk), xfer->speed_hz * 2);
  if (clk_div > 255)
    clk_div = 255;
  else if (clk_div > 0)
    clk_div -= 1;

  return clk_div;
}

void spi_engine_gen_xfer(struct spi_engine_program *p, bool dry,
                         struct spi_transfer *xfer) {
  unsigned int len = xfer->len;

  while (len) {
    unsigned int n = min(len, 256U);
    unsigned int flags = 0;

    if (xfer->tx_buf) flags |= SPI_ENGINE_TRANSFER_WRITE;
    if (xfer->rx_buf) flags |= SPI_ENGINE_TRANSFER_READ;

    spi_engine_program_add_cmd(p, dry, SPI_ENGINE_CMD_TRANSFER(flags, n - 1));
    len -= n;
  }
}

void spi_engine_gen_sleep(struct spi_engine_program *p, bool dry,
                          struct spi_engine *spi_engine, unsigned int clk_div,
                          unsigned int delay) {
  unsigned int spi_clk = clk_get_rate(spi_engine->ref_clk);
  unsigned int t;

  if (delay == 0) return;

  t = DIV_ROUND_UP(delay * spi_clk, (clk_div + 1) * 2);
  while (t) {
    unsigned int n = min(t, 256U);

    spi_engine_program_add_cmd(p, dry, SPI_ENGINE_CMD_SLEEP(n - 1));
    t -= n;
  }
}

void spi_engine_gen_cs(struct spi_engine_program *p, bool dry,
                       struct spi_device *spi, bool assert) {
  unsigned int mask = 0xff;

  if (assert) mask ^= BIT(spi->chip_select);

  spi_engine_program_add_cmd(p, dry, SPI_ENGINE_CMD_ASSERT(1, mask));
}

int spi_engine_compile_message(struct spi_engine *spi_engine,
                               struct spi_message *msg, bool dry,
                               struct spi_engine_program *p) {
  struct spi_device *spi = msg->spi;
  struct spi_transfer *xfer;
  int clk_div, new_clk_div;
  bool cs_change = true;

  clk_div = -1;

  spi_engine_program_add_cmd(p, dry,
                             SPI_ENGINE_CMD_WRITE(SPI_ENGINE_CMD_REG_CONFIG,
                                                  spi_engine_get_config(spi)));

  list_for_each_entry(xfer, &msg->transfers, transfer_list) {
    new_clk_div = spi_engine_get_clk_div(spi_engine, spi, xfer);
    if (new_clk_div != clk_div) {
      clk_div = new_clk_div;
      spi_engine_program_add_cmd(
          p, dry, SPI_ENGINE_CMD_WRITE(SPI_ENGINE_CMD_REG_CLK_DIV, clk_div));
    }

    if (cs_change) spi_engine_gen_cs(p, dry, spi, true);

    spi_engine_gen_xfer(p, dry, xfer);
    spi_engine_gen_sleep(p, dry, spi_engine, clk_div, xfer->delay_usecs);

    cs_change = xfer->cs_change;
    if (list_is_last(&xfer->transfer_list, &msg->transfers))
      cs_change = !cs_change;

    if (cs_change) spi_engine_gen_cs(p, dry, spi, false);
  }

  return 0;
}

void spi_engine_xfer_next(struct spi_engine *spi_engine,
                          struct spi_transfer **_xfer) {
  struct spi_message *msg = spi_engine->msg;
  struct spi_transfer *xfer = *_xfer;

  if (!xfer) {
    xfer =
        list_first_entry(&msg->transfers, struct spi_transfer, transfer_list);
  } else if (list_is_last(&xfer->transfer_list, &msg->transfers)) {
    xfer = NULL;
  } else {
    xfer = list_next_entry(xfer, transfer_list);
  }

  *_xfer = xfer;
}

void spi_engine_tx_next(struct spi_engine *spi_engine) {
  struct spi_transfer *xfer = spi_engine->tx_xfer;

  do {
    spi_engine_xfer_next(spi_engine, &xfer);
  } while (xfer && !xfer->tx_buf);

  spi_engine->tx_xfer = xfer;
  if (xfer) {
    spi_engine->tx_length = xfer->len;
    spi_engine->tx_buf = xfer->tx_buf;
  } else {
    spi_engine->tx_buf = NULL;
  }
}

void spi_engine_rx_next(struct spi_engine *spi_engine) {
  struct spi_transfer *xfer = spi_engine->rx_xfer;

  do {
    spi_engine_xfer_next(spi_engine, &xfer);
  } while (xfer && !xfer->rx_buf);

  spi_engine->rx_xfer = xfer;
  if (xfer) {
    spi_engine->rx_length = xfer->len;
    spi_engine->rx_buf = xfer->rx_buf;
  } else {
    spi_engine->rx_buf = NULL;
  }
}

bool spi_engine_write_cmd_fifo(struct spi_engine *spi_engine) {
  void __iomem *addr = spi_engine->base + SPI_ENGINE_REG_CMD_FIFO;
  unsigned int n, m, i;
  const uint16_t *buf;

  n = readl_relaxed(spi_engine->base + SPI_ENGINE_REG_CMD_FIFO_ROOM);
  while (n && spi_engine->cmd_length) {
    m = min(n, spi_engine->cmd_length);
    buf = spi_engine->cmd_buf;
    for (i = 0; i < m; i++) writel_relaxed(buf[i], addr);
    spi_engine->cmd_buf += m;
    spi_engine->cmd_length -= m;
    n -= m;
  }

  return spi_engine->cmd_length != 0;
}

bool spi_engine_write_tx_fifo(struct spi_engine *spi_engine) {
  void __iomem *addr = spi_engine->base + SPI_ENGINE_REG_SDO_DATA_FIFO;
  unsigned int n, m, i;
  const uint8_t *buf;

  n = readl_relaxed(spi_engine->base + SPI_ENGINE_REG_SDO_FIFO_ROOM);
  while (n && spi_engine->tx_length) {
    m = min(n, spi_engine->tx_length);
    buf = spi_engine->tx_buf;
    for (i = 0; i < m; i++) writel_relaxed(buf[i], addr);
    spi_engine->tx_buf += m;
    spi_engine->tx_length -= m;
    n -= m;
    if (spi_engine->tx_length == 0) spi_engine_tx_next(spi_engine);
  }

  return spi_engine->tx_length != 0;
}

bool spi_engine_read_rx_fifo(struct spi_engine *spi_engine) {
  void __iomem *addr = spi_engine->base + SPI_ENGINE_REG_SDI_DATA_FIFO;
  unsigned int n, m, i;
  uint8_t *buf;

  n = readl_relaxed(spi_engine->base + SPI_ENGINE_REG_SDI_FIFO_LEVEL);
  while (n && spi_engine->rx_length) {
    m = min(n, spi_engine->rx_length);
    buf = spi_engine->rx_buf;
    for (i = 0; i < m; i++) buf[i] = readl_relaxed(addr);
    spi_engine->rx_buf += m;
    spi_engine->rx_length -= m;
    n -= m;
    if (spi_engine->rx_length == 0) spi_engine_rx_next(spi_engine);
  }

  return spi_engine->rx_length != 0;
}

/* spi engine setup */
int spi_engine_setup(int irq, struct axi_adc_dev *axi_adc_dev,
                     struct spi_engine *spi_engine, struct spi_master *master) {
  unsigned int version;
  int ret;
  struct device *dev = &axi_adc_dev->pdev->dev;
  /* setup spi engine */
  axi_adc_dev->spi_engine->base = axi_adc_dev->adc_virtaddr;
  version = readl_relaxed(spi_engine->base + SPI_ENGINE_REG_VERSION);
  if (SPI_ENGINE_VERSION_MAJOR(version) != 1) {
    dev_err(dev, "Unsupported peripheral version %u.%u.%c\n",
            SPI_ENGINE_VERSION_MAJOR(version),
            SPI_ENGINE_VERSION_MINOR(version),
            SPI_ENGINE_VERSION_PATCH(version));
    return -ENODEV;
  }

  /*   spi_engine->clk = devm_clk_get(&axi_adc_dev->pdev->dev, "s_axi_aclk");
    if (IS_ERR(spi_engine->clk)) {
      ret = PTR_ERR(spi_engine->clk);
      goto err_put_master;
    } */

  /*     spi_engine->ref_clk = devm_clk_get(&axi_adc_dev->pdev->dev, "spi_clk");
      if (IS_ERR(spi_engine->ref_clk)) {
        ret = PTR_ERR(spi_engine->ref_clk);
        goto err_put_master;
      } */

  /*   ret = clk_prepare_enable(spi_engine->clk);
    if (ret) goto err_put_master; */

  /*   ret = clk_prepare_enable(spi_engine->ref_clk);
    if (ret) goto err_clk_disable; */

  writel_relaxed(0x00, spi_engine->base + SPI_ENGINE_REG_RESET);
  writel_relaxed(0xff, spi_engine->base + SPI_ENGINE_REG_INT_PENDING);
  writel_relaxed(0x00, spi_engine->base + SPI_ENGINE_REG_INT_ENABLE);

  ret = request_irq(irq, spi_engine_irq, 0, axi_adc_dev->pdev->name, master);
  if (ret) goto err_ref_clk_disable;

  // ?
  master->dev.of_node = axi_adc_dev->pdev->dev.of_node;
  master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_3WIRE;
  master->bits_per_word_mask = SPI_BPW_MASK(8);
  master->max_speed_hz = 100000000 / 2;
  master->transfer_one_message = spi_engine_transfer_one_message;
  master->num_chipselect = 8;

  // register SPI master controller
  ret = spi_register_master(master);
  if (ret) goto err_free_irq;

  platform_set_drvdata(axi_adc_dev->pdev, master);

  return 0;

err_free_irq:
  free_irq(irq, master);
err_ref_clk_disable:
  dev_err(dev, "can't get the irq!\n");
  //   clk_disable_unprepare(spi_engine->ref_clk);
  /* err_clk_disable:
    clk_disable_unprepare(spi_engine->clk); */
  // err_put_master:
  //   spi_master_put(master);
  return ret;
}

int32_t spi_engine_write_and_read(struct spi_master *master, uint8_t ss,
                                  uint8_t *data, uint8_t bytes_number) {
  uint32_t expected_id = 0;
  uint32_t i;
  struct spi_message *msg;
  struct spi_transfer *xfer;
  uint8_t tx[8];
  uint8_t rx[8];

  for (i = 0; i < bytes_number; i++) tx[i] = data[i];

  // message.chip_select = ss; ok
  msg->spi->chip_select = ss;

  // msg.cs_change = true; ok
  xfer->cs_change = true;

  // msg.spi_clk = 1000000;?SPI speed 1MHz ok
  xfer->speed_hz = 1000000;

  // msg.spi_config = SPI_ENGINE_CONFIG_CPOL; ok
  msg->spi->mode = SPI_CPOL;

  // msg.tx_buf = tx; ok
  xfer->tx_buf = tx;
  // msg.rx_buf = rx; ok
  xfer->rx_buf = rx;

  // expected_id = spi.sync_id + 1;ok
  // expected_id = master->sync_id + 1;
  // spi.completed_id = 0xFFFFFFFF;ok
  // master->completed_id = 0xFFFFFFFF;

  // msg.tx_length = bytes_number;ok
  // msg.rx_length = bytes_number;ok
  xfer->len = bytes_number;

  // msg.rx_buf = rx; ok
  xfer->rx_buf = rx;

  // 通过API向spi message添加spi transfer结构，增加链表
  spi_message_add_tail(xfer, msg);

  // spi_engine_transfer_one_message(&msg);
  spi_engine_transfer_one_message(master, msg);

  /* while (master->completed_id != expected_id)
    ; */

  for (i = 0; i < bytes_number; i++) data[i] = rx[i];

  return 0;
}
