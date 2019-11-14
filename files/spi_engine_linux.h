#ifndef SPI_ENGINE_LINUX_H
#define SPI_ENGINE_LINUX_H

#include "axi-ad7616-dma.h"

/* spi engine */
#define SPI_ENGINE_VERSION_MAJOR(x) ((x >> 16) & 0xff)
#define SPI_ENGINE_VERSION_MINOR(x) ((x >> 8) & 0xff)
#define SPI_ENGINE_VERSION_PATCH(x) (x & 0xff)

#define SPI_ENGINE_REG_VERSION 0x00

#define SPI_ENGINE_REG_RESET 0x40

#define SPI_ENGINE_REG_INT_ENABLE 0x80
#define SPI_ENGINE_REG_INT_PENDING 0x84
#define SPI_ENGINE_REG_INT_SOURCE 0x88

#define SPI_ENGINE_REG_SYNC_ID 0xc0

#define SPI_ENGINE_REG_CMD_FIFO_ROOM 0xd0
#define SPI_ENGINE_REG_SDO_FIFO_ROOM 0xd4
#define SPI_ENGINE_REG_SDI_FIFO_LEVEL 0xd8

#define SPI_ENGINE_REG_CMD_FIFO 0xe0
#define SPI_ENGINE_REG_SDO_DATA_FIFO 0xe4
#define SPI_ENGINE_REG_SDI_DATA_FIFO 0xe8
#define SPI_ENGINE_REG_SDI_DATA_FIFO_PEEK 0xec

#define SPI_ENGINE_REG_OFFLOAD_CTRL(x) (0x100 + (0x20 * x))
#define SPI_ENGINE_REG_OFFLOAD_STATUS(x) (0x104 + (0x20 * x))
#define SPI_ENGINE_REG_OFFLOAD_RESET(x) (0x108 + (0x20 * x))
#define SPI_ENGINE_REG_OFFLOAD_CMD_MEM(x) (0x110 + (0x20 * x))
#define SPI_ENGINE_REG_OFFLOAD_SDO_MEM(x) (0x114 + (0x20 * x))

#define SPI_ENGINE_OFFLOAD_CTRL_ENABLE BIT(0)
#define SPI_ENGINE_OFFLOAD_STATUS_ENABLED BIT(0)

#define SPI_ENGINE_INT_CMD_ALMOST_EMPTY BIT(0)
#define SPI_ENGINE_INT_SDO_ALMOST_EMPTY BIT(1)
#define SPI_ENGINE_INT_SDI_ALMOST_FULL BIT(2)
#define SPI_ENGINE_INT_SYNC BIT(3)

#define SPI_ENGINE_CONFIG_CPHA BIT(0)
#define SPI_ENGINE_CONFIG_CPOL BIT(1)
#define SPI_ENGINE_CONFIG_3WIRE BIT(2)

#define SPI_ENGINE_INST_TRANSFER 0x0
#define SPI_ENGINE_INST_ASSERT 0x1
#define SPI_ENGINE_INST_WRITE 0x2
#define SPI_ENGINE_INST_MISC 0x3

#define SPI_ENGINE_CMD_REG_CLK_DIV 0x0
#define SPI_ENGINE_CMD_REG_CONFIG 0x1

#define SPI_ENGINE_MISC_SYNC 0x0
#define SPI_ENGINE_MISC_SLEEP 0x1

#define SPI_ENGINE_TRANSFER_WRITE 0x1
#define SPI_ENGINE_TRANSFER_READ 0x2

#define SPI_ENGINE_CMD(inst, arg1, arg2) \
  (((inst) << 12) | ((arg1) << 8) | (arg2))

#define SPI_ENGINE_CMD_TRANSFER(flags, n) \
  SPI_ENGINE_CMD(SPI_ENGINE_INST_TRANSFER, (flags), (n))
#define SPI_ENGINE_CMD_ASSERT(delay, cs) \
  SPI_ENGINE_CMD(SPI_ENGINE_INST_ASSERT, (delay), (cs))
#define SPI_ENGINE_CMD_WRITE(reg, val) \
  SPI_ENGINE_CMD(SPI_ENGINE_INST_WRITE, (reg), (val))
#define SPI_ENGINE_CMD_SLEEP(delay) \
  SPI_ENGINE_CMD(SPI_ENGINE_INST_MISC, SPI_ENGINE_MISC_SLEEP, (delay))
#define SPI_ENGINE_CMD_SYNC(id) \
  SPI_ENGINE_CMD(SPI_ENGINE_INST_MISC, SPI_ENGINE_MISC_SYNC, (id))

struct spi_engine_program {
  unsigned int length;
  uint16_t instructions[];
};

struct spi_engine {
  struct clk *clk;
  struct clk *ref_clk;

  spinlock_t lock;

  // void *base;
  void __iomem *base;
  //   unsigned int base;

  struct spi_message *msg;
  struct spi_engine_program *p;
  unsigned cmd_length;
  const uint16_t *cmd_buf;

  struct spi_transfer *tx_xfer;
  unsigned int tx_length;
  const uint8_t *tx_buf;

  struct spi_transfer *rx_xfer;
  unsigned int rx_length;
  uint8_t *rx_buf;

  unsigned int sync_id;
  unsigned int completed_id;

  unsigned int int_enable;
};

// spi engine transfer one message
int spi_engine_transfer_one_message(struct spi_master *master,
                                    struct spi_message *msg);

// interrupt handler
irqreturn_t spi_engine_irq(int irq, void *devid);

void spi_engine_program_add_cmd(struct spi_engine_program *p, bool dry,
                                uint16_t cmd);

unsigned int spi_engine_get_config(struct spi_device *spi);

unsigned int spi_engine_get_clk_div(struct spi_engine *spi_engine,
                                    struct spi_device *spi,
                                    struct spi_transfer *xfer);

void spi_engine_gen_xfer(struct spi_engine_program *p, bool dry,
                         struct spi_transfer *xfer);

void spi_engine_gen_sleep(struct spi_engine_program *p, bool dry,
                          struct spi_engine *spi_engine, unsigned int clk_div,
                          unsigned int delay);

void spi_engine_gen_cs(struct spi_engine_program *p, bool dry,
                       struct spi_device *spi, bool assert);

int spi_engine_compile_message(struct spi_engine *spi_engine,
                               struct spi_message *msg, bool dry,
                               struct spi_engine_program *p);

void spi_engine_xfer_next(struct spi_engine *spi_engine,
                          struct spi_transfer **_xfer);

void spi_engine_tx_next(struct spi_engine *spi_engine);

void spi_engine_rx_next(struct spi_engine *spi_engine);

bool spi_engine_write_cmd_fifo(struct spi_engine *spi_engine);

bool spi_engine_write_tx_fifo(struct spi_engine *spi_engine);

bool spi_engine_read_rx_fifo(struct spi_engine *spi_engine);

void spi_engine_setup(int irq, struct axi_adc_dev *axi_adc_dev,
                      struct spi_engine *spi_engine, struct spi_master *master);

int32_t spi_engine_write_and_read(struct spi_engine *master, uint8_t ss,
                                  uint8_t *data, uint8_t bytes_number);

int32_t spi_engine_write(uint32_t reg_addr, uint32_t reg_data);

int32_t spi_engine_read(uint32_t reg_addr, uint32_t *reg_data);

#endif