
#ifndef AD7616_H_
#define AD7616_H_

#include <stdin.h>
#include "axi-ad7616-dma.h"
#include "spi_engine_linux.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD7616_REG_CONFIG 0x02
#define AD7616_REG_CHANNEL 0x03
#define AD7616_REG_INPUT_RANGE_A1 0x04
#define AD7616_REG_INPUT_RANGE_A2 0x05
#define AD7616_REG_INPUT_RANGE_B1 0x06
#define AD7616_REG_INPUT_RANGE_B2 0x07
#define AD7616_REG_SEQUENCER_STACK(x) (0x20 + (x))

/* AD7616_REG_CONFIG */
#define AD7616_SDEF (1 << 7)
#define AD7616_BURSTEN (1 << 6)
#define AD7616_SEQEN (1 << 5)
#define AD7616_OS(x) (((x)&0x7) << 2)
#define AD7616_STATUSEN (1 << 1)
#define AD7616_CRCEN (1 << 0)

/* AD7616_REG_INPUT_RANGE */
#define AD7616_INPUT_RANGE(ch, x) (((x)&0x3) << (((ch)&0x3) * 2))

/* AD7616_REG_SEQUENCER_STACK(x) */
#define AD7616_ADDR(x) (((x)&0x7F) << 9)
#define AD7616_SSREN (1 << 8)
#define AD7616_BSEL(x) (((x)&0xF) << 4)
#define AD7616_ASEL(x) (((x)&0xF) << 0)

/* AD7616_REG_STATUS */
#define AD7616_STATUS_A(x) (((x)&0xF) << 12)
#define AD7616_STATUS_B(x) (((x)&0xF) << 8)
#define AD7616_STATUS_CRC(x) (((x)&0xFF) << 0)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
typedef enum {
  AD7616_SW,
  AD7616_HW,
} ad7616_mode;

typedef enum {
  AD7616_SERIAL,
  AD7616_PARALLEL,
} ad7616_interface;

typedef enum {
  AD7616_VA0,
  AD7616_VA1,
  AD7616_VA2,
  AD7616_VA3,
  AD7616_VA4,
  AD7616_VA5,
  AD7616_VA6,
  AD7616_VA7,
  AD7616_VB0,
  AD7616_VB1,
  AD7616_VB2,
  AD7616_VB3,
  AD7616_VB4,
  AD7616_VB5,
  AD7616_VB6,
  AD7616_VB7,
} ad7616_ch;

typedef enum {
  AD7616_2V5 = 1,
  AD7616_5V = 2,
  AD7616_10V = 3,
} ad7616_range;

typedef enum {
  AD7616_OSR_0,
  AD7616_OSR_2,
  AD7616_OSR_4,
  AD7616_OSR_8,
  AD7616_OSR_16,
  AD7616_OSR_32,
  AD7616_OSR_64,
  AD7616_OSR_128,
} ad7616_osr;

struct ad7616_dev {
  /* SPI */
  // spi_device spi_dev;

  /* GPIO */
  // gpio_device gpio_dev;
  // int8_t gpio_hw_rngsel0;
  // int8_t gpio_hw_rngsel1;
  // int8_t gpio_reset;
  // int8_t gpio_os0;
  // int8_t gpio_os1;
  // int8_t gpio_os2;

  // gpio reset
  struct gpio_desc *gpio_reset;

  /* Device Settings */
  ad7616_interface interface;
  ad7616_mode mode;
  ad7616_range va[8];
  ad7616_range vb[8];
  ad7616_osr osr;
  struct adc_core *core;
};

struct ad7616_init_param {
  /* SPI */
  uint8_t spi_chip_select;
  spi_mode spi_mode;
  spi_type spi_type;
  uint32_t spi_device_id;
  /* GPIO */
  gpio_type gpio_type;
  uint32_t gpio_device_id;
  int8_t gpio_hw_rngsel0;
  int8_t gpio_hw_rngsel1;
  int8_t gpio_reset;
  int8_t gpio_os0;
  int8_t gpio_os1;
  int8_t gpio_os2;
  /* Device Settings */
  ad7616_mode mode;
  ad7616_range va[8];
  ad7616_range vb[8];
  ad7616_osr osr;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* SPI read from device. */
int32_t ad7616_read(struct spi_engine *master, struct ad7616_dev *dev,
                    uint8_t reg_addr, uint16_t *reg_data);
/* SPI write to device. */
int32_t ad7616_write(struct spi_engine *master, struct ad7616_dev *dev,
                     uint8_t reg_addr, uint16_t reg_data);
/* SPI read from device using a mask. */
int32_t ad7616_read_mask(struct spi_engine *master, struct ad7616_dev *dev,
                         uint8_t reg_addr, uint16_t mask, uint16_t *data);
/* SPI write to device using a mask. */
int32_t ad7616_write_mask(struct spi_engine *master, struct ad7616_dev *dev,
                          uint8_t reg_addr, uint16_t mask, uint16_t data);
/* SPI read from device. */
int32_t ad7616_spi_read(struct spi_engine *master, struct ad7616_dev *dev,
                        uint8_t reg_addr, uint16_t *reg_data);
/* SPI write to device. */
int32_t ad7616_spi_write(struct spi_engine *master, struct ad7616_dev *dev,
                         uint8_t reg_addr, uint16_t reg_data);
/* PAR read from device. */
int32_t ad7616_par_read(struct ad7616_dev *dev, uint8_t reg_addr,
                        uint16_t *reg_data);
/* PAR write to device. */
int32_t ad7616_par_write(struct ad7616_dev *dev, uint8_t reg_addr,
                         uint16_t reg_data);
/* Perform a full reset of the device. */
int32_t ad7616_reset(struct ad7616_dev *dev);
/* Set the analog input range for the selected analog input channel. */
int32_t ad7616_set_range(struct spi_engine *master, struct ad7616_dev *dev,
                         ad7616_ch ch, ad7616_range range);
/* Set the operation mode (software or hardware). */
int32_t ad7616_set_mode(struct spi_engine *master, struct ad7616_dev *dev,
                        ad7616_mode mode);
/* Set the oversampling ratio. */
int32_t ad7616_set_oversampling_ratio(struct spi_engine *master,
                                      struct ad7616_dev *dev, ad7616_osr osr);
/* Initialize the device. */
// int32_t ad7616_setup(ad7616_dev **device, adc_core *core,
//                      ad7616_init_param init_param);

void ad7616_setup(struct axi_adc_dev *axi_adc_dev,
                  struct ad7616_init_param init_param);

#endif
