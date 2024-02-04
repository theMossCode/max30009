#ifndef SPI_FUNCTIONS
#define SPI_FUNCTIONS

#include <stdint.h>

#define SPI_CS_ACTIVE           0
#define SPI_CS_INACTIVE         1

int spi_init();

void spi_cs_set(int value);

int spi_read_reg(uint8_t reg, uint8_t *data);

int spi_burst_read(uint8_t reg, uint8_t *data, uint8_t data_len);

int spi_write_reg(uint8_t reg, uint8_t data);

#endif