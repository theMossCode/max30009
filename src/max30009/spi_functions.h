#ifndef SPI_FUNCTIONS
#define SPI_FUNCTIONS

#include <stdint.h>

int spi_init();

int spi_read_reg(uint8_t reg, uint8_t *data);

int spi_burst_read(uint8_t reg, uint8_t *data, uint8_t data_len);

int spi_write_reg(uint8_t reg, uint8_t data);

#endif