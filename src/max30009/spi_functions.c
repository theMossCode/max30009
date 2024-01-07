#include "spi_functions.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(MAX30009_SPI_FUNCTIONS, LOG_LEVEL_DBG);

#define MAX30009_SPI_NODE     DT_ALIAS(max30009_spi) 
#define MAX30009_CS_NODE      DT_NODELABEL(max30009_cs)

#define MAX30009_SPI_READ       0x80
#define MAX30009_SPI_WRITE      0x00

static const struct device *spi_device = DEVICE_DT_GET(MAX30009_SPI_NODE);
static const struct spi_config max30009_cfg = {
    .cs = {{0}},
    .slave = 0,
    .frequency = 1000000,
    .operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA, // MODE 3
};

static struct gpio_dt_spec max30009_cs_dt = GPIO_DT_SPEC_GET(MAX30009_CS_NODE, gpios);

static void spi_cs_set(uint8_t value)
{
    gpio_pin_set_dt(&max30009_cs_dt, value);
}


int spi_init()
{
    if(!gpio_is_ready_dt(&max30009_cs_dt)){
        LOG_ERR("CS GPIO not ready");
        return -ENODEV;
    }

    if(gpio_pin_configure_dt(&max30009_cs_dt, GPIO_OUTPUT_INACTIVE)){
        LOG_ERR("Config CS fail");
        return -EIO;
    }

    spi_cs_set(0);

    if(!device_is_ready(spi_device)){
        LOG_ERR("SPI device not ready");
        return -ENODEV;
    }

    LOG_INF("SPI Init complete");

    return 0;
}

int spi_read_reg(uint8_t reg, uint8_t *data)
{
    int err = 0;

    uint8_t tx_data[] = {
        reg, MAX30009_SPI_READ
    };

    struct spi_buf tx_buf = {
        .buf = tx_data,
        .len = 2,
    };

    struct spi_buf rx_buf = {
        .buf = data,
        .len = 1
    };

    struct spi_buf_set rx_buf_set = {
        .buffers = &rx_buf,
        .count = 1
    };

    struct spi_buf_set tx_buf_set = {
        .buffers = &tx_buf,
        .count = 1
    };

    spi_cs_set(1);

    err = spi_transceive(spi_device, &max30009_cfg, &tx_buf_set, &rx_buf_set);
    if(err){
        LOG_ERR("SPI read fail, err %d", err);
        return err;
    }

    spi_cs_set(0);

    LOG_DBG("SPI reg read complete, value 0x%02x", *data);

    return 0;
}

int spi_burst_read(uint8_t reg, uint8_t *data, uint8_t data_len)
{
    int err = 0;

    uint8_t tx_data[] = {
        reg, MAX30009_SPI_READ
    };

    struct spi_buf tx_buf = {
        .buf = tx_data,
        .len = 2,
    };

    struct spi_buf rx_buf = {
        .buf = data,
        .len = data_len
    };

    struct spi_buf_set rx_buf_set = {
        .buffers = &rx_buf,
        .count = 1
    };

    struct spi_buf_set tx_buf_set = {
        .buffers = &tx_buf,
        .count = 1
    };

    spi_cs_set(1);

    err = spi_transceive(spi_device, &max30009_cfg, &tx_buf_set, &rx_buf_set);
    if(err){
        LOG_ERR("SPI read fail, err %d", err);
        return err;
    }

    spi_cs_set(0);

    LOG_DBG("SPI reg read complete, value 0x%02x", *data);

    return 0;
}

int spi_write_reg(uint8_t reg, uint8_t data)
{
    int err = 0;

    uint8_t tx_data[] = {
        reg, MAX30009_SPI_WRITE
    };


    struct spi_buf tx_buf = {
        .buf = tx_data,
        .len = 2
    };

    struct spi_buf_set tx_buf_set = {
        .buffers = &tx_buf,
        .count = 1
    };

    spi_cs_set(1);

    err = spi_write(spi_device, &max30009_cfg, &tx_buf_set);
    if(err){
        LOG_ERR("SPI write fail, err %d", err);
        return err;
    }

    spi_cs_set(0);

    return 0;
}
