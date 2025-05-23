#include "sh2_hal.h"
#include "sh2_hal_init.h"
#include "sh2_err.h"
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <string.h>

#define SPI_HOST    SPI2_HOST
#define PIN_MISO    2
#define PIN_MOSI    3
#define PIN_CLK     4
#define PIN_CS      5

static spi_device_handle_t dev;
static sh2_Hal_t sh2_hal;

static int spi_open(sh2_Hal_t *self)
{
    spi_bus_config_t bus = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 1,
    };
    esp_err_t err = spi_bus_initialize(SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) return SH2_ERR_IO;
    err = spi_bus_add_device(SPI_HOST, &devcfg, &dev);
    if (err != ESP_OK) return SH2_ERR_IO;
    return SH2_OK;
}

static void spi_close(sh2_Hal_t *self)
{
    spi_bus_remove_device(dev);
    spi_bus_free(SPI_HOST);
}

static int spi_read(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t)
{
    spi_transaction_t tdesc = {0};
    tdesc.length = len * 8;
    tdesc.rx_buffer = buf;
    esp_err_t err = spi_device_transmit(dev, &tdesc);
    if (err == ESP_OK) return len; else return SH2_ERR_IO;
}

static int spi_write(sh2_Hal_t *self, uint8_t *buf, unsigned len)
{
    spi_transaction_t tdesc = {0};
    tdesc.length = len * 8;
    tdesc.tx_buffer = buf;
    esp_err_t err = spi_device_transmit(dev, &tdesc);
    return err == ESP_OK ? len : SH2_ERR_IO;
}

static uint32_t spi_time(sh2_Hal_t *self)
{
    return (uint32_t)esp_timer_get_time();
}

sh2_Hal_t *shtp_spi_hal_init(void)
{
    sh2_hal.open = spi_open;
    sh2_hal.close = spi_close;
    sh2_hal.read = spi_read;
    sh2_hal.write = spi_write;
    sh2_hal.getTimeUs = spi_time;
    return &sh2_hal;
}
