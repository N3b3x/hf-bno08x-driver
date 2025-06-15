#include "i2c_hal.h"
#include "sh2_hal_init.h"
#include "sh2_err.h"
#include "esp_timer.h"
#include <driver/i2c.h>
#include <string.h>
#include <sys/param.h>

// ESP32 I2C configuration for BNO08x
#define I2C_PORT    I2C_NUM_0
#define I2C_SCL_IO  4
#define I2C_SDA_IO  5
#define I2C_FREQ_HZ 400000

static i2c_hal_t *hal;

static esp_err_t init_bus(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_PORT, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

static int i2c_open(sh2_Hal_t *self_)
{
    hal = (i2c_hal_t *)self_;
    if (init_bus() != ESP_OK) return SH2_ERR_IO;
    return SH2_OK;
}

static void i2c_close(sh2_Hal_t *self)
{
    i2c_driver_delete(I2C_PORT);
}

static int i2c_read(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t)
{
    int ret = i2c_master_read_from_device(I2C_PORT, hal->i2c_addr, buf, len, 50 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) return len; 
    return SH2_ERR_IO;
}

static int i2c_write(sh2_Hal_t *self, uint8_t *buf, unsigned len)
{
    int ret = i2c_master_write_to_device(I2C_PORT, hal->i2c_addr, buf, len, 50 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) return len;
    return SH2_ERR_IO;
}

static uint32_t i2c_get_time(sh2_Hal_t *self)
{
    return (uint32_t)(esp_timer_get_time());
}

sh2_Hal_t *shtp_i2c_hal_init(i2c_hal_t *pHal, bool dfu, uint8_t addr)
{
    memset(pHal, 0, sizeof(*pHal));
    pHal->dfu = dfu;
    pHal->i2c_addr = addr;
    pHal->sh2_hal.open = i2c_open;
    pHal->sh2_hal.close = i2c_close;
    pHal->sh2_hal.read = i2c_read;
    pHal->sh2_hal.write = i2c_write;
    pHal->sh2_hal.getTimeUs = i2c_get_time;
    return &pHal->sh2_hal;
}

sh2_Hal_t *bno_dfu_i2c_hal_init(i2c_hal_t *pHal, bool dfu, uint8_t addr)
{
    // DFU uses same transport on ESP32
    return shtp_i2c_hal_init(pHal, dfu, addr);
}
