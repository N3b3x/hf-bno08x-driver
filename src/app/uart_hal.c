#include "uart_hal.h"
#include "sh2_hal_init.h"
#include "sh2_err.h"
#include <driver/uart.h>
#include <string.h>
#include "esp_timer.h"

#define UART_PORT UART_NUM_1
#define UART_TX 17
#define UART_RX 18
#define UART_BAUD 3000000

static uart_hal_t *hal;

static esp_err_t init_uart(void)
{
    const uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);
    if (err != ESP_OK) return err;
    uart_param_config(UART_PORT, &cfg);
    return uart_set_pin(UART_PORT, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static int uart_open(sh2_Hal_t *self_)
{
    hal = (uart_hal_t *)self_;
    if (init_uart() != ESP_OK) return SH2_ERR_IO;
    return SH2_OK;
}

static void uart_close(sh2_Hal_t *self)
{
    uart_driver_delete(UART_PORT);
}

static int uart_read(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t)
{
    int ret = uart_read_bytes(UART_PORT, buf, len, 50 / portTICK_PERIOD_MS);
    if(ret >= 0){ return ret; }
    return SH2_ERR_IO;
}

static int uart_write(sh2_Hal_t *self, uint8_t *buf, unsigned len)
{
    int ret = uart_write_bytes(UART_PORT, (const char*)buf, len);
    return ret >= 0 ? ret : SH2_ERR_IO;
}

static uint32_t uart_time(sh2_Hal_t *self)
{
    return (uint32_t)esp_timer_get_time();
}

sh2_Hal_t *shtp_uart_hal_init(uart_hal_t *pHal, bool dfu)
{
    memset(pHal,0,sizeof(*pHal));
    pHal->dfu = dfu;
    pHal->sh2_hal.open = uart_open;
    pHal->sh2_hal.close = uart_close;
    pHal->sh2_hal.read = uart_read;
    pHal->sh2_hal.write = uart_write;
    pHal->sh2_hal.getTimeUs = uart_time;
    return &pHal->sh2_hal;
}

sh2_Hal_t *bno_dfu_uart_hal_init(uart_hal_t *pHal, bool dfu)
{
    return shtp_uart_hal_init(pHal, dfu);
}
