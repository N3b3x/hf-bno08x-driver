#include "rvc_hal.h"
#include <driver/uart.h>
#include <driver/gpio.h>
#include "esp_timer.h"
#include <string.h>

#define UART_PORT UART_NUM_0
#define UART_TX GPIO_NUM_21
#define UART_RX GPIO_NUM_20
#define PIN_RST GPIO_NUM_9
#define PIN_BOOT GPIO_NUM_10
#define BUF_SIZE 256
#define FRAME_LEN 19

static RvcHalC_t hal;
static uint8_t frame[FRAME_LEN];
static size_t len = 0;
static bool ready = false;

static bool checksum(const uint8_t *f)
{
    uint8_t sum = 0;
    for (int i = 2; i < FRAME_LEN - 1; ++i)
        sum += f[i];
    return sum == f[FRAME_LEN - 1];
}

static void process(uint8_t c)
{
    if (len == FRAME_LEN) {
        memmove(frame, frame + 1, FRAME_LEN - 1);
        frame[FRAME_LEN - 1] = c;
    } else {
        frame[len++] = c;
    }
    if (len == FRAME_LEN && frame[0] == 0xAA && frame[1] == 0xAA &&
        checksum(frame)) {
        ready = true;
    }
}

static int hal_open(void *ctx)
{
    const uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_PORT, &cfg);
    uart_set_pin(UART_PORT, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT, BUF_SIZE, 0, 0, NULL, 0);
    gpio_set_level(PIN_BOOT, 1);
    gpio_set_level(PIN_RST, 0);
    ets_delay_us(10000);
    gpio_set_level(PIN_RST, 1);
    len = 0;
    ready = false;
    return RVC_OK;
}

static void hal_close(void *ctx)
{
    uart_driver_delete(UART_PORT);
}

static int hal_read(void *ctx, rvc_SensorEvent_t *event)
{
    uint8_t c;
    while (uart_read_bytes(UART_PORT, &c, 1, 0) == 1) {
        process(c);
    }
    if (ready) {
        event->timestamp_uS = esp_timer_get_time();
        event->index = frame[2];
        event->yaw = (int16_t)((frame[4] << 8) | frame[3]);
        event->pitch = (int16_t)((frame[6] << 8) | frame[5]);
        event->roll = (int16_t)((frame[8] << 8) | frame[7]);
        event->acc_x = (int16_t)((frame[10] << 8) | frame[9]);
        event->acc_y = (int16_t)((frame[12] << 8) | frame[11]);
        event->acc_z = (int16_t)((frame[14] << 8) | frame[13]);
        event->mi = frame[15];
        event->mr = frame[16];
        ready = false;
        len = 0;
        return 1;
    }
    return 0;
}

RvcHalC_t *rvc_hal_init(void)
{
    hal.ctx = NULL;
    hal.open = hal_open;
    hal.close = hal_close;
    hal.read = hal_read;
    return &hal;
}
