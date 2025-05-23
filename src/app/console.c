#include "console.h"
#include <driver/uart.h>
#include <string.h>
#include <stdio.h>

#define CONSOLE_PORT UART_NUM_0
#define CONSOLE_TX 1
#define CONSOLE_RX 3
#define CONSOLE_BAUD 115200

void console_init(void)
{
    const uart_config_t cfg = {
        .baud_rate = CONSOLE_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(CONSOLE_PORT, 1024, 0, 0, NULL, 0);
    uart_param_config(CONSOLE_PORT, &cfg);
    uart_set_pin(CONSOLE_PORT, CONSOLE_TX, CONSOLE_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
