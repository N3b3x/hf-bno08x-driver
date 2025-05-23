#include "dbg.h"
#include <driver/gpio.h>

#define DEBUG_GPIO GPIO_NUM_0
#define DEBUG2_GPIO GPIO_NUM_2

void dbg_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL<<DEBUG_GPIO) | (1ULL<<DEBUG2_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cfg);
}

void dbg_pulse(unsigned count)
{
    for(unsigned i=0;i<count;i++){
        gpio_set_level(DEBUG_GPIO,1);
        gpio_set_level(DEBUG_GPIO,0);
    }
}
void dbg_set(void){ gpio_set_level(DEBUG_GPIO,1); }
void dbg_clear(void){ gpio_set_level(DEBUG_GPIO,0); }
void dbg2_pulse(unsigned count){
    for(unsigned i=0;i<count;i++){gpio_set_level(DEBUG2_GPIO,1);gpio_set_level(DEBUG2_GPIO,0);} }
void dbg2_set(void){ gpio_set_level(DEBUG2_GPIO,1); }
void dbg2_clear(void){ gpio_set_level(DEBUG2_GPIO,0); }
void dbg_manchester(unsigned byte){
    for(int i=7;i>=0;i--){
        int b=(byte>>i)&1; gpio_set_level(DEBUG_GPIO,!b); gpio_set_level(DEBUG_GPIO,b); }
}
