#include "button.h"
#include <driver/gpio.h>

#define BUTTON_GPIO GPIO_NUM_9

static button_cb_t onUp = NULL;
static button_cb_t onDown = NULL;
static button_cb_t onShort = NULL;
static button_cb_t onLong = NULL;

static bool last_state = true;
static uint32_t last_change;

void button_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    last_state = gpio_get_level(BUTTON_GPIO);
    last_change = 0;
}

void button_onUp(button_cb_t cb){ onUp = cb; }
void button_onDown(button_cb_t cb){ onDown = cb; }
void button_onShort(button_cb_t cb){ onShort = cb; }
void button_onLong(button_cb_t cb){ onLong = cb; }

bool button_poll(uint32_t now_us)
{
    bool state = gpio_get_level(BUTTON_GPIO);
    if(state != last_state){
        if(!state && onDown) onDown();
        if(state && onUp){
            if(now_us - last_change < 500000 && onShort) onShort();
            onUp();
        }
        last_state = state;
        last_change = now_us;
    }else if(!state && (now_us - last_change > 500000)){
        if(onLong){ onLong(); }
        last_change = now_us + 1000000; // avoid repeat
    }
    return !state;
}
