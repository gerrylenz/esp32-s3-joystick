#include "led_control.hpp"

#define LED_STRIP_PIN GPIO_NUM_48
#define LED_STRIP_LENGTH 1
#define LED_BRIGHTNESS 0.3f //30%

led_strip_handle_t led_strip = nullptr;

void led_strip_init()
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_PIN,
        .max_leds = LED_STRIP_LENGTH,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {.invert_out = false}};
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags = {.with_dma = false}};
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_ERROR_CHECK(led_strip_clear(led_strip));
}

void led_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    if (!led_strip)
        return;
    r = (uint8_t)(r * LED_BRIGHTNESS);
    g = (uint8_t)(g * LED_BRIGHTNESS);
    b = (uint8_t)(b * LED_BRIGHTNESS);

    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

void led_flash(uint8_t r, uint8_t g, uint8_t b, int ms)
{
    led_set_color(r, g, b);
    vTaskDelay(pdMS_TO_TICKS(ms));
    led_set_color(0, 0, 0);
}
