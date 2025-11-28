#pragma once
#include "led_strip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

// Globale Handle für LED-Strip
extern led_strip_handle_t led_strip;

// Initialisierung
void led_strip_init();

// Farbe setzen
void led_set_color(uint8_t r, uint8_t g, uint8_t b);

// Farbflash
void led_flash(uint8_t r, uint8_t g, uint8_t b, int ms);
