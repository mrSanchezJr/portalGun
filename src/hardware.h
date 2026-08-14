#pragma once
#include <stdint.h>
#include <Arduino.h>

// --- Button Events ---
enum class BtnEvent : uint8_t {
    NONE      = 0,
    SHORT     = 1,
    LONG      = 2,
    VERY_LONG = 3
};

// --- Power Management ---
void power_init(void);
void power_enter_sleep(void);
void power_init_watchdog(void);
void power_kick_watchdog(void);

// --- 1W LEDs ---
void led1w_init(void);
void led1w_set_duty(uint8_t ch, uint16_t duty);
void led1w_set_step(uint8_t step);
uint8_t led1w_get_step(void);
void led1w_off(void);

// --- WS2812B ---
void ws2812_init(void);
void ws2812_set_pixel(uint8_t idx, uint8_t r, uint8_t g, uint8_t b);
void ws2812_fill(uint8_t r, uint8_t g, uint8_t b);
void ws2812_show(void);
void ws2812_off(void);

// --- Battery ADC ---
void battery_init(void);
void battery_update(void);
float battery_voltage(void);
uint8_t battery_percent(void);
bool battery_is_critical(void);
bool battery_is_low(void);

// --- Encoder & Button ---
void encoder_init(void);
void encoder_update(void);
int8_t encoder_get_delta(void);
BtnEvent encoder_get_event(void);
bool encoder_btn_pressed(void);
uint32_t encoder_btn_held_ms(void);
void encoder_disable_interrupts(void);
void encoder_enable_interrupts(void);

// --- TM1637 Display ---
void display_init(void);
void display_set_brightness(uint8_t brightness);
void display_show_counter(uint16_t value);
void display_show_brightness(uint8_t value);
void display_show_bat_label(void);
void display_show_voltage(float voltage);
void display_show_battery_percentage(uint8_t percent);
void display_show_hu(void);
void display_off(void);
