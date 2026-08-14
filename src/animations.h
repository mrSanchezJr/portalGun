#pragma once
#include <stdint.h>

enum class WS2812Anim : uint8_t {
    IDLE        = 0,
    PORTAL_WAVE = 1,
    RED_FLASH   = 2,
    OFF_REQ     = 3
};

enum class LED1WAnim : uint8_t {
    IDLE     = 0,
    FADE_IN  = 1,
    FADE_OUT = 2
};

void animations_init(void);

void anim_ws_start_portal(void);
void anim_ws_start_red_flash(void);
void anim_ws_stop(void);
WS2812Anim anim_ws_get_state(void);
bool anim_ws_flash_done(void);

void anim_led_fade_in(uint8_t target_step);
void anim_led_fade_out(void);
LED1WAnim anim_led_get_state(void);
bool anim_led_done(void);

void animations_update(void);