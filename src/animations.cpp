#include "animations.h"
#include "config.h"
#include "hardware.h"
#include <Arduino.h>
#include <math.h>

static const uint32_t WS_UPDATE_MS    = 20;
static const uint32_t LED_UPDATE_MS   = 10;
static const uint8_t  RED_FLASH_COUNT = 3;

static WS2812Anim s_ws_state    = WS2812Anim::IDLE;
static uint32_t   s_ws_last_ms  = 0;
static float      s_ws_phase    = 0.0f;
static bool       s_flash_done  = false;
static uint8_t    s_flash_count = 0;
static uint8_t    s_flash_sub   = 0;
static uint32_t   s_flash_ms    = 0;

static LED1WAnim  s_led_state    = LED1WAnim::IDLE;
static uint32_t   s_led_start_ms = 0;
static uint8_t    s_led_target   = 0;
static uint16_t   s_led_from[3]  = {0, 0, 0};
static uint16_t   s_led_to[3]    = {0, 0, 0};
static bool       s_led_done     = false;
static uint32_t   s_led_last_ms  = 0;
static bool       s_led_reverse  = false;

static const uint32_t LED_DELAYS[3] = {
    LED1_FADE_DELAY_MS,
    LED2_FADE_DELAY_MS,
    LED3_FADE_DELAY_MS
};

static inline uint16_t lerp16(uint16_t a, uint16_t b, float t) {
    return (uint16_t)(a + (b - a) * t);
}

static inline uint16_t step_to_duty_local(uint8_t step) {
    if (step == 0) return 0;
    if (step >= LED1W_MAX_STEP) return 255;
    float ratio = (float)step / (float)LED1W_MAX_STEP;
    return (uint16_t)(ratio * ratio * 255.0f);
}

static void update_ws_portal(uint32_t now) {
    if (now - s_ws_last_ms < WS_UPDATE_MS) return;
    s_ws_last_ms = now;

    s_ws_phase += 0.0628f;
    if (s_ws_phase > 2.0f * M_PI) s_ws_phase -= 2.0f * M_PI;

    for (uint8_t i = 0; i < WS2812_NUM; i++) {
        float phi = s_ws_phase + i * (2.0f * M_PI / WS2812_NUM);
        float brightness = 0.65f + 0.35f * sinf(phi);

        uint8_t g = (uint8_t)(brightness * 220);
        uint8_t b = (uint8_t)(brightness * 50 * (0.5f + 0.5f * sinf(phi + 1.0f)));
        uint8_t r = 0;

        ws2812_set_pixel(i, r, g, b);
    }
    ws2812_show();
}

static void update_ws_red_flash(uint32_t now) {
    if (s_flash_done) return;

    if (s_flash_sub == 0) {
        ws2812_fill(220, 0, 0);
        ws2812_show();
        s_flash_ms = now;
        s_flash_sub = 1;
    } else {
        uint32_t elapsed = now - s_flash_ms;
        if (s_flash_sub == 1 && elapsed >= LOW_BAT_FLASH_MS) {
            ws2812_off();
            s_flash_ms = now;
            s_flash_sub = 2;
        } else if (s_flash_sub == 2 && elapsed >= LOW_BAT_FLASH_MS) {
            s_flash_count++;
            s_flash_sub = 0;
            if (s_flash_count >= RED_FLASH_COUNT) {
                s_flash_done = true;
                s_ws_state = WS2812Anim::PORTAL_WAVE;
            }
        }
    }
}

static void update_led_fade(uint32_t now) {
    if (s_led_done) return;
    if (now - s_led_last_ms < LED_UPDATE_MS) return;
    s_led_last_ms = now;

    uint32_t elapsed = now - s_led_start_ms;
    bool all_done = true;

    for (uint8_t ch = 0; ch < 3; ch++) {
        uint8_t delay_idx = s_led_reverse ? (2 - ch) : ch;
        uint32_t delay = LED_DELAYS[delay_idx];

        if (elapsed < delay) {
            led1w_set_duty(ch + 1, s_led_from[ch]);
            all_done = false;
            continue;
        }

        uint32_t ch_elapsed = elapsed - delay;

        if (ch_elapsed >= FADE_DURATION_MS) {
            led1w_set_duty(ch + 1, s_led_to[ch]);
        } else {
            float t = (float)ch_elapsed / (float)FADE_DURATION_MS;
            led1w_set_duty(ch + 1, lerp16(s_led_from[ch], s_led_to[ch], t));
            all_done = false;
        }
    }

    if (all_done) {
        s_led_done = true;
        s_led_state = LED1WAnim::IDLE;
    }
}

void animations_init(void) {
    s_ws_state = WS2812Anim::IDLE;
    s_led_state = LED1WAnim::IDLE;
    s_led_done = true;
    s_flash_done = false;
}

void anim_ws_start_portal(void) {
    s_ws_state = WS2812Anim::PORTAL_WAVE;
    s_ws_phase = 0.0f;
    s_ws_last_ms = 0;
}

void anim_ws_start_red_flash(void) {
    s_ws_state = WS2812Anim::RED_FLASH;
    s_flash_done = false;
    s_flash_count = 0;
    s_flash_sub = 0;
    s_flash_ms = millis();
}

void anim_ws_stop(void) {
    s_ws_state = WS2812Anim::IDLE;
    ws2812_off();
}

WS2812Anim anim_ws_get_state(void) {
    return s_ws_state;
}

bool anim_ws_flash_done(void) {
    return s_flash_done;
}

void anim_led_fade_in(uint8_t target_step) {
    s_led_state = LED1WAnim::FADE_IN;
    s_led_target = target_step;
    s_led_start_ms = millis();
    s_led_done = false;
    s_led_reverse = false;

    uint16_t target_duty = step_to_duty_local(target_step);
    for (uint8_t ch = 0; ch < 3; ch++) {
        s_led_from[ch] = 0;
        s_led_to[ch] = target_duty;
        led1w_set_duty(ch + 1, 0);
    }
    s_led_last_ms = millis();
}

void anim_led_fade_out(void) {
    s_led_state = LED1WAnim::FADE_OUT;
    s_led_start_ms = millis();
    s_led_done = false;
    s_led_reverse = true;

    uint16_t current_duty = step_to_duty_local(led1w_get_step());
    for (uint8_t ch = 0; ch < 3; ch++) {
        s_led_from[ch] = current_duty;
        s_led_to[ch] = 0;
    }
    s_led_last_ms = millis();
}

LED1WAnim anim_led_get_state(void) {
    return s_led_state;
}

bool anim_led_done(void) {
    return s_led_done;
}

void animations_update(void) {
    uint32_t now = millis();

    switch (s_ws_state) {
        case WS2812Anim::PORTAL_WAVE: update_ws_portal(now); break;
        case WS2812Anim::RED_FLASH:   update_ws_red_flash(now); break;
        default: break;
    }

    switch (s_led_state) {
        case LED1WAnim::FADE_IN:
        case LED1WAnim::FADE_OUT:
            update_led_fade(now);
            break;
        default: break;
    }
}