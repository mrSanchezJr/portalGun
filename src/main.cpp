#include <Arduino.h>
#include "config.h"
#include "hardware.h"
#include "animations.h"
#include "eeprom_emu.h"

enum class AppState : uint8_t {
    SLEEP,
    WAKEUP,
    ACTIVE,
    MEASURE,
    BRIGHTNESS,
    BATTERY_BAT,
    BATTERY_VOLT,
    GOING_SLEEP
};

static AppState g_state = AppState::SLEEP;
static uint16_t g_counter = 137;
static uint8_t  g_brightness = 5;

static uint32_t g_state_enter_ms = 0;
static uint32_t g_last_activity_ms = 0;
static uint32_t g_last_bat_flash_ms = 0;
static uint32_t g_last_bat_update_ms = 0;
static uint32_t g_last_wdg_kick_ms = 0;

static bool g_hold_b_handled = false;
static bool g_hold_bat_handled = false;
static bool g_hu_active = false;

static void enter_state(AppState new_state) {
    g_state = new_state;
    g_state_enter_ms = millis();
}

static void load_from_eeprom(void) {
    uint16_t val;
    if (eeprom_read(EE_ADDR_COUNTER, val))    g_counter = val;
    if (eeprom_read(EE_ADDR_BRIGHTNESS, val)) g_brightness = (uint8_t)val;

    if (g_counter > 999) g_counter = 137;
    if (g_brightness > LED1W_MAX_STEP) g_brightness = 5;
}

static void do_wakeup(void) {
    display_init();
    led1w_init();
    ws2812_init();

    anim_ws_start_portal();
    anim_led_fade_in(g_brightness);
    display_show_counter(g_counter);

    enter_state(AppState::ACTIVE);
}

static void do_sleep(void) {
    anim_led_fade_out();
    enter_state(AppState::GOING_SLEEP);
}

static void finalize_sleep(void) {
    encoder_disable_interrupts();

    display_off();
    ws2812_off();
    led1w_off();

    power_enter_sleep();

    while (encoder_btn_pressed()) {
        delay(10);
        power_kick_watchdog();
    }
    encoder_get_event();
    encoder_enable_interrupts();

    enter_state(AppState::WAKEUP);
}

static void handle_measure(BtnEvent btn, int8_t delta) {
    if (g_hu_active) return;

    uint32_t now = millis();
    uint32_t held = encoder_btn_held_ms();

    if (encoder_btn_pressed()) {
        if (held >= 2000 && !g_hold_bat_handled) {
            g_hold_bat_handled = true;
            g_hold_b_handled = true;
            display_show_bat_label();
            enter_state(AppState::BATTERY_BAT);
            return;
        } else if (held >= 1000 && !g_hold_b_handled) {
            g_hold_b_handled = true;
            g_last_activity_ms = now;
            display_show_brightness(g_brightness);
            enter_state(AppState::BRIGHTNESS);
            return;
        }
    } else {

        g_hold_b_handled = false;
        g_hold_bat_handled = false;
    }

    if (btn == BtnEvent::SHORT) {

        do_sleep();
        return;
    }

    if (delta != 0) {
        int16_t new_val = (int16_t)g_counter + delta;
        if (new_val < 0) new_val = 0;
        if (new_val > 999) new_val = 999;
        g_counter = (uint16_t)new_val;

        display_show_counter(g_counter);
        eeprom_write(EE_ADDR_COUNTER, g_counter);
    }

    if (battery_is_low()) {
        if (now - g_last_bat_flash_ms >= LOW_BAT_FLASH_PERIOD_MS) {
            g_last_bat_flash_ms = now;
            anim_ws_start_red_flash();
        }
    }
}

static void handle_brightness(BtnEvent btn, int8_t delta) {
    uint32_t now = millis();
    uint32_t held = encoder_btn_held_ms();

    if (encoder_btn_pressed() && held >= 2000 && !g_hold_bat_handled) {
        g_hold_bat_handled = true;
        
        display_show_bat_label();
        enter_state(AppState::BATTERY_BAT);
        return;
    }

    if (!encoder_btn_pressed()) {
        g_hold_b_handled = false;
        g_hold_bat_handled = false;
    }

    if (btn == BtnEvent::SHORT || (now - g_last_activity_ms >= BRIGHTNESS_TIMEOUT_MS && delta == 0)) {
        
        eeprom_write(EE_ADDR_BRIGHTNESS, g_brightness);
        
        display_show_counter(g_counter);
        enter_state(AppState::MEASURE);
        return;
    }

    if (delta != 0) {
        g_last_activity_ms = now;

        int8_t new_step = (int8_t)g_brightness + delta;
        if (new_step < 0) new_step = 0;
        if (new_step > LED1W_MAX_STEP) new_step = LED1W_MAX_STEP;
        g_brightness = (uint8_t)new_step;

        led1w_set_step(g_brightness);
        display_show_brightness(g_brightness);
    }
}

static void handle_battery_bat(BtnEvent btn) {
    uint32_t now = millis();

    if (now - g_state_enter_ms >= BAT_SHOW_MS) {
        
        uint8_t pct = battery_percent();
        display_show_battery_percentage(pct);
        enter_state(AppState::BATTERY_VOLT);
    }

    (void)btn;
}

static void handle_battery_volt(BtnEvent btn) {
    uint32_t now = millis();

    bool timeout = (now - g_state_enter_ms >= BAT_VOLTAGE_SHOW_MS);
    bool pressed = (btn == BtnEvent::SHORT);

    if (timeout || pressed) {
        display_show_counter(g_counter);
        enter_state(AppState::MEASURE);
    }
}

static void handle_going_sleep(void) {
    if (anim_led_done()) {
        finalize_sleep();
    }
}

void setup(void) {
    eeprom_init();
    load_from_eeprom();

    power_init();
    encoder_init();
    display_init();
    led1w_init();
    ws2812_init();
    battery_init();
    animations_init();

    pinMode(PIN_HU_LOAD, OUTPUT);
    digitalWrite(PIN_HU_LOAD, LOW);

    power_init_watchdog();

    g_state = AppState::SLEEP;
    finalize_sleep();
}

void loop(void) {
    uint32_t now = millis();

    if (now - g_last_wdg_kick_ms >= 500) {
        g_last_wdg_kick_ms = now;
        power_kick_watchdog();
    }

    if (now - g_last_bat_update_ms >= 100) {
        g_last_bat_update_ms = now;
        battery_update();
    }

    static uint32_t s_hu_btn_time = 0;
    if (g_state == AppState::MEASURE && g_counter == HU_COUNTER) {
        bool physical_pressed = encoder_btn_pressed();
        if (physical_pressed) {
            if (!g_hu_active) {
                if (s_hu_btn_time == 0) {
                    s_hu_btn_time = now;
                } else if (now - s_hu_btn_time >= 25) {
                    g_hu_active = true;
                    digitalWrite(PIN_HU_LOAD, HIGH);
                    display_show_hu();
                }
            }
        } else {
            s_hu_btn_time = 0;
            if (g_hu_active) {
                g_hu_active = false;
                digitalWrite(PIN_HU_LOAD, LOW);
                display_show_counter(g_counter);
                encoder_get_event();
            }
        }
    }

    BtnEvent btn   = encoder_get_event();
    int8_t   delta = encoder_get_delta();

    if (g_hu_active) {
        btn = BtnEvent::NONE;
        delta = 0;
    }

    animations_update();

    switch (g_state) {
        case AppState::SLEEP:
            break;

        case AppState::WAKEUP:
            do_wakeup();
            break;

        case AppState::ACTIVE:
            if (anim_led_done()) {
                display_show_counter(g_counter);
                enter_state(AppState::MEASURE);
            }
            if (btn == BtnEvent::SHORT) {
                do_sleep();
            }
            break;

        case AppState::MEASURE:
            handle_measure(btn, delta);
            break;

        case AppState::BRIGHTNESS:
            handle_brightness(btn, delta);
            break;

        case AppState::BATTERY_BAT:
            handle_battery_bat(btn);
            break;

        case AppState::BATTERY_VOLT:
            handle_battery_volt(btn);
            break;

        case AppState::GOING_SLEEP:
            handle_going_sleep();
            break;
    }
}
