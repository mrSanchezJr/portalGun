#pragma once
#include <Arduino.h>

// Pins
#define PIN_ENC_CLK     PB0
#define PIN_ENC_DT      PB1
#define PIN_ENC_BTN     PB7

#define PIN_TM_CLK      PB6
#define PIN_TM_DIO      PB5

#define PIN_WS2812      PA6

#define PIN_LED1        PA8
#define PIN_LED2        PA9
#define PIN_LED3        PA10

#define PIN_ADC_BAT     PA1

#define PIN_HU_LOAD     PB13
#define HU_COUNTER      111

// Battery
#define BAT_CRITICAL_V      3.00f
#define BAT_LOW_V           3.50f
#define BAT_FULL_V          4.20f
#define BAT_DIV_K           0.5f
#define BAT_ADC_CALIB_K     2.333f
#define ADC_VREF            3.3f
#define ADC_BITS            10
#define ADC_MAX_VAL         1023
#define ADC_SAMPLES         10

// Encoder & Button
#define BTN_DEBOUNCE_MS     50
#define SHORT_PRESS_MS      1000
#define LONG_PRESS_MIN_MS   1000
#define LONG_PRESS_MAX_MS   3000
#define VERY_LONG_PRESS_MS  3000

// Display
#define TM_BRIGHTNESS       5

// LEDs
#define WS2812_NUM          5
#define LED1W_MAX_STEP      10
#define FADE_DURATION_MS    1000
#define LED1_FADE_DELAY_MS  0
#define LED2_FADE_DELAY_MS  150
#define LED3_FADE_DELAY_MS  300

// Timeouts
#define BRIGHTNESS_TIMEOUT_MS   3000
#define BAT_SHOW_MS             1000
#define BAT_VOLTAGE_SHOW_MS     3000
#define LOW_BAT_FLASH_PERIOD_MS 60000
#define LOW_BAT_FLASH_MS        200

// Flash EEPROM Emulation
#define EEPROM_PAGE0_BASE   0x0800F800
#define EEPROM_PAGE1_BASE   0x0800FC00
#define EEPROM_PAGE_SIZE    0x400

#define EE_ADDR_COUNTER     0x0001
#define EE_ADDR_BRIGHTNESS  0x0002