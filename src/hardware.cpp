#include "hardware.h"
#include "config.h"
#include <Adafruit_NeoPixel.h>
#include <IWatchdog.h>

extern "C" void SystemClock_Config(void);

// ============================================================
//  Power Management
// ============================================================

void power_init(void) {
    __HAL_RCC_PWR_CLK_ENABLE();
}

void power_enter_sleep(void) {
    IWatchdog.reload();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    SystemClock_Config();
}

void power_init_watchdog(void) {}
void power_kick_watchdog(void) {}

// ============================================================
//  1W LEDs (PWM & Direct DC)
// ============================================================

static uint8_t s_led_step = 0;

static uint8_t step_to_duty(uint8_t step) {
    if (step == 0) return 0;
    if (step >= LED1W_MAX_STEP) return 255;
    float ratio = (float)step / (float)LED1W_MAX_STEP;
    float val = (0.4f * ratio + 0.6f * ratio * ratio) * 255.0f;
    return (uint8_t)(val + 0.5f);
}

void led1w_init(void) {
    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_LED3, OUTPUT);
    led1w_off();
}

void led1w_set_duty(uint8_t ch, uint16_t duty) {
    if (duty > 255) duty = 255;

    uint8_t pin = 0;
    switch (ch) {
        case 1: pin = PIN_LED1; break;
        case 2: pin = PIN_LED2; break;
        case 3: pin = PIN_LED3; break;
        default: return;
    }

    if (duty == 0) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    } else if (duty == 255) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
    } else {
        analogWrite(pin, duty);
    }
}

void led1w_set_step(uint8_t step) {
    if (step > LED1W_MAX_STEP) step = LED1W_MAX_STEP;
    s_led_step = step;

    if (step == 0) {
        led1w_off();
    } else if (step == 10) {
        led1w_set_duty(1, 255);
        led1w_set_duty(2, 255);
        led1w_set_duty(3, 255);
    } else {
        uint8_t duty = step_to_duty(step);
        led1w_set_duty(1, duty);
        led1w_set_duty(2, duty);
        led1w_set_duty(3, duty);
    }
}

uint8_t led1w_get_step(void) {
    return s_led_step;
}

void led1w_off(void) {
    analogWrite(PIN_LED1, 0);
    analogWrite(PIN_LED2, 0);
    analogWrite(PIN_LED3, 0);

    pinMode(PIN_LED1, OUTPUT);
    digitalWrite(PIN_LED1, LOW);
    pinMode(PIN_LED2, OUTPUT);
    digitalWrite(PIN_LED2, LOW);
    pinMode(PIN_LED3, OUTPUT);
    digitalWrite(PIN_LED3, LOW);

    s_led_step = 0;
}

// ============================================================
//  WS2812B NeoPixel
// ============================================================

static Adafruit_NeoPixel s_strip(WS2812_NUM, PIN_WS2812, NEO_GRB + NEO_KHZ800);

void ws2812_init(void) {
    s_strip.begin();
    s_strip.setBrightness(255);
    s_strip.clear();
    s_strip.show();
}

void ws2812_set_pixel(uint8_t idx, uint8_t r, uint8_t g, uint8_t b) {
    if (idx >= WS2812_NUM) return;
    s_strip.setPixelColor(idx, s_strip.Color(r, g, b));
}

void ws2812_fill(uint8_t r, uint8_t g, uint8_t b) {
    s_strip.fill(s_strip.Color(r, g, b), 0, WS2812_NUM);
}

void ws2812_show(void) {
    s_strip.show();
}

void ws2812_off(void) {
    s_strip.clear();
    s_strip.show();
    pinMode(PIN_WS2812, OUTPUT);
    digitalWrite(PIN_WS2812, LOW);
}

// ============================================================
//  Battery ADC
// ============================================================

static uint16_t s_samples[ADC_SAMPLES];
static uint8_t  s_bat_idx = 0;
static bool     s_buf_filled = false;
static uint32_t s_bat_sum = 0;

void battery_init(void) {
    pinMode(PIN_ADC_BAT, INPUT_ANALOG);

    uint16_t init_val = (uint16_t)analogRead(PIN_ADC_BAT);
    for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
        s_samples[i] = init_val;
    }
    s_bat_sum = (uint32_t)init_val * ADC_SAMPLES;
    s_buf_filled = true;
    s_bat_idx = 0;
}

void battery_update(void) {
    uint16_t raw = (uint16_t)analogRead(PIN_ADC_BAT);
    s_bat_sum -= s_samples[s_bat_idx];
    s_samples[s_bat_idx] = raw;
    s_bat_sum += raw;

    s_bat_idx++;
    if (s_bat_idx >= ADC_SAMPLES) {
        s_bat_idx = 0;
        s_buf_filled = true;
    }
}

float battery_voltage(void) {
    uint8_t count = s_buf_filled ? ADC_SAMPLES : (s_bat_idx == 0 ? 1 : s_bat_idx);
    float avg = (float)s_bat_sum / count;
    float v = (avg / 1023.0f) * ADC_VREF / BAT_DIV_K;
    return v * BAT_ADC_CALIB_K;
}

uint8_t battery_percent(void) {
    float v = battery_voltage();
    if (v >= BAT_FULL_V) return 100;
    if (v <= 3.0f) return 0;
    return (uint8_t)(((v - 3.0f) / (BAT_FULL_V - 3.0f)) * 100.0f);
}

bool battery_is_critical(void) {
    float v = battery_voltage();
    return (v > 0.5f) && (v < BAT_CRITICAL_V);
}

bool battery_is_low(void) {
    float v = battery_voltage();
    return (v > 0.5f) && (v < BAT_LOW_V);
}

// ============================================================
//  Rotary Encoder & Button
// ============================================================

static const int8_t ENC_STATES[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
};

static volatile uint8_t  s_prev_state   = 0;
static volatile int8_t   s_subcount     = 0;
static volatile int8_t   s_delta        = 0;

static volatile bool     s_btn_down     = false;
static volatile uint32_t s_btn_press_ms = 0;
static volatile BtnEvent s_btn_event    = BtnEvent::NONE;

static void isr_encoder_pin(void) {
    uint32_t idr = GPIOB->IDR;
    uint8_t curr = ((idr & 0x01) << 1) | ((idr & 0x02) >> 1);
    uint8_t idx  = ((s_prev_state & 0x03) << 2) | (curr & 0x03);

    int8_t move = ENC_STATES[idx];
    if (move != 0) {
        s_subcount += move;
        if (s_subcount >= 4) {
            s_delta++;
            s_subcount = 0;
        } else if (s_subcount <= -4) {
            s_delta--;
            s_subcount = 0;
        }
    }
    s_prev_state = curr;
}

static void isr_btn(void) {
    uint32_t now = millis();
    bool raw = !digitalRead(PIN_ENC_BTN);

    if (raw && !s_btn_down) {
        s_btn_down = true;
        s_btn_press_ms = now;
    } else if (!raw && s_btn_down) {
        s_btn_down = false;
        uint32_t duration = now - s_btn_press_ms;

        if (duration < BTN_DEBOUNCE_MS) {
            return;
        } else if (duration < SHORT_PRESS_MS) {
            s_btn_event = BtnEvent::SHORT;
        } else if (duration < VERY_LONG_PRESS_MS) {
            s_btn_event = BtnEvent::LONG;
        } else {
            s_btn_event = BtnEvent::VERY_LONG;
        }
    }
}

void encoder_init(void) {
    pinMode(PIN_ENC_CLK, INPUT_PULLUP);
    pinMode(PIN_ENC_DT,  INPUT_PULLUP);
    pinMode(PIN_ENC_BTN, INPUT_PULLUP);

    uint32_t idr = GPIOB->IDR;
    uint8_t clk = (idr & 0x01) ? 1 : 0;
    uint8_t dt  = (idr & 0x02) ? 1 : 0;
    s_prev_state = (clk << 1) | dt;

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), isr_encoder_pin, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_DT),  isr_encoder_pin, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_BTN), isr_btn, CHANGE);
}

void encoder_update(void) {}

int8_t encoder_get_delta(void) {
    noInterrupts();
    int8_t d = s_delta;
    s_delta = 0;
    interrupts();
    return -d;
}

BtnEvent encoder_get_event(void) {
    noInterrupts();
    BtnEvent ev = s_btn_event;
    s_btn_event = BtnEvent::NONE;
    interrupts();
    return ev;
}

bool encoder_btn_pressed(void) {
    return s_btn_down;
}

uint32_t encoder_btn_held_ms(void) {
    if (!s_btn_down) return 0;
    return millis() - s_btn_press_ms;
}

void encoder_disable_interrupts(void) {
    detachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK));
    detachInterrupt(digitalPinToInterrupt(PIN_ENC_DT));
}

void encoder_enable_interrupts(void) {
    uint32_t idr = GPIOB->IDR;
    uint8_t clk = (idr & 0x01) ? 1 : 0;
    uint8_t dt  = (idr & 0x02) ? 1 : 0;
    s_prev_state = (clk << 1) | dt;

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), isr_encoder_pin, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_DT),  isr_encoder_pin, CHANGE);
}

// ============================================================
//  TM1637 Display Driver
// ============================================================

static const uint8_t SEG_DIGITS[10] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111  // 9
};

static const uint8_t SEG_C    = 0b00111001;
static const uint8_t SEG_b    = 0b01111100;
static const uint8_t SEG_A    = 0b01110111;
static const uint8_t SEG_t    = 0b01111000;
static const uint8_t SEG_DASH = 0b01000000;
static const uint8_t SEG_DP   = 0b10000000;
static const uint8_t SEG_OFF  = 0b00000000;

static uint8_t s_disp_brightness = TM_BRIGHTNESS;

static inline void tm_delay(void) {
    delayMicroseconds(10);
}

static inline void tm_clk_high(void) { digitalWrite(PIN_TM_CLK, HIGH); }
static inline void tm_clk_low(void)  { digitalWrite(PIN_TM_CLK, LOW);  }
static inline void tm_dio_high(void) { digitalWrite(PIN_TM_DIO, HIGH); }
static inline void tm_dio_low(void)  { digitalWrite(PIN_TM_DIO, LOW);  }

static void tm_start(void) {
    tm_dio_high();
    tm_clk_high();
    tm_delay();
    tm_dio_low();
    tm_delay();
    tm_clk_low();
    tm_delay();
}

static void tm_stop(void) {
    tm_clk_low();
    tm_delay();
    tm_dio_low();
    tm_delay();
    tm_clk_high();
    tm_delay();
    tm_dio_high();
    tm_delay();
}

static bool tm_write_byte(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        tm_clk_low();
        tm_delay();
        if (data & 0x01) {
            tm_dio_high();
        } else {
            tm_dio_low();
        }
        data >>= 1;
        tm_delay();
        tm_clk_high();
        tm_delay();
    }
    tm_clk_low();
    tm_dio_high();
    tm_delay();
    tm_clk_high();
    tm_delay();
    tm_clk_low();
    tm_delay();
    return true;
}

static void tm_send_display_ctrl(bool on, uint8_t bright) {
    tm_start();
    uint8_t cmd = 0x80 | (on ? 0x08 : 0x00) | (bright & 0x07);
    tm_write_byte(cmd);
    tm_stop();
}

static void tm_send_data(const uint8_t segs[4]) {
    tm_start();
    tm_write_byte(0x40);
    tm_stop();

    tm_start();
    tm_write_byte(0xC0);
    for (uint8_t i = 0; i < 4; i++) {
        tm_write_byte(segs[i]);
    }
    tm_stop();
}

void display_init(void) {
    pinMode(PIN_TM_CLK, OUTPUT);
    digitalWrite(PIN_TM_CLK, HIGH);
    pinMode(PIN_TM_DIO, OUTPUT);
    digitalWrite(PIN_TM_DIO, HIGH);

    tm_send_display_ctrl(true, s_disp_brightness);
}

void display_set_brightness(uint8_t brightness) {
    s_disp_brightness = brightness & 0x07;
    tm_send_display_ctrl(true, s_disp_brightness);
}

void display_off(void) {
    tm_send_display_ctrl(false, 0);
    pinMode(PIN_TM_CLK, OUTPUT);
    digitalWrite(PIN_TM_CLK, LOW);
    pinMode(PIN_TM_DIO, OUTPUT);
    digitalWrite(PIN_TM_DIO, LOW);
}

void display_show_counter(uint16_t value) {
    if (value > 999) value = 999;

    uint8_t segs[4];
    segs[0] = SEG_C;
    segs[1] = SEG_DIGITS[value / 100];
    segs[2] = SEG_DIGITS[(value / 10) % 10];
    segs[3] = SEG_DIGITS[value % 10];

    tm_send_data(segs);
    tm_send_display_ctrl(true, s_disp_brightness);
}

void display_show_brightness(uint8_t value) {
    if (value > 10) value = 10;

    uint8_t segs[4];
    segs[0] = SEG_b;

    if (value == 10) {
        segs[1] = SEG_DIGITS[1];
        segs[2] = SEG_DIGITS[0];
    } else {
        segs[1] = SEG_OFF;
        segs[2] = SEG_DIGITS[value];
    }
    segs[3] = SEG_OFF;

    tm_send_data(segs);
    tm_send_display_ctrl(true, s_disp_brightness);
}

void display_show_bat_label(void) {
    uint8_t segs[4] = { SEG_b, SEG_A, SEG_t, SEG_OFF };
    tm_send_data(segs);
    tm_send_display_ctrl(true, s_disp_brightness);
}

void display_show_voltage(float voltage) {
    if (voltage > 9.9f) voltage = 9.9f;
    if (voltage < 0.0f) voltage = 0.0f;

    uint16_t v10 = (uint16_t)(voltage * 10.0f + 0.5f);

    uint8_t segs[4];
    segs[0] = SEG_OFF;
    segs[1] = SEG_DIGITS[v10 / 10] | SEG_DP;
    segs[2] = SEG_DIGITS[v10 % 10];
    segs[3] = SEG_OFF;

    tm_send_data(segs);
    tm_send_display_ctrl(true, s_disp_brightness);
}

void display_show_battery_percentage(uint8_t percent) {
    if (percent > 100) percent = 100;

    uint8_t segs[4];
    segs[0] = 0b01110011; // 'P'

    if (percent == 100) {
        segs[1] = SEG_DIGITS[1];
        segs[2] = SEG_DIGITS[0];
        segs[3] = SEG_DIGITS[0];
    } else if (percent >= 10) {
        segs[1] = SEG_OFF;
        segs[2] = SEG_DIGITS[percent / 10];
        segs[3] = SEG_DIGITS[percent % 10];
    } else {
        segs[1] = SEG_OFF;
        segs[2] = SEG_OFF;
        segs[3] = SEG_DIGITS[percent];
    }

    tm_send_data(segs);
    tm_send_display_ctrl(true, s_disp_brightness);
}

void display_show_hu(void) {
    uint8_t segs[4];
    segs[0] = SEG_DASH;
    segs[1] = 0b01110110; // 'H'
    segs[2] = 0b00111110; // 'U'
    segs[3] = SEG_DASH;

    tm_send_data(segs);
    tm_send_display_ctrl(true, s_disp_brightness);
}
