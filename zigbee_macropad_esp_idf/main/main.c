#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "led_strip.h"   // via espressif/led_strip component

static const char *TAG = "MACROPAD";

// -------- Pins --------
#define BTN_COUNT 16
static const gpio_num_t BTN_PINS[BTN_COUNT] = {
    GPIO_NUM_1,  GPIO_NUM_2,  GPIO_NUM_3,  GPIO_NUM_4,
    GPIO_NUM_5,  GPIO_NUM_6,  GPIO_NUM_7,  GPIO_NUM_10,
    GPIO_NUM_11, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_18,
    GPIO_NUM_19, GPIO_NUM_20, GPIO_NUM_21, GPIO_NUM_22
};

// WS2812 on GPIO 8, single pixel
#define PIXEL_GPIO   GPIO_NUM_8
#define PIXEL_COUNT  1

// -------- Timing (ms) --------
#define DEBOUNCE_MS      30
#define DOUBLE_CLICK_MS 400
#define LONG_PRESS_MS  1000

// -------- Button state machine --------
typedef struct {
    bool last_raw;               // last sampled raw level
    bool stable;                 // debounced state
    uint64_t last_change_us;     // debounce timestamp
    uint64_t press_start_us;     // when stable press started
    uint64_t last_release_us;    // for double-click detection
    bool long_fired;             // true if long press event fired
} btn_state_t;

static btn_state_t g_btn[BTN_COUNT];

// -------- LED driver --------
static led_strip_handle_t s_led;

// Timing helpers
static inline uint64_t now_us(void) { return esp_timer_get_time(); }
static inline uint32_t us_to_ms(uint64_t us) { return (uint32_t)(us / 1000ULL); }

// LED helpers
static void pixel_set(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness /*0..100*/) {
    uint16_t scale = (uint16_t)brightness * 255 / 100;
    uint8_t sr = (uint8_t)((r * scale) / 255);
    uint8_t sg = (uint8_t)((g * scale) / 255);
    uint8_t sb = (uint8_t)((b * scale) / 255);
    led_strip_set_pixel(s_led, 0, sr, sg, sb);
    led_strip_refresh(s_led);
}
static void pixel_off(void) { pixel_set(0,0,0,100); }

typedef enum { ACT_SINGLE, ACT_DOUBLE, ACT_LONG } action_t;
static const char* action_str(action_t a) {
    return (a==ACT_SINGLE) ? "single" : (a==ACT_DOUBLE) ? "double" : "long";
}

// Visual flash per action
static void flash_action(action_t a, uint8_t brightness) {
    uint8_t r=0,g=0,b=0;
    if (a==ACT_SINGLE)      { g=255; }        // green
    else if (a==ACT_DOUBLE) { b=255; }        // blue
    else                    { r=255; g=255; } // yellow
    pixel_set(r,g,b,brightness);
    vTaskDelay(pdMS_TO_TICKS(120));
    pixel_off();
}

// Application callback
static void on_button_action(uint8_t index, action_t act) {
    ESP_LOGI(TAG, "Button %u -> %s", (unsigned)index+1, action_str(act));
    flash_action(act, 80);
}

// Polling scanner task
static void button_task(void *arg) {
    const uint32_t poll_ms = 2;
    while (true) {
        uint64_t t_us = now_us();
        for (int i=0; i<BTN_COUNT; ++i) {
            int raw_level = gpio_get_level(BTN_PINS[i]);  // 0=pressed, 1=released
            bool raw_pressed = (raw_level == 0);

            if (raw_pressed != g_btn[i].last_raw) {
                g_btn[i].last_raw = raw_pressed;
                g_btn[i].last_change_us = t_us;
            }

            // Debounce logic
            if (us_to_ms(t_us - g_btn[i].last_change_us) >= DEBOUNCE_MS) {
                if (raw_pressed != g_btn[i].stable) {
                    g_btn[i].stable = raw_pressed;
                    if (g_btn[i].stable) {
                        g_btn[i].press_start_us = t_us;
                        g_btn[i].long_fired = false;
                    } else {
                        uint32_t held_ms = us_to_ms(t_us - g_btn[i].press_start_us);
                        if (!g_btn[i].long_fired) {
                            if (g_btn[i].last_release_us &&
                                us_to_ms(t_us - g_btn[i].last_release_us) <= DOUBLE_CLICK_MS) {
                                on_button_action(i, ACT_DOUBLE);
                                g_btn[i].last_release_us = 0;
                            } else {
                                g_btn[i].last_release_us = t_us;
                            }
                        } else {
                            g_btn[i].last_release_us = 0;
                        }
                    }
                }
            }

            // Long press
            if (g_btn[i].stable && !g_btn[i].long_fired) {
                if (us_to_ms(t_us - g_btn[i].press_start_us) >= LONG_PRESS_MS) {
                    g_btn[i].long_fired = true;
                    on_button_action(i, ACT_LONG);
                    g_btn[i].last_release_us = 0;
                }
            }

            // Single-click timeout
            if (g_btn[i].last_release_us &&
                us_to_ms(t_us - g_btn[i].last_release_us) > DOUBLE_CLICK_MS) {
                on_button_action(i, ACT_SINGLE);
                g_btn[i].last_release_us = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(poll_ms));
    }
}

// LED initialization for IDF ≥ 5.2
static void init_led_strip(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = PIXEL_GPIO,
        .max_leds = PIXEL_COUNT,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10 MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led));
    ESP_ERROR_CHECK(led_strip_clear(s_led));
    pixel_off();
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting 16-button macropad scanner (ESP-IDF)");

    // Configure GPIOs as inputs with pull-ups
    gpio_config_t io = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    for (int i=0;i<BTN_COUNT;++i) {
        io.pin_bit_mask |= 1ULL << BTN_PINS[i];
    }
    ESP_ERROR_CHECK(gpio_config(&io));

    // Initialize button states
    memset(g_btn, 0, sizeof(g_btn));
    uint64_t t0 = now_us();
    for (int i=0;i<BTN_COUNT;++i) {
        g_btn[i].last_raw = (gpio_get_level(BTN_PINS[i]) == 0);
        g_btn[i].stable = false;
        g_btn[i].last_change_us = t0;
    }

    // Initialize LED strip
    init_led_strip();

    // Launch button polling task
    xTaskCreatePinnedToCore(button_task, "button_task", 4096, NULL, 5, NULL, tskNO_AFFINITY);

    ESP_LOGI(TAG, "Ready. Press any of the 16 buttons (active-LOW).");
}