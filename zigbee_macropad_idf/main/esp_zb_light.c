/*
 * Zigbee HA_color_dimmable_light Example (ESP32-C6)
 * -------------------------------------------------
 * Features:
 *  - RGB LED blinks RED while searching/joining a network.
 *  - Stops blinking once joined; Zigbee commands work normally.
 *  - BOOT button triggers factory reset and restarts pairing.
 *  - Button scanner task is low priority and non-blocking (no WDT trips).
 */

#include <stdio.h>
#include <string.h>
#include "esp_zb_light.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_timer.h"

#if !defined CONFIG_ZB_ZCZR
#error Define ZB_ZCZR in idf.py menuconfig to compile light (Router) source code.
#endif

#define TAG                 "MACROPAD"

/* --- PINS --------------------------------------------------------------- */
#define BTN_COUNT 16
static const gpio_num_t BTN_PINS[BTN_COUNT] = {
    GPIO_NUM_1,  GPIO_NUM_2,  GPIO_NUM_3,  GPIO_NUM_4,
    GPIO_NUM_5,  GPIO_NUM_6,  GPIO_NUM_7,  GPIO_NUM_10,
    GPIO_NUM_11, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_18,
    GPIO_NUM_19, GPIO_NUM_20, GPIO_NUM_21, GPIO_NUM_22
};
#define BOOT_BUTTON_GPIO     GPIO_NUM_9

/* --- Timing (ms) for local keypad -------------------------------------- */
#define BTN_POLL_INTERVAL_MS   10
#define DEBOUNCE_MS        30
#define DOUBLE_CLICK_MS   400
#define LONG_PRESS_MS    1000

/* --- Button state machine ----------------------------------------------- */
typedef struct {
    bool raw;
    bool stable;
    bool prev_stable;
    uint64_t last_change_us;
    uint64_t press_start_us;
    uint64_t last_release_us;
    bool long_fired;
} btn_state_t;

typedef struct {
    uint8_t idx;       // which button (0–15)
    uint64_t timestamp_us;
    bool level;        // 0 = pressed, 1 = released
} btn_evt_t;

static QueueHandle_t s_boot_evt_q = NULL;

static btn_state_t g_btn[BTN_COUNT];

/* --- Zigbee + LED state ------------------------------------------------- */
static bool g_is_joined     = false;
static bool g_blinking      = false;
static bool g_driver_ready  = false;
static bool g_blink_on      = false;
static bool g_zb_ready = false;

/* --- Helpers ------------------------------------------------------------ */
static inline uint64_t now_us(void) { return esp_timer_get_time(); }
static inline uint32_t us_to_ms(uint64_t us) { return (uint32_t)(us / 1000ULL); }

typedef enum { ACT_NONE, ACT_SINGLE, ACT_DOUBLE, ACT_LONG } action_t;

/* --- Forward declarations  ------------------------------------------------------ */
static const char* action_str(action_t a);
static void flash_action(action_t a, uint8_t brightness);
static void start_network_steering(uint8_t param);
static void zb_blink_step(void);
static void zb_reset_and_steer_cb(void);

/* ======================================================================= */
/*                          BLINKER (ZB CONTEXT)                           */
/* ======================================================================= */
static void zb_blink_step(void)
{
    if (!g_driver_ready) {
        esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)zb_blink_step, 0);
        esp_zb_scheduler_alarm((esp_zb_callback_t)zb_blink_step, 0, 500);
        return;
    }

    if (!g_blinking) {
        /* IMPORTANT: do NOT touch power/level when not blinking.
           This prevents overriding user UI commands. */
        g_blink_on = false;
        esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)zb_blink_step, 0);
        return;
    }

    g_blink_on = !g_blink_on;
    if (g_blink_on) {
        light_driver_set_color_xy(0xA3D6, 0x547B);  // red
        //green : Color X: 0x4ccd  Color Y: 0x9999
        //blue : Color X: 0x2666  Color Y: 0x0f5c
        //yellow : Color X: 0x6b58  Color Y: 0x8157
        light_driver_set_power(true);
        light_driver_set_level(100);   // adjust if you want dimmer pairing
    } else {
        light_driver_set_level(0);
    }

    /* Reschedule single instance */
    esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)zb_blink_step, 0);
    esp_zb_scheduler_alarm((esp_zb_callback_t)zb_blink_step, 0, 300);
}

/* ======================================================================= */
/*                   FACTORY RESET + COMMISSION (ZB CTX)                   */
/* ======================================================================= */
static void start_network_steering(uint8_t param)
{
    (void)param;
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
}

static void zb_reset_and_steer_cb(void)
{
    ESP_LOGW(TAG, "Factory reset: clearing Zigbee NVS and restarting commissioning...");
    g_is_joined = false;
    g_blinking  = true;

    esp_zb_factory_reset();

    /* Start fresh blink loop (single instance) */
    esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)zb_blink_step, 0);
    esp_zb_scheduler_alarm((esp_zb_callback_t)zb_blink_step, 0, 0);

    /* Start network steering shortly after */
    esp_zb_scheduler_alarm(start_network_steering, 0, 1500);
}

/* ======================================================================= */
/*                      BOOT BUTTON (ISR + QUEUE)                          */
/* ======================================================================= */
static void IRAM_ATTR boot_button_isr(void *arg) {
    uint32_t dummy = 1;
    xQueueSendFromISR(s_boot_evt_q, &dummy, NULL);
}

static void boot_button_task(void *arg) {
    uint32_t dummy;
    while (1) {
        if (xQueueReceive(s_boot_evt_q, &dummy, portMAX_DELAY)) {
            if (g_zb_ready) {
                esp_zb_scheduler_alarm((esp_zb_callback_t)zb_reset_and_steer_cb, 0, 0);
            } else {
                ESP_LOGW(TAG, "Zigbee not ready; ignoring button press");
            }
        }
    }
}

static const char* action_str(action_t a) {
    return (a==ACT_SINGLE) ? "single" : (a==ACT_DOUBLE) ? "double" : "long";
}

static void flash_action(action_t a, uint8_t brightness) {
    switch(a){
        case ACT_SINGLE: light_driver_set_color_xy(0x4ccd, 0x9999);break;  // green
        case ACT_DOUBLE: light_driver_set_color_xy(0x2666, 0x0f5c);break;  // blue
        case ACT_LONG: light_driver_set_color_xy(0x6b58, 0x8157);break;  // yellow
        default: break;
    }
    light_driver_set_power(true);
    light_driver_set_level(brightness);   // adjust if you want dimmer pairing
    vTaskDelay(pdMS_TO_TICKS(150));
    light_driver_set_power(false);
}

static void on_button_action(uint8_t index, action_t act) {
    ESP_LOGI(TAG, "Button %u -> %s", (unsigned)index+1, action_str(act));
    flash_action(act, 80);
}

static void button_task(void *arg)
{
    const uint64_t poll_interval_us = BTN_POLL_INTERVAL_MS * 1000ULL;
    const uint64_t debounce_us      = DEBOUNCE_MS * 1000ULL;
    const uint64_t double_click_us  = DOUBLE_CLICK_MS * 1000ULL;
    const uint64_t long_press_us    = LONG_PRESS_MS * 1000ULL;

    while (true) {
        uint64_t now = esp_timer_get_time();

        for (int i = 0; i < BTN_COUNT; ++i) {
            bool raw = (gpio_get_level(BTN_PINS[i]) == 0);  // active low
            btn_state_t *b = &g_btn[i];

            // Debounce transition
            if (raw != b->stable && now - b->last_change_us > debounce_us) {
                b->prev_stable = b->stable;
                b->stable = raw;
                b->last_change_us = now;

                if (b->stable) {
                    // pressed
                    b->press_start_us = now;
                    b->long_fired = false;
                } else {
                    // released
                    if (!b->long_fired) {
                        if (b->last_release_us &&
                            (now - b->last_release_us < double_click_us)) {
                            on_button_action(i, ACT_DOUBLE);
                            b->last_release_us = 0;
                        } else {
                            b->last_release_us = now;
                        }
                    }
                }
            }

            // long press detection
            if (b->stable && !b->long_fired &&
                now - b->press_start_us > long_press_us) {
                b->long_fired = true;
                on_button_action(i, ACT_LONG);
                b->last_release_us = 0;
            }

            // single click confirmation (timeout expired)
            if (!b->stable && b->last_release_us &&
                now - b->last_release_us > double_click_us) {
                on_button_action(i, ACT_SINGLE);
                b->last_release_us = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(BTN_POLL_INTERVAL_MS));
    }
}

/* ======================================================================= */
/*                  DEFERRED LIGHT DRIVER INITIALIZATION                   */
/* ======================================================================= */
static esp_err_t deferred_driver_init(void)
{
    static bool inited = false;
    if (!inited) {
        light_driver_init(LIGHT_DEFAULT_OFF);
        g_driver_ready = true;
        ESP_LOGI(TAG, "Light driver initialized");
        inited = true;
    }
    return inited ? ESP_OK : ESP_FAIL;
}

/* ======================================================================= */
/*                      ZIGBEE SIGNAL HANDLER                              */
/* ======================================================================= */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *sig)
{
    uint32_t *p_sg_p    = sig->p_app_signal;
    esp_err_t err_status = sig->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            deferred_driver_init();
            bool is_fn = esp_zb_bdb_is_factory_new();
            g_is_joined = !is_fn;
            g_blinking  = is_fn;
            ESP_LOGI(TAG, "Device %s factory new", is_fn ? "is" : "is not");

            if (is_fn) {
                esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)zb_blink_step, 0);
                esp_zb_scheduler_alarm((esp_zb_callback_t)zb_blink_step, 0, 0);
                ESP_LOGI(TAG, "Starting network steering...");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            g_is_joined = true;
            g_blinking  = false;  // stop blinking
            esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)zb_blink_step, 0);
            ESP_LOGI(TAG, "Joined network successfully");
        } else {
            g_is_joined = false;
            g_blinking  = true;   // retry blink
            ESP_LOGI(TAG, "Steering failed → retry & blink");
            esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)zb_blink_step, 0);
            esp_zb_scheduler_alarm((esp_zb_callback_t)zb_blink_step, 0, 0);
            esp_zb_scheduler_alarm((esp_zb_callback_t)
                                   esp_zb_bdb_start_top_level_commissioning,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 2000);
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        g_is_joined = false;
        g_blinking  = true;
        ESP_LOGW(TAG, "Left network → rejoining and blinking");
        esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)zb_blink_step, 0);
        esp_zb_scheduler_alarm((esp_zb_callback_t)zb_blink_step, 0, 0);
        esp_zb_scheduler_alarm(start_network_steering, 0, 1500);
        break;

    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x) status: %s",
                 esp_zb_zdo_signal_to_string(sig_type),
                 sig_type, esp_err_to_name(err_status));
        break;
    }
}

/* ======================================================================= */
/*                ZCL attribute & action handlers (standard)                */
/* ======================================================================= */
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool     light_state   = 0;
    uint8_t  light_level   = 0;
    uint16_t light_color_x = 0;
    uint16_t light_color_y = 0;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS,
                        ESP_ERR_INVALID_ARG, TAG, "Bad ZCL status");

    if (message->info.dst_endpoint == HA_COLOR_DIMMABLE_LIGHT_ENDPOINT) {
        switch (message->info.cluster) {
        case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
                light_state = *(bool *)message->attribute.data.value;
                light_driver_set_power(light_state);
                ESP_LOGI(TAG, "Light %s", light_state ? "ON" : "OFF");
            }
            break;
        case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID) {
                light_color_x = *(uint16_t *)message->attribute.data.value;
                light_color_y = *(uint16_t *)esp_zb_zcl_get_attribute(
                    message->info.dst_endpoint, message->info.cluster,
                    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID)->data_p;
            } else if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID) {
                light_color_y = *(uint16_t *)message->attribute.data.value;
                light_color_x = *(uint16_t *)esp_zb_zcl_get_attribute(
                    message->info.dst_endpoint, message->info.cluster,
                    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID)->data_p;
            }
            light_driver_set_color_xy(light_color_x, light_color_y);
            break;
        case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID) {
                light_level = *(uint8_t *)message->attribute.data.value;
                light_driver_set_level(light_level);
            }
            break;
        default:
            ESP_LOGI(TAG, "Unhandled cluster 0x%x", message->info.cluster);
        }
    }
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    if (callback_id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID)
        return zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
    return ESP_OK;
}

/* ======================================================================= */
/*                       ZIGBEE STACK TASK                                 */
/* ======================================================================= */
static void esp_zb_task(void *pv)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_color_dimmable_light_cfg_t light_cfg = ESP_ZB_DEFAULT_COLOR_DIMMABLE_LIGHT_CONFIG();
    esp_zb_ep_list_t *ep =
        esp_zb_color_dimmable_light_ep_create(HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, &light_cfg);

    zcl_basic_manufacturer_info_t info = {
        .manufacturer_name = ESP_MANUFACTURER_NAME,
        .model_identifier  = ESP_MODEL_IDENTIFIER,
    };
    esp_zcl_utility_add_ep_basic_manufacturer_info(ep, HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, &info);
    esp_zb_device_register(ep);
    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    g_zb_ready = true;
    esp_zb_stack_main_loop();
}

/* ======================================================================= */
/*                                MAIN                                     */
/* ======================================================================= */
void app_main(void)
{
    ESP_LOGI(TAG, "Starting 16-button macropad + Zigbee color light");

    // --- Configure 16 button inputs ---
    gpio_config_t io = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE  // trigger on both press/release
    };
    for (int i = 0; i < BTN_COUNT; ++i) {
        io.pin_bit_mask |= 1ULL << BTN_PINS[i];
    }
    ESP_ERROR_CHECK(gpio_config(&io));

    // --- Configure BOOT button input ---
    gpio_config_t btnio = {
        .pin_bit_mask = 1ULL << BOOT_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&btnio));

    // --- Create queue ---
    s_boot_evt_q = xQueueCreate(4, sizeof(uint32_t));

    // --- Install ISR service ONCE ---
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    gpio_isr_handler_add(BOOT_BUTTON_GPIO, boot_button_isr, NULL);

    // --- Launch tasks ---
    xTaskCreate(button_task, "button_task", 4096, NULL, 1, NULL);
    xTaskCreate(boot_button_task, "boot_btn", 2048, NULL, 1, NULL);

    /* --- ZIGBEE --------------------------------------------------------- */
    
    ESP_ERROR_CHECK(nvs_flash_init());

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(esp_zb_task, "ZB_main", 8192, NULL, 5, NULL);

    ESP_LOGI(TAG, "Ready.");
}
