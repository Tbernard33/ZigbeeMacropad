/*
 * Zigbee HA_color_dimmable_light Example (ESP32-C6)
 * -------------------------------------------------
 * Features:
 *  - RGB LED blinks red while searching/joining a network.
 *  - Stops blinking once joined; Zigbee commands work normally.
 *  - BOOT button triggers factory reset and restarts pairing.
 */

#include "esp_zb_light.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"

#if !defined CONFIG_ZB_ZCZR
#error Define ZB_ZCZR in idf.py menuconfig to compile light (Router) source code.
#endif

#define TAG                 "ESP_ZB_COLOR_DIMM_LIGHT"
#define BOOT_BUTTON_GPIO     GPIO_NUM_9   // BOOT button (active low on most ESP32-C6 dev boards)

static bool g_is_joined = false;
static bool g_blinking = false;
static bool g_driver_ready = false;
static bool g_blink_on = false;

/* -------------------------------------------------------------------------- */
/* Forward declarations                                                       */
/* -------------------------------------------------------------------------- */
static void zb_blink_step(void);

/* -------------------------------------------------------------------------- */
/* Safe blink scheduler: ensures only one instance is active                  */
/* -------------------------------------------------------------------------- */
static void zb_blink_step(void)
{
    if (!g_driver_ready) {
        esp_zb_scheduler_alarm((esp_zb_callback_t)zb_blink_step, 0, 500);
        return;
    }

    if (!g_blinking) {
        /* Ensure LED off when blinking stops */
        light_driver_set_power(false);
        g_blink_on = false;
        return;
    }

    /* Toggle LED red */
    g_blink_on = !g_blink_on;
    if (g_blink_on) {
        light_driver_set_color_xy(0xFFFF, 0x0000);  // red
        light_driver_set_power(true);
        light_driver_set_level(100);
    } else {
        light_driver_set_level(0);
    }

    /* Re-schedule single blink instance */
    esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)zb_blink_step, 0);
    esp_zb_scheduler_alarm((esp_zb_callback_t)zb_blink_step, 0, 300);
}

/* -------------------------------------------------------------------------- */
/* Factory reset + steering (runs on Zigbee thread)                           */
/* -------------------------------------------------------------------------- */
static void zb_reset_and_steer_cb(void)
{
    ESP_LOGW(TAG, "Factory reset: clearing Zigbee NVS and restarting commissioning...");
    g_is_joined = false;
    g_blinking = true;
    esp_zb_factory_reset();

    /* Cancel any old blinking loops and restart one clean instance */
    esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)zb_blink_step, 0);
    esp_zb_scheduler_alarm((esp_zb_callback_t)zb_blink_step, 0, 0);

    /* Start network steering after short delay */
    esp_zb_scheduler_alarm((esp_zb_callback_t)esp_zb_bdb_start_top_level_commissioning,ESP_ZB_BDB_MODE_NETWORK_STEERING, 1500);
}

/* -------------------------------------------------------------------------- */
/* BOOT button polling task (FreeRTOS context)                                */
/* -------------------------------------------------------------------------- */
static void boot_button_task(void *arg)
{
    gpio_set_direction(BOOT_BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_pullup_en(BOOT_BUTTON_GPIO);
    bool prev = true;

    for (;;) {
        bool pressed = (gpio_get_level(BOOT_BUTTON_GPIO) == 0);
        if (pressed && prev) {
            ESP_LOGI(TAG, "BOOT button pressed → trigger factory reset");
            esp_zb_scheduler_alarm((esp_zb_callback_t)zb_reset_and_steer_cb, 0, 0);
        }
        prev = pressed;
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

/* -------------------------------------------------------------------------- */
/* Deferred light driver initialization                                       */
/* -------------------------------------------------------------------------- */
static esp_err_t deferred_driver_init(void)
{
    static bool inited = false;
    if (!inited) {
        light_driver_init(LIGHT_DEFAULT_OFF);
        g_driver_ready = true;
        ESP_LOGI(TAG, "Light driver initialized");
        /* Start the blinking loop — harmless if not blinking yet */
        esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)zb_blink_step, 0);
        esp_zb_scheduler_alarm((esp_zb_callback_t)zb_blink_step, 0, 0);
        inited = true;
    }
    return inited ? ESP_OK : ESP_FAIL;
}

/* -------------------------------------------------------------------------- */
/* Zigbee signal handler                                                      */
/* -------------------------------------------------------------------------- */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *sig)
{
    uint32_t *p_sg_p = sig->p_app_signal;
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
            g_blinking = is_fn;
            ESP_LOGI(TAG, "Device %s factory new", is_fn ? "is" : "is not");

            if (is_fn) {
                /* Start one blink loop */
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
            g_blinking = false;  // stop blinking
            ESP_LOGI(TAG, "Joined network successfully");
        } else {
            g_is_joined = false;
            g_blinking = true;   // retry blink
            ESP_LOGI(TAG, "Steering failed → retry & blink");
            esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)zb_blink_step, 0);;
            esp_zb_scheduler_alarm((esp_zb_callback_t)zb_blink_step, 0, 0);
            esp_zb_scheduler_alarm((esp_zb_callback_t)
                                   esp_zb_bdb_start_top_level_commissioning,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 2000);
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        g_is_joined = false;
        g_blinking = true;
        ESP_LOGW(TAG, "Left network → rejoining and blinking");
        esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)zb_blink_step, 0);;
        esp_zb_scheduler_alarm((esp_zb_callback_t)zb_blink_step, 0, 0);
        esp_zb_scheduler_alarm((esp_zb_callback_t)
                               esp_zb_bdb_start_top_level_commissioning,
                               ESP_ZB_BDB_MODE_NETWORK_STEERING, 2000);
        break;

    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x) status: %s",
                 esp_zb_zdo_signal_to_string(sig_type),
                 sig_type, esp_err_to_name(err_status));
        break;
    }
}

/* -------------------------------------------------------------------------- */
/* Attribute and action handlers (standard Espressif example)                 */
/* -------------------------------------------------------------------------- */
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool light_state = 0;
    uint8_t light_level = 0;
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

/* -------------------------------------------------------------------------- */
/* Zigbee stack task                                                          */
/* -------------------------------------------------------------------------- */
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
    esp_zb_stack_main_loop();
}

/* -------------------------------------------------------------------------- */
/* Main entry                                                                 */
/* -------------------------------------------------------------------------- */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(esp_zb_task, "ZB_main", 8192, NULL, 5, NULL);
    xTaskCreate(boot_button_task, "boot_btn", 2048, NULL, 5, NULL);
}
