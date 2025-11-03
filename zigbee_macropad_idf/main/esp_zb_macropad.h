/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_Dimmable_Light (Macropad)
 *
 * This code defines a minimal Zigbee endpoint that exposes only
 * On/Off and Brightness control (Level cluster).
 */

#pragma once

#include "esp_zigbee_core.h"
#include "light_driver.h"
#include "zcl_utility.h"

/* --- Zigbee configuration --- */
#define MAX_CHILDREN                      10                                    /* Max number of connected children */
#define INSTALLCODE_POLICY_ENABLE         false                                 /* Disable install code policy for simplicity */
/* Endpoint: keep older name working, but define a single number */
#define HA_DIMMABLE_LIGHT_ENDPOINT  10
#ifndef HA_COLOR_DIMMABLE_LIGHT_ENDPOINT
#define HA_COLOR_DIMMABLE_LIGHT_ENDPOINT  HA_DIMMABLE_LIGHT_ENDPOINT
#endif

#define ESP_ZB_PRIMARY_CHANNEL_MASK       ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK  /* Allow all channels */

/* --- Manufacturer information --- */
#define ESP_MANUFACTURER_NAME  "\x09""STARKYDIY"    /* Custom manufacturer name */
#define ESP_MODEL_IDENTIFIER   "\x08""MACROPAD"     /* Model identifier */

/* --- Zigbee device configuration macros --- */
#define ESP_ZB_ZR_CONFIG()                                      \
    {                                                           \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,               \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,       \
        .nwk_cfg.zczr_cfg = {                                   \
            .max_children = MAX_CHILDREN,                       \
        },                                                      \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }
