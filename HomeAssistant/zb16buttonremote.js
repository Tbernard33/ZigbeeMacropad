//Script for HomeAssistant in Zigbee2MQTT folder.
//Put this script in /config/zigbee2mqtt/devices/custom_devices/zb16buttonremote.js
const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const ea = exposes.access;

module.exports = {
    zigbeeModel: ['ZB16ButtonRemote'],
    model: 'ZB16ButtonRemote',
    vendor: 'Custom',
    description: 'Custom ESP32-C6 16-button Zigbee remote (single, double, long)',
    fromZigbee: [fz.on_off, fz.brightness],
    toZigbee: [tz.on_off, tz.light_brightness],
    exposes: (() => {
        const e = [];

        // Actions
        for (let i = 1; i <= 16; i++) {
            e.push(exposes.enum(`button_${i}_action`, ea.STATE, ['single', 'double', 'long'])
                .withDescription(`Action from button ${i}`));
        }

        // LED brightness control
        e.push(exposes.numeric('led_brightness', ea.ALL)
            .withValueMin(0).withValueMax(100).withUnit('%')
            .withDescription('LED brightness level'));

        return e;
    })(),
    meta: { multiEndpoint: false },
};
