// Script for HomeAssistant in Zigbee2MQTT folder.
// Put this script in /config/zigbee2mqtt/devices/custom_devices/zb16buttonremote.js
// Custom converter for a 16-button ESP32 remote.
// Maps incoming brightness/level to button_{n}_{single|double|long}

const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const ea = exposes.access;

function parseLevelToAction(level) {
    // level expected 1..100 according to mapping:
    // level = (button_index - 1) * 6 + action_code
    // action_code: 1=single, 2=double, 3=long
    if (typeof level !== 'number' || level < 1) return null;
    const idx0 = Math.floor((level - 1) / 6);   // 0-based button index
    const actionCode = ((level - 1) % 6) + 1;   // 1..6 but valid codes are 1..3
    if (idx0 < 0 || idx0 >= 16) return null;
    if (actionCode < 1 || actionCode > 3) return null;
    const button = idx0 + 1;
    const actionStr = actionCode === 1 ? 'single' : (actionCode === 2 ? 'double' : 'long');
    return `button_${button}_${actionStr}`;
}

const fromZigbeeButtonMapper = {
    cluster: 'genOnOff', // we expect ON reports with brightness level
    type: ['attributeReport','readResponse','commandOn','commandOff','commandToggle'], // be tolerant
    convert: (model, msg, publish, options, meta) => {
        // try to find brightness/level in the message
        // different coordinators / wrappers use different keys; check a few common ones
        let level = null;

        // common: msg.data has 'brightness' or 'level' or 'value'
        if (msg.data && typeof msg.data === 'object') {
            if (msg.data.hasOwnProperty('brightness')) level = Number(msg.data.brightness);
            else if (msg.data.hasOwnProperty('level')) level = Number(msg.data.level);
            else if (msg.data.hasOwnProperty('value')) level = Number(msg.data.value);
            else if (msg.data.hasOwnProperty('payload') && typeof msg.data.payload === 'object') {
                // rare: nested payload
                const p = msg.data.payload;
                if (p.hasOwnProperty('brightness')) level = Number(p.brightness);
            }
        }

        // fallback: some implementations send brightness as msg.data[0]?.value
        if (level === null && Array.isArray(msg.data)) {
            for (const k of msg.data) {
                if (k && typeof k === 'object' && k.hasOwnProperty('value')) {
                    level = Number(k.value);
                    break;
                }
            }
        }

        // If still null, try msg.data.brightnessPercentage (some stacks)
        if (level === null && msg.data && msg.data.hasOwnProperty('brightnessPercentage')) {
            level = Number(msg.data.brightnessPercentage);
        }

        if (level === null) {
            // nothing to do
            return null;
        }

        const action = parseLevelToAction(level);
        if (!action) return null;

        // publish a single field "action" with the produced action string
        const result = { action: action };
        // publish() can be used in newer converters; return result works too
        return result;
    }
};

module.exports = {
    zigbeeModel: ['ZB16BtnRemote', 'ZB16ButtonRemote', 'CustomZB16'], // match your device model string
    model: 'ZB16ButtonRemote',
    vendor: 'Custom',
    description: 'ESP32-C6 16-button remote (single/double/long) - custom converter',
    fromZigbee: [ fromZigbeeButtonMapper ],
    toZigbee: [],
    exposes: [
        // one text state that homeassistant can use: values are e.g. "button_3_double"
        exposes.text('action', ea.STATE).withDescription('Last button action, format button_N_{single|double|long}'),
        // optional LED brightness number control that you might expose (if you add a toZigbee handler)
        exposes.numeric('led_brightness', ea.ALL).withValueMin(0).withValueMax(100).withDescription('LED brightness (0-100)'),
    ],
    meta: { multiEndpoint: false },
};
