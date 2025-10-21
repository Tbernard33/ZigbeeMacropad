// Script for HomeAssistant in Zigbee2MQTT folder.
// Put this script in /config/zigbee2mqtt/devices/custom_devices/zb16buttonremote.js
// Custom converter for ESP32-C6 16-button remote
// Recognizes level-based events from Arduino firmware

const exposes = require('zigbee-herdsman-converters/lib/exposes');
const ea = exposes.access;

function parseLevelToAction(level) {
    if (typeof level !== 'number' || level < 1) return null;
    const idx = Math.floor(level / 6);       // 0-based button index
    const code = level % 6;                  // action code
    const button = idx + 1;
    let action = 'unknown';
    if (code === 1) action = 'single';
    else if (code === 2) action = 'double';
    else if (code === 3) action = 'long';
    else return null;
    return `button_${button}_${action}`;
}

const fromZigbee = [{
    cluster: 'genOnOff',
    type: ['attributeReport', 'readResponse', 'commandOn', 'commandOff', 'commandToggle'],
    convert: (model, msg, publish, options, meta) => {
        const level = msg.data?.brightness ?? msg.data?.level ?? msg.data?.value;
        if (level == null) return null;
        const action = parseLevelToAction(Number(level));
        if (!action) return null;
        return { action };
    },
}];

module.exports = {
    zigbeeModel: ['ZB16BtnRemote'],   // must match your Arduino sketch
    model: 'ZB16ButtonRemote',
    vendor: 'Custom',
    description: 'ESP32-C6 16-button Zigbee remote with brightness',
    fromZigbee,
    toZigbee: [],
    exposes: [
        exposes.text('action', ea.STATE).withDescription('Last button action'),
        exposes.numeric('brightness', ea.ALL)
            .withValueMin(0).withValueMax(100)
            .withUnit('%')
            .withDescription('LED brightness control'),
    ],
    meta: { multiEndpoint: false },
};
