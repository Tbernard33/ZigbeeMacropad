// nanoESP32-C6-N8 — 16-button remote with single/double/long actions
// Uses ZigbeeColorDimmableLight endpoint so Home Assistant can control LED brightness
// All comments are in English
#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Zigbee.h>                      // Arduino Zigbee wrapper (Arduino-ESP32 v3.x)
 
// ----------------------- Hardware configuration -----------------------
#define BUTTON_COUNT 16
const uint8_t buttonPins[BUTTON_COUNT] = {
  1, 2, 3, 4, 5, 6, 7, 10,
  11, 12, 13, 18, 19, 20, 21, 22
};

#define BOOT_BUTTON 9            // physical BOOT button pin on your board
#define NEOPIXEL_PIN 8           // onboard WS2812 pin (NeoPixel)
#define NUM_PIXELS 1

// ----------------------- Timing and thresholds ------------------------
constexpr unsigned long DEBOUNCE_MS     = 30;
constexpr unsigned long DOUBLE_MS       = 400;
constexpr unsigned long LONG_MS         = 1000;
constexpr unsigned long EVENT_FLASH_MS  = 160;   // flash duration for visual feedback

// ----------------------- LED / color definitions ----------------------
Adafruit_NeoPixel rgbLed(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

struct RGB { uint8_t r, g, b; };
const RGB COLOR_OFF    = {0, 0, 0};
const RGB COLOR_GREEN  = {0, 255, 0};
const RGB COLOR_BLUE   = {0, 0, 255};
const RGB COLOR_PURPLE = {180, 0, 255};
const RGB COLOR_RED    = {255, 0, 0};

// brightness level from Zigbee (0..100). Default 50%
volatile uint8_t ledBrightness = 50;

// Helper: set the NeoPixel color scaled by brightness (0..100)
void setNeoPixelColor(const RGB &c, uint8_t brightnessPercent = 100) {
  uint16_t scale = (uint16_t)brightnessPercent * 255 / 100;
  uint8_t r = (uint8_t)(((uint16_t)c.r * scale) / 255);
  uint8_t g = (uint8_t)(((uint16_t)c.g * scale) / 255);
  uint8_t b = (uint8_t)(((uint16_t)c.b * scale) / 255);
  rgbLed.setPixelColor(0, rgbLed.Color(r, g, b));
  rgbLed.show();
}

void flashNeoPixel(const RGB &c, uint16_t duration_ms = EVENT_FLASH_MS) {
  setNeoPixelColor(c, ledBrightness);
  delay(duration_ms);
  setNeoPixelColor(COLOR_OFF, 100);
}

// ----------------------- Button state machine -------------------------
bool lastRawState[BUTTON_COUNT];
unsigned long lastStableTime[BUTTON_COUNT];
bool stableState[BUTTON_COUNT];            // true => pressed (active LOW)
unsigned long pressStartTime[BUTTON_COUNT];
unsigned long lastReleaseTime[BUTTON_COUNT]; // for double click window
bool longAlreadyTriggered[BUTTON_COUNT];

// non-blocking debounce/update function (call frequently)
void handleButtons(void (*onAction)(uint8_t index, const char *action)) {
  unsigned long now = millis();
  for (uint8_t i = 0; i < BUTTON_COUNT; ++i) {
    bool raw = (digitalRead(buttonPins[i]) == LOW); // active LOW
    if (raw != lastRawState[i]) {
      // raw change -> reset stable timer
      lastRawState[i] = raw;
      lastStableTime[i] = now;
    }

    if ((now - lastStableTime[i]) >= DEBOUNCE_MS) {
      if (raw != stableState[i]) {
        // stable state changed
        stableState[i] = raw;
        if (stableState[i]) {
          // pressed
          pressStartTime[i] = now;
          longAlreadyTriggered[i] = false;
        } else {
          // released
          unsigned long pressDur = now - pressStartTime[i];
          if (!longAlreadyTriggered[i]) {
            // candidate for single/double
            if (lastReleaseTime[i] && (now - lastReleaseTime[i]) <= DOUBLE_MS) {
              // double detected
              onAction(i, "double");
              lastReleaseTime[i] = 0;
            } else {
              // store release to wait for double window
              lastReleaseTime[i] = now;
            }
          } else {
            // long was already triggered; ignore release
            lastReleaseTime[i] = 0;
          }
        }
      }
    }

    // long press detection while held
    if (stableState[i] && !longAlreadyTriggered[i]) {
      if ((now - pressStartTime[i]) >= LONG_MS) {
        longAlreadyTriggered[i] = true;
        onAction(i, "long");
        // consume double window
        lastReleaseTime[i] = 0;
      }
    }

    // single click timeout handler
    if (lastReleaseTime[i] && ((now - lastReleaseTime[i]) > DOUBLE_MS)) {
      // nothing came second => single
      onAction(i, "single");
      lastReleaseTime[i] = 0;
    }
  }
}

// ----------------------- Zigbee endpoint ------------------------------
// We'll use a single endpoint implemented with ZigbeeColorDimmableLight
// so the coordinator (Zigbee2MQTT / Home Assistant) can set brightness (level)
// and optionally color. When HA writes level, onLightChange will be invoked.
#define ZIGBEE_ENDPOINT 10
ZigbeeColorDimmableLight zbLight(ZIGBEE_ENDPOINT);

// Callback from Zigbee wrapper when coordinator changes the light state / color / level
// signature: void onLightChange(bool state, uint8_t red, uint8_t green, uint8_t blue, uint8_t level)
void zigbeeLightChanged(bool state, uint8_t red, uint8_t green, uint8_t blue, uint8_t level) {
  // store brightness level (0..100)
  if (level > 100) level = 100; // guard
  ledBrightness = level;
  // Optionally store color components if you want to use them as default flash colors.
  Serial.printf("[ZB] onLightChange state=%d level=%u rgb=%u,%u,%u\n", (int)state, level, red, green, blue);
  // Immediately reflect the new brightness/color on the NeoPixel if the device is on
  if (state) {
    RGB c = { red, green, blue };
    setNeoPixelColor(c, ledBrightness);
  } else {
    setNeoPixelColor(COLOR_OFF, 100);
  }
}

// Helper: produce a short Zigbee report so HA receives an event.
// We set the light ON with requested level/color then turn it OFF quickly.
// Using ZigbeeColorDimmableLight API setLight(state, level, r,g,b)
void emitZigbeeEventWithColor(uint8_t btnIndex, const RGB &c) {
  // Use current ledBrightness as level for reports (0..100)
  uint8_t level = ledBrightness;
  // Set light ON: state true, level, and RGB color
  zbLight.setLight(true, level, c.r, c.g, c.b);
  // short delay to allow coordinator to see the report
  delay(60);
  // set light OFF — keep other params zero
  zbLight.setLight(false, 0, 0, 0, 0);
}

// The action handler called when buttons produce an action
void onButtonAction(uint8_t index, const char *action) {
  Serial.printf("Button %u -> %s\n", index + 1, action);

  // Visual feedback
  if (strcmp(action, "single") == 0)      flashNeoPixel(COLOR_GREEN);
  else if (strcmp(action, "double") == 0) flashNeoPixel(COLOR_BLUE);
  else if (strcmp(action, "long") == 0)   flashNeoPixel(COLOR_PURPLE);

  // Emit a Zigbee report so coordinator can interpret it as an event.
  // We include button index as information in Serial logs. Home Assistant will see On->Off toggles.
  // Because we cannot send custom ZCL commands from Arduino wrapper yet, we use a short on/off report.
  emitZigbeeEventWithColor(index, (strcmp(action, "single")==0)?COLOR_GREEN:
                                  (strcmp(action, "double")==0)?COLOR_BLUE:
                                  COLOR_PURPLE);

  // Special behavior: if this is button 16 (index 15) and it's a long press,
  // blink red on the BOOT button action (user requested BOOT causes red blinking).
  if ((index == 15) && (strcmp(action, "long") == 0)) {
    // Blink red to indicate pairing/reset candidate
    for (int i = 0; i < 6; ++i) {
      setNeoPixelColor(COLOR_RED, ledBrightness);
      delay(120);
      setNeoPixelColor(COLOR_OFF, 100);
      delay(120);
    }
    // Optionally call Zigbee.factoryReset() here if you want to reset on long press
    // Zigbee.factoryReset();
  }
}

// ----------------------- Setup and loop --------------------------------
void setup() {
  Serial.begin(115200);
  delay(50);

  // initialize NeoPixel
  rgbLed.begin();
  setNeoPixelColor(COLOR_OFF, 100);

  // initialize buttons
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  for (uint8_t i = 0; i < BUTTON_COUNT; ++i) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    lastRawState[i] = digitalRead(buttonPins[i]) == LOW;
    lastStableTime[i] = millis();
    stableState[i] = false;
    pressStartTime[i] = 0;
    lastReleaseTime[i] = 0;
    longAlreadyTriggered[i] = false;
  }

  // Configure Zigbee endpoint (Color Dimmable Light) so HA can control level (brightness)
  zbLight.setManufacturerAndModel("Custom", "ZB16BtnRemote");
  zbLight.onLightChange(zigbeeLightChanged);
  Zigbee.addEndpoint(&zbLight);

  // Start Zigbee stack (End Device by Tools->Zigbee mode)
  if (!Zigbee.begin()) {
    Serial.println("Zigbee failed to start. Restarting...");
    delay(2000);
    ESP.restart();
  }

  Serial.println("Waiting for Zigbee network connection...");
  while (!Zigbee.connected()) {
    Serial.print('.');
    delay(200);
  }
  Serial.println("\nZigbee connected.");
}

void loop() {
  // Handle button state machines and call onButtonAction when actions occur
  handleButtons(onButtonAction);

  // BOOT button -> red blink (for pairing feedback or manual factory-reset visual)
  if (digitalRead(BOOT_BUTTON) == LOW) {
    delay(40);
    if (digitalRead(BOOT_BUTTON) == LOW) {
      // short debounce and blink red once
      for (int i = 0; i < 4; ++i) {
        setNeoPixelColor(COLOR_RED, ledBrightness);
        delay(120);
        setNeoPixelColor(COLOR_OFF, 100);
        delay(120);
      }
    }
  }

  delay(2); // tiny yield to let background tasks run
}
