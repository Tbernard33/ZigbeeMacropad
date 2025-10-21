// nanoESP32-C6-N8 — 16-button Zigbee remote with RGB LED and pairing support
#ifndef ZIGBEE_MODE_ED 
#error "Zigbee end device mode is not selected in Tools->Zigbee mode" 
#endif

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Zigbee.h>

// ----------------------- Hardware configuration -----------------------
#define BUTTON_COUNT 16
const uint8_t buttonPins[BUTTON_COUNT] = {
  1, 2, 3, 4, 5, 6, 7, 10,
  11, 12, 13, 18, 19, 20, 21, 22
};

#define BOOT_BUTTON 9
#define NEOPIXEL_PIN 8
#define NUM_PIXELS 1

// ----------------------- Timing and thresholds ------------------------
constexpr unsigned long DEBOUNCE_MS = 30;
constexpr unsigned long DOUBLE_MS = 400;
constexpr unsigned long LONG_MS = 1000;
constexpr unsigned long EVENT_FLASH_MS = 150;

// ----------------------- LED and colors -------------------------------
Adafruit_NeoPixel rgbLed(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

struct RGB { uint8_t r, g, b; };
const RGB COLOR_OFF    = {0, 0, 0};
const RGB COLOR_GREEN  = {0, 255, 0};
const RGB COLOR_BLUE   = {0, 0, 255};
const RGB COLOR_YELLOW = {255, 255, 0};
const RGB COLOR_RED    = {255, 0, 0};

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
  setNeoPixelColor(COLOR_OFF);
}

// ----------------------- Button logic ---------------------------------
bool lastRawState[BUTTON_COUNT];
bool stableState[BUTTON_COUNT];
unsigned long lastStableTime[BUTTON_COUNT];
unsigned long pressStartTime[BUTTON_COUNT];
unsigned long lastReleaseTime[BUTTON_COUNT];
bool longAlreadyTriggered[BUTTON_COUNT];

void handleButtons(void (*onAction)(uint8_t, const char *)) {
  unsigned long now = millis();
  for (uint8_t i = 0; i < BUTTON_COUNT; ++i) {
    bool raw = (digitalRead(buttonPins[i]) == LOW);
    if (raw != lastRawState[i]) {
      lastRawState[i] = raw;
      lastStableTime[i] = now;
    }

    if ((now - lastStableTime[i]) >= DEBOUNCE_MS) {
      if (raw != stableState[i]) {
        stableState[i] = raw;
        if (stableState[i]) {
          pressStartTime[i] = now;
          longAlreadyTriggered[i] = false;
        } else {
          unsigned long pressDur = now - pressStartTime[i];
          if (!longAlreadyTriggered[i]) {
            if (lastReleaseTime[i] && (now - lastReleaseTime[i]) <= DOUBLE_MS) {
              onAction(i, "double");
              lastReleaseTime[i] = 0;
            } else {
              lastReleaseTime[i] = now;
            }
          } else {
            lastReleaseTime[i] = 0;
          }
        }
      }
    }

    if (stableState[i] && !longAlreadyTriggered[i] &&
        (now - pressStartTime[i]) >= LONG_MS) {
      longAlreadyTriggered[i] = true;
      onAction(i, "long");
      lastReleaseTime[i] = 0;
    }

    if (lastReleaseTime[i] && ((now - lastReleaseTime[i]) > DOUBLE_MS)) {
      onAction(i, "single");
      lastReleaseTime[i] = 0;
    }
  }
}

// ----------------------- Zigbee endpoint ------------------------------
#define ZIGBEE_ENDPOINT 10
ZigbeeColorDimmableLight zbLight(ZIGBEE_ENDPOINT);

void zigbeeLightChanged(bool state, uint8_t r, uint8_t g, uint8_t b, uint8_t level) {
  ledBrightness = constrain(level, 0, 100);
  if (state) setNeoPixelColor({r, g, b}, ledBrightness);
  else setNeoPixelColor(COLOR_OFF);
}

void emitZigbeeAction(uint8_t index, uint8_t actionCode, const RGB &color) {
  uint8_t level = (index) * 6 + actionCode;
  zbLight.setLight(true, level, color.r, color.g, color.b);
  delay(60);
  zbLight.setLight(false, 0, 0, 0, 0);
}

void onButtonAction(uint8_t index, const char *action) {
  Serial.printf("Button %u -> %s\n", index + 1, action);

  RGB color = COLOR_OFF;
  uint8_t actionCode = 0;
  if (strcmp(action, "single") == 0) { color = COLOR_GREEN; actionCode = 1; }
  else if (strcmp(action, "double") == 0) { color = COLOR_BLUE; actionCode = 2; }
  else if (strcmp(action, "long") == 0) { color = COLOR_YELLOW; actionCode = 3; }

  flashNeoPixel(color);
  emitZigbeeAction(index, actionCode, color);
}

// ----------------------- Boot button pairing --------------------------
bool pairingActive = false;

void startPairing() {
  pairingActive = true;
  Serial.println("[ZB] Factory reset + Zigbee pairing...");
  Zigbee.factoryReset();   // starts join mode
}

void updatePairingLED() {
  if (pairingActive && !Zigbee.connected()) {
    static unsigned long lastBlink = 0;
    unsigned long now = millis();
    if (now - lastBlink > 300) {
      lastBlink = now;
      static bool on = false;
      on = !on;
      if (on) setNeoPixelColor(COLOR_RED, ledBrightness);
      else setNeoPixelColor(COLOR_OFF);
    }
  } else if (pairingActive && Zigbee.connected()) {
    pairingActive = false;
    setNeoPixelColor(COLOR_OFF);
    Serial.println("[ZB] Connected. Stopping pairing LED.");
  }
}

// ----------------------- Setup & loop ---------------------------------
void setup() {
  Serial.begin(115200);
  rgbLed.begin();
  setNeoPixelColor(COLOR_OFF);

  for (uint8_t i = 0; i < BUTTON_COUNT; ++i) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    stableState[i] = false;
    lastRawState[i] = digitalRead(buttonPins[i]) == LOW;
    lastStableTime[i] = millis();
  }
  pinMode(BOOT_BUTTON, INPUT_PULLUP);

  zbLight.setManufacturerAndModel("Custom", "ZB16BtnRemote");
  zbLight.onLightChange(zigbeeLightChanged);
  Zigbee.addEndpoint(&zbLight);

  if (!Zigbee.begin()) {
    Serial.println("Zigbee failed to start!");
    delay(2000);
    ESP.restart();
  }

  Serial.println("[ZB] Starting...");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(200);
  }
  Serial.println("\n[ZB] Connected.");
}

void loop() {
  handleButtons(onButtonAction);

  if (digitalRead(BOOT_BUTTON) == LOW && !pairingActive) {
    delay(100);
    if (digitalRead(BOOT_BUTTON) == LOW) {
      startPairing();
    }
  }

  updatePairingLED();
}
