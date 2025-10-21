#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Zigbee.h>

/* === Configuration === */
#define BUTTON_COUNT 16
const uint8_t buttonPins[BUTTON_COUNT] = {
  1, 2, 3, 4, 5, 6, 7, 10,
  11, 12, 13, 18, 19, 20, 21, 22
};
#define BOOT_BUTTON 9      // bouton BOOT du NanoESP32-C6-N8
#define LED_PIN 8

#define DEBOUNCE_MS 30
#define DOUBLE_CLICK_MS 400
#define LONG_PRESS_MS 1000

/* === LED RGB === */
Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);
struct RGB { uint8_t r, g, b; };
const RGB COLOR_OFF    = {0, 0, 0};
const RGB COLOR_GREEN  = {0, 255, 0};
const RGB COLOR_BLUE   = {0, 0, 255};
const RGB COLOR_PURPLE = {180, 0, 255};
const RGB COLOR_RED    = {255, 0, 0};

void setColor(const RGB &c, uint8_t br = 50) {
  uint8_t r = (c.r * br) / 100, g = (c.g * br) / 100, b = (c.b * br) / 100;
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}
void flashColor(const RGB &c, uint16_t ms = 150) {
  setColor(c); delay(ms); setColor(COLOR_OFF);
}
void blinkRed(uint8_t times = 6, uint16_t onMs = 120, uint16_t offMs = 120) {
  for (uint8_t i = 0; i < times; i++) {
    setColor(COLOR_RED); delay(onMs);
    setColor(COLOR_OFF); delay(offMs);
  }
}

/* === Détection boutons === */
bool lastState[BUTTON_COUNT];
unsigned long pressTime[BUTTON_COUNT];
unsigned long lastClickTime[BUTTON_COUNT];
bool longTriggered[BUTTON_COUNT];

/* === Zigbee === */
#define ENDPOINT_ID 10
ZigbeeLight zbDevice(ENDPOINT_ID);   // ✅ utiliser ZigbeeLight (fonctionne avec setLight)

/* Envoi d’un “report” simple */
void sendZigbeeAction(uint8_t id, const char *action) {
  Serial.printf("[ZB] Bouton %u : %s\n", id + 1, action);
  // Change brièvement l’état pour provoquer un report Zigbee
  zbDevice.setLight(true);
  delay(30);
  zbDevice.setLight(false);
}

/* === Setup === */
void setup() {
  Serial.begin(115200);
  led.begin(); setColor(COLOR_OFF);

  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  for (int i = 0; i < BUTTON_COUNT; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    lastState[i] = HIGH;
    pressTime[i] = 0;
    lastClickTime[i] = 0;
    longTriggered[i] = false;
  }

  zbDevice.setManufacturerAndModel("Custom", "ZB16ButtonRemote");
  zbDevice.onLightChange([](bool s){ /* ignoré ici */ });
  Zigbee.addEndpoint(&zbDevice);

  if (!Zigbee.begin()) {
    Serial.println("Zigbee start failed, reboot...");
    delay(2000);
    ESP.restart();
  }

  Serial.println("Connexion au réseau Zigbee...");
  while (!Zigbee.connected()) { Serial.print("."); delay(200); }
  Serial.println("\n✅ Connecté au réseau Zigbee !");
}

/* === Boucle principale === */
void loop() {
  unsigned long now = millis();

  /* Boutons physiques */
  for (int i = 0; i < BUTTON_COUNT; i++) {
    bool state = digitalRead(buttonPins[i]);
    if (state != lastState[i]) {
      delay(DEBOUNCE_MS);
      state = digitalRead(buttonPins[i]);
      if (state == LOW) {
        pressTime[i] = now;
        longTriggered[i] = false;
      } else {  // relâchement
        unsigned long dur = now - pressTime[i];
        if (!longTriggered[i]) {
          if (lastClickTime[i] && (now - lastClickTime[i]) < DOUBLE_CLICK_MS) {
            flashColor(COLOR_BLUE);
            sendZigbeeAction(i, "double");
            lastClickTime[i] = 0;
          } else {
            lastClickTime[i] = now;
          }
        }
      }
      lastState[i] = state;
    }

    // Long press
    if (!longTriggered[i] && lastState[i] == LOW && (now - pressTime[i]) > LONG_PRESS_MS) {
      flashColor(COLOR_PURPLE);
      sendZigbeeAction(i, "long");
      longTriggered[i] = true;
    }

    // Single press
    if (lastClickTime[i] && (now - lastClickTime[i]) > DOUBLE_CLICK_MS && !longTriggered[i]) {
      flashColor(COLOR_GREEN);
      sendZigbeeAction(i, "single");
      lastClickTime[i] = 0;
    }
  }

  /* Bouton BOOT → clignotement rouge */
  if (digitalRead(BOOT_BUTTON) == LOW) {
    delay(50);
    if (digitalRead(BOOT_BUTTON) == LOW) {
      Serial.println("BOOT button → blink red");
      blinkRed();
    }
  }

  delay(2);
}
