// LED/wiring test sketch for the CAMROD exterior light MCU (Arduino Nano).
//
// Standalone hardware check - NO serial protocol. Just power the board and
// watch the strips. It walks through a fixed sequence so you can confirm:
//   - left WS2815 strip lights (D7)
//   - right WS2815 strip lights (D5)
//   - both sides (hazard look)
//   - a per-LED chase to spot dead/miswired pixels
//   - headlight relay clicks on/off (D10)
// The onboard LED (D13) blinks once per second as a "sketch is alive" heartbeat.
//
// Pins and strip length match camrod_light_mcu.ino. Adjust NUM_LEDS_PER_SIDE
// to the strip length actually installed.
//
// Libraries (Library Manager): FastLED 3.x.

#include <FastLED.h>

#define PIN_RELAY_HEADLIGHT 10
// As installed, D7/D8 feed the physical LEFT strip and D5/D6 the RIGHT
// one. Mirrored from camrod_light_mcu.ino so this sketch and the firmware
// agree on which side is which.
#define PIN_LEFT_DATA 7
#define PIN_RIGHT_DATA 5
#define PIN_ONBOARD_LED 13
#define NUM_LEDS_PER_SIDE 30  // match the real strip length

#define INDICATOR_COLOR CRGB(255, 96, 0)  // amber, RGB order on WS2815
#define INDICATOR_BRIGHTNESS 200
#define STEP_MS 1000UL  // how long each test step is held

CRGB left_leds[NUM_LEDS_PER_SIDE];
CRGB right_leds[NUM_LEDS_PER_SIDE];

void showBoth() { FastLED.show(); }

void bothBlack() {
  fill_solid(left_leds, NUM_LEDS_PER_SIDE, CRGB::Black);
  fill_solid(right_leds, NUM_LEDS_PER_SIDE, CRGB::Black);
  showBoth();
}

void setup() {
  pinMode(PIN_RELAY_HEADLIGHT, OUTPUT);
  digitalWrite(PIN_RELAY_HEADLIGHT, LOW);
  pinMode(PIN_ONBOARD_LED, OUTPUT);

  FastLED.addLeds<WS2812B, PIN_LEFT_DATA, RGB>(left_leds, NUM_LEDS_PER_SIDE);
  FastLED.addLeds<WS2812B, PIN_RIGHT_DATA, RGB>(right_leds, NUM_LEDS_PER_SIDE);
  FastLED.setBrightness(INDICATOR_BRIGHTNESS);
  bothBlack();

  Serial.begin(115200);
  Serial.println(F("CAMROD light MCU - LED test. Watch the strips."));
}

// Blink the onboard LED and hold for `ms`, so each step shows a heartbeat.
void holdWithHeartbeat(unsigned long ms) {
  const unsigned long start = millis();
  while (millis() - start < ms) {
    digitalWrite(PIN_ONBOARD_LED, (millis() % 1000) < 500 ? HIGH : LOW);
  }
}

void loop() {
  // 1) Left side only.
  Serial.println(F("Step 1: LEFT strip"));
  fill_solid(left_leds, NUM_LEDS_PER_SIDE, INDICATOR_COLOR);
  fill_solid(right_leds, NUM_LEDS_PER_SIDE, CRGB::Black);
  showBoth();
  holdWithHeartbeat(STEP_MS);

  // 2) Right side only.
  Serial.println(F("Step 2: RIGHT strip"));
  fill_solid(left_leds, NUM_LEDS_PER_SIDE, CRGB::Black);
  fill_solid(right_leds, NUM_LEDS_PER_SIDE, INDICATOR_COLOR);
  showBoth();
  holdWithHeartbeat(STEP_MS);

  // 3) Both sides (hazard look), blink 3 times.
  Serial.println(F("Step 3: BOTH (hazard)"));
  for (uint8_t i = 0; i < 3; i++) {
    fill_solid(left_leds, NUM_LEDS_PER_SIDE, INDICATOR_COLOR);
    fill_solid(right_leds, NUM_LEDS_PER_SIDE, INDICATOR_COLOR);
    showBoth();
    holdWithHeartbeat(400);
    bothBlack();
    holdWithHeartbeat(400);
  }

  // 4) Per-LED chase - a dead or miswired pixel shows up here.
  Serial.println(F("Step 4: chase (per-pixel)"));
  bothBlack();
  for (uint16_t i = 0; i < NUM_LEDS_PER_SIDE; i++) {
    left_leds[i] = INDICATOR_COLOR;
    right_leds[i] = INDICATOR_COLOR;
    showBoth();
    holdWithHeartbeat(60);
  }
  holdWithHeartbeat(300);

  // 5) RGB sanity (confirms color order): red, green, blue on both sides.
  Serial.println(F("Step 5: R / G / B color check"));
  const CRGB colors[] = {CRGB::Red, CRGB::Green, CRGB::Blue};
  for (uint8_t c = 0; c < 3; c++) {
    fill_solid(left_leds, NUM_LEDS_PER_SIDE, colors[c]);
    fill_solid(right_leds, NUM_LEDS_PER_SIDE, colors[c]);
    showBoth();
    holdWithHeartbeat(700);
  }

  // 6) Headlight relay click on/off.
  Serial.println(F("Step 6: headlight relay ON"));
  bothBlack();
  digitalWrite(PIN_RELAY_HEADLIGHT, HIGH);
  holdWithHeartbeat(STEP_MS);
  Serial.println(F("Step 6: headlight relay OFF"));
  digitalWrite(PIN_RELAY_HEADLIGHT, LOW);
  holdWithHeartbeat(STEP_MS);

  Serial.println(F("--- cycle complete, repeating ---"));
}
