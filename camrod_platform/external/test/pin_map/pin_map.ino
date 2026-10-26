// Pin -> strip mapping diagnostic for the CAMROD light MCU (Arduino Nano).
//
// No serial protocol. Powers each WS2815 data pin ONE AT A TIME so you can see
// exactly which physical strip (and how many LEDs) each pin drives. Use it to
// find why one side is dark: a pin that lights nothing (or one stuck LED) is a
// wiring / strip / power problem on that side.
//
// Sequence (repeats), each step ~3 s, also printed on serial @115200:
//   D5 = RED      D6 = GREEN      D7 = BLUE      D8 = WHITE
// Between steps all pins are cleared so only the active pin should show colour.
//
// Overwrites the main firmware; re-upload camrod_light_mcu afterwards.
// Library: FastLED 3.x.

#include <FastLED.h>

#define NUM_LEDS 60  // >= real strip length so the whole strip is addressed

CRGB leds5[NUM_LEDS];
CRGB leds6[NUM_LEDS];
CRGB leds7[NUM_LEDS];
CRGB leds8[NUM_LEDS];

void clearAll() {
  fill_solid(leds5, NUM_LEDS, CRGB::Black);
  fill_solid(leds6, NUM_LEDS, CRGB::Black);
  fill_solid(leds7, NUM_LEDS, CRGB::Black);
  fill_solid(leds8, NUM_LEDS, CRGB::Black);
  FastLED.show();
}

void setup() {
  FastLED.addLeds<WS2812B, 5, GRB>(leds5, NUM_LEDS);
  FastLED.addLeds<WS2812B, 6, GRB>(leds6, NUM_LEDS);
  FastLED.addLeds<WS2812B, 7, GRB>(leds7, NUM_LEDS);
  FastLED.addLeds<WS2812B, 8, GRB>(leds8, NUM_LEDS);
  FastLED.setBrightness(150);
  clearAll();
  Serial.begin(115200);
  Serial.println(F("pin_map: D5=RED D6=GREEN D7=BLUE D8=WHITE, one at a time"));
}

void step(CRGB *buf, const __FlashStringHelper *msg, CRGB color) {
  clearAll();
  Serial.println(msg);
  fill_solid(buf, NUM_LEDS, color);
  FastLED.show();
  delay(3000);
}

void loop() {
  step(leds5, F("D5 (RED)   -> which strip?"), CRGB::Red);
  step(leds6, F("D6 (GREEN) -> which strip?"), CRGB::Green);
  step(leds7, F("D7 (BLUE)  -> which strip?"), CRGB::Blue);
  step(leds8, F("D8 (WHITE) -> which strip?"), CRGB::White);
  clearAll();
  Serial.println(F("--- all off, repeating ---"));
  delay(1500);
}
