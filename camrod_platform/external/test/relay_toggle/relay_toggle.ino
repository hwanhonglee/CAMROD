// Standalone relay toggle test for the CAMROD headlight relay (Arduino Nano).
// No serial protocol: power the board and D10 flips HIGH/LOW every second so
// the relay clicks ON/OFF - a direct hardware check of the relay + wiring.
// Overwrites the main firmware; re-upload camrod_light_mcu afterwards.

#define PIN_RELAY_HEADLIGHT 10
void setup() { pinMode(PIN_RELAY_HEADLIGHT, OUTPUT); }
void loop() {
  digitalWrite(PIN_RELAY_HEADLIGHT, HIGH);  // 릴레이 ON
  delay(1000);
  digitalWrite(PIN_RELAY_HEADLIGHT, LOW);   // 릴레이 OFF
  delay(1000);
}
