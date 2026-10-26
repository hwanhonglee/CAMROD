// HH_260708 - CAMROD exterior light MCU (Arduino Nano, ATmega328P).
//
// Receives newline-terminated JSON from camrod_platform/mcu_serial_bridge_node
// and drives the headlight relay + WS2815 turn indicator strips.
//
//   PC -> MCU @ 5 Hz : {"h":0|1,"i":"O"|"L"|"R"|"H","q":<seq>}
//   MCU -> PC @ 1 Hz : {"a":<last q>,"h":0|1,"i":"<mode>","e":<error>}
//
// Blink timing is generated locally (BLINK_PERIOD_MS); the PC only selects the
// mode. The indicators are command-only: they are lit solely while the PC is
// actively commanding them. On link loss (no frame for LINK_TIMEOUT_MS) the
// strips go dark - the MCU stays powered across a PC reboot, so an autonomous
// link-loss blink would light the indicators with nothing running. link loss is
// still reported in the ack "e" field for diagnostics. The headlight relay keeps
// its last commanded state during link loss - killing the headlight at night
// because of a serial fault would be worse than keeping it.
//
// Wiring (project decision, docs/lights-design-doc.html):
//   D10 -> relay IN (headlight). Active HIGH; use a relay module with a
//          pull-down so the floating pin during reset cannot chatter it.
//   D5  -> left  WS2815 DI   (D6 = left  backup data, driven in code, mirrors D5)
//   D7  -> right WS2815 DI   (D8 = right backup data, driven in code, mirrors D7)
//   WS2815 strips are 12 V powered; data lines are 5 V; common GND required.
//
// Libraries (Library Manager): ArduinoJson 6.x, FastLED 3.x.

#include <ArduinoJson.h>
#include <FastLED.h>

// ---- Wiring / strip configuration -----------------------------------------
#define PIN_RELAY_HEADLIGHT 10
// HH - pins swapped vs the original doc: as installed, D7/D8 feed the physical
// LEFT strip and D5/D6 the physical RIGHT strip. Mapped here so a LEFT command
// lights the left strip (wiring left as-is, corrected in firmware).
#define PIN_LEFT_DATA 7
#define PIN_RIGHT_DATA 5
#define PIN_LEFT_BACKUP 8     // left  backup data line, mirrors PIN_LEFT_DATA
#define PIN_RIGHT_BACKUP 6    // right backup data line, mirrors PIN_RIGHT_DATA
#define NUM_LEDS_PER_SIDE 30  // adjust to the strip length actually installed

// ---- Behaviour -------------------------------------------------------------
#define SERIAL_BAUD 115200
#define LINK_TIMEOUT_MS 1000UL   // must match lights.yaml mcu_timeout_s
#define BLINK_PERIOD_MS 800UL    // 1.25 Hz automotive-style blink
#define ACK_PERIOD_MS 1000UL
// Re-assert the strip contents at this interval even without a state change, so
// a pixel that latched a stray colour at power-on (12 V strip vs 5 V data power
// sequencing) gets cleared within one period. 1 Hz keeps the ~3.6 ms show()
// blackout rare enough not to disturb the 5 Hz serial link.
#define STRIP_REFRESH_MS 1000UL
#define LINE_BUFFER_SIZE 96      // frames are ~30 bytes; SRAM is 2 KB total

// Amber indicator colour (WS2815 = WS2812 protocol, RGB order).
#define INDICATOR_COLOR CRGB(255, 96, 0)
#define INDICATOR_BRIGHTNESS 200

// Error codes reported in the "e" ack field.
enum ErrorCode : uint8_t {
  ERR_NONE = 0,
  ERR_JSON = 1,       // last received line failed to parse
  ERR_LINK_LOST = 2,  // failsafe hazard active
};

enum IndicatorMode : uint8_t { MODE_OFF, MODE_LEFT, MODE_RIGHT, MODE_HAZARD };

CRGB left_leds[NUM_LEDS_PER_SIDE];
CRGB right_leds[NUM_LEDS_PER_SIDE];

char line_buffer[LINE_BUFFER_SIZE];
uint8_t line_length = 0;

IndicatorMode commanded_mode = MODE_OFF;
bool headlight_on = false;
long last_sequence = -1;
uint8_t last_error = ERR_NONE;

bool link_ever_up = false;      // becomes true on the first valid frame
unsigned long last_frame_ms = 0;
unsigned long last_ack_ms = 0;

void setup() {
  pinMode(PIN_RELAY_HEADLIGHT, OUTPUT);
  digitalWrite(PIN_RELAY_HEADLIGHT, LOW);

  FastLED.addLeds<WS2812B, PIN_LEFT_DATA, RGB>(left_leds, NUM_LEDS_PER_SIDE);
  FastLED.addLeds<WS2812B, PIN_RIGHT_DATA, RGB>(right_leds, NUM_LEDS_PER_SIDE);
  // Backup data lines share the same pixel buffers, so FastLED.show() drives
  // them with identical data (redundant feed into the WS2815 backup input).
  FastLED.addLeds<WS2812B, PIN_LEFT_BACKUP, RGB>(left_leds, NUM_LEDS_PER_SIDE);
  FastLED.addLeds<WS2812B, PIN_RIGHT_BACKUP, RGB>(right_leds, NUM_LEDS_PER_SIDE);
  FastLED.setBrightness(INDICATOR_BRIGHTNESS);
  fill_solid(left_leds, NUM_LEDS_PER_SIDE, CRGB::Black);
  fill_solid(right_leds, NUM_LEDS_PER_SIDE, CRGB::Black);
  FastLED.show();

  Serial.begin(SERIAL_BAUD);
}

void handleLine(const char *line) {
  StaticJsonDocument<96> doc;
  const DeserializationError error = deserializeJson(doc, line);
  if (error != DeserializationError::Ok || !doc.containsKey("i")) {
    last_error = ERR_JSON;
    return;
  }

  headlight_on = doc["h"] == 1;
  last_sequence = doc["q"] | -1L;

  const char *indicator = doc["i"] | "O";
  switch (indicator[0]) {
    case 'L': commanded_mode = MODE_LEFT; break;
    case 'R': commanded_mode = MODE_RIGHT; break;
    case 'H': commanded_mode = MODE_HAZARD; break;
    default:  commanded_mode = MODE_OFF; break;
  }

  last_error = ERR_NONE;
  link_ever_up = true;
  last_frame_ms = millis();
}

void pollSerial() {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());
    if (incoming == '\n') {
      line_buffer[line_length] = '\0';
      if (line_length > 0) {
        handleLine(line_buffer);
      }
      line_length = 0;
    } else if (line_length < LINE_BUFFER_SIZE - 1) {
      line_buffer[line_length++] = incoming;
    } else {
      line_length = 0;  // oversized garbage - drop and resync on next newline
      last_error = ERR_JSON;
    }
  }
}

const char *modeToWire(IndicatorMode mode) {
  switch (mode) {
    case MODE_LEFT: return "L";
    case MODE_RIGHT: return "R";
    case MODE_HAZARD: return "H";
    default: return "O";
  }
}

void sendAck() {
  StaticJsonDocument<96> doc;
  doc["a"] = last_sequence;
  doc["h"] = headlight_on ? 1 : 0;
  doc["i"] = modeToWire(commanded_mode);
  doc["e"] = last_error;
  serializeJson(doc, Serial);
  Serial.print('\n');
}

void loop() {
  pollSerial();

  const unsigned long now_ms = millis();
  const bool link_lost =
      link_ever_up && (now_ms - last_frame_ms > LINK_TIMEOUT_MS);

  // Indicators follow commands only - no autonomous blink. On link loss the
  // strips go dark (not hazard): a reboot leaves the MCU powered with a stale
  // link_ever_up, and auto-hazard then lit the indicators with nothing running.
  // Nothing may be lit until the PC actively commands it. last_error still
  // reports the link loss for diagnostics; it just no longer drives the LEDs.
  IndicatorMode active_mode = commanded_mode;
  if (link_lost) {
    active_mode = MODE_OFF;
    last_error = ERR_LINK_LOST;
  } else if (!link_ever_up) {
    active_mode = MODE_OFF;  // PC still booting: stay dark
  }

  // Local blink generation: phase from millis so all sides stay in sync.
  const bool blink_on = (now_ms % BLINK_PERIOD_MS) < (BLINK_PERIOD_MS / 2);

  const bool left_active =
      active_mode == MODE_LEFT || active_mode == MODE_HAZARD;
  const bool right_active =
      active_mode == MODE_RIGHT || active_mode == MODE_HAZARD;
  const bool left_lit = left_active && blink_on;
  const bool right_lit = right_active && blink_on;

  // HH_260728 - FastLED.show() disables interrupts for ~3.6 ms (4 x 30 WS2815 LEDs, incl.
  // the two backup data lines) while
  // the UART RX buffer holds only 2 bytes, so calling it every loop drops most
  // incoming bytes at 115200 baud. Refresh on a state change, plus once every
  // STRIP_REFRESH_MS so a stray power-on pixel is cleared even while idle.
  static bool prev_left_lit = false;
  static bool prev_right_lit = false;
  static unsigned long last_show_ms = 0;
  const bool state_changed =
      (left_lit != prev_left_lit) || (right_lit != prev_right_lit);
  if (state_changed || (now_ms - last_show_ms >= STRIP_REFRESH_MS)) {
    prev_left_lit = left_lit;
    prev_right_lit = right_lit;
    last_show_ms = now_ms;
    fill_solid(left_leds, NUM_LEDS_PER_SIDE,
               left_lit ? INDICATOR_COLOR : CRGB::Black);
    fill_solid(right_leds, NUM_LEDS_PER_SIDE,
               right_lit ? INDICATOR_COLOR : CRGB::Black);
    FastLED.show();
  }

  // Headlight relay: independent channel, holds last state on link loss.
  digitalWrite(PIN_RELAY_HEADLIGHT, headlight_on ? HIGH : LOW);

  if (now_ms - last_ack_ms >= ACK_PERIOD_MS) {
    last_ack_ms = now_ms;
    sendAck();
  }
}
