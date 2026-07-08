// 260708 - CAMROD exterior light MCU (Arduino Nano, ATmega328P).
//
// Receives newline-terminated JSON from camrod_platform/mcu_serial_bridge_node
// and drives the headlight relay + WS2815 turn indicator strips.
//
//   PC -> MCU @ 5 Hz : {"h":0|1,"i":"O"|"L"|"R"|"H","q":<seq>}
//   MCU -> PC @ 1 Hz : {"a":<last q>,"h":0|1,"i":"<mode>","e":<error>}
//
// Blink timing is generated locally (BLINK_PERIOD_MS); the PC only selects the
// mode. Fail-safe: after the FIRST valid frame has been received, losing the
// link for LINK_TIMEOUT_MS switches the indicators to autonomous HAZARD blink
// (project decision). Before the first frame (PC still booting) everything
// stays dark so a PC restart cannot flash hazards. The headlight relay keeps
// its last commanded state during link loss - killing the headlight at night
// because of a serial fault would be worse than keeping it.
//
// Wiring (project decision, docs/lights-design-doc.html):
//   D10 -> relay IN (headlight). Active HIGH; use a relay module with a
//          pull-down so the floating pin during reset cannot chatter it.
//   D5  -> left  WS2815 DI  (D6 = left  BI backup, parallel wiring, no code)
//   D7  -> right WS2815 DI  (D8 = right BI backup, parallel wiring, no code)
//   WS2815 strips are 12 V powered; data lines are 5 V; common GND required.
//
// Libraries (Library Manager): ArduinoJson 6.x, FastLED 3.x.

#include <ArduinoJson.h>
#include <FastLED.h>

// ---- Wiring / strip configuration -----------------------------------------
#define PIN_RELAY_HEADLIGHT 10
#define PIN_LEFT_DATA 5
#define PIN_RIGHT_DATA 7
#define NUM_LEDS_PER_SIDE 30  // adjust to the strip length actually installed

// ---- Behaviour -------------------------------------------------------------
#define SERIAL_BAUD 115200
#define LINK_TIMEOUT_MS 1000UL   // must match lights.yaml mcu_timeout_s
#define BLINK_PERIOD_MS 800UL    // 1.25 Hz automotive-style blink
#define ACK_PERIOD_MS 1000UL
#define LINE_BUFFER_SIZE 96      // frames are ~30 bytes; SRAM is 2 KB total

// Amber indicator colour (WS2815 = WS2812 protocol, GRB order).
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

  FastLED.addLeds<WS2812B, PIN_LEFT_DATA, GRB>(left_leds, NUM_LEDS_PER_SIDE);
  FastLED.addLeds<WS2812B, PIN_RIGHT_DATA, GRB>(right_leds, NUM_LEDS_PER_SIDE);
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

  IndicatorMode active_mode = commanded_mode;
  if (link_lost) {
    active_mode = MODE_HAZARD;  // project failsafe decision
    last_error = ERR_LINK_LOST;
  } else if (!link_ever_up) {
    active_mode = MODE_OFF;  // PC still booting: stay dark, no false hazard
  }

  // Local blink generation: phase from millis so all sides stay in sync.
  const bool blink_on = (now_ms % BLINK_PERIOD_MS) < (BLINK_PERIOD_MS / 2);
  const CRGB lit = blink_on ? INDICATOR_COLOR : CRGB::Black;

  const bool left_active =
      active_mode == MODE_LEFT || active_mode == MODE_HAZARD;
  const bool right_active =
      active_mode == MODE_RIGHT || active_mode == MODE_HAZARD;
  fill_solid(left_leds, NUM_LEDS_PER_SIDE, left_active ? lit : CRGB::Black);
  fill_solid(right_leds, NUM_LEDS_PER_SIDE, right_active ? lit : CRGB::Black);
  FastLED.show();

  // Headlight relay: independent channel, holds last state on link loss.
  digitalWrite(PIN_RELAY_HEADLIGHT, headlight_on ? HIGH : LOW);

  if (now_ms - last_ack_ms >= ACK_PERIOD_MS) {
    last_ack_ms = now_ms;
    sendAck();
  }
}
