#!/usr/bin/env python3
"""Indicator (turn-signal) test for camrod_light_mcu.ino, over JSON.

Alternates LEFT and RIGHT turn signals, holding each for a fixed time and
repeating, so you can watch both sides blink in turn.

    LEFT  (D5 + D6 backup)  for N seconds  -> left strip blinks  ~1.25 Hz
    RIGHT (D7 + D8 backup)  for N seconds  -> right strip blinks ~1.25 Hz
    ... repeat ...

Blink is generated on the MCU (~1.25 Hz); this script only selects the mode and
streams the command at 5 Hz so the link stays alive (no failsafe hazard). The
headlight relay is kept OFF ("h":0) throughout.

Default port is /dev/camrod_light_mcu.

Usage:
    python3 test_indicator.py                 # 10 s each, loop forever (Ctrl+C)
    python3 test_indicator.py --hold 5        # 5 s each side
    python3 test_indicator.py --cycles 3      # 3 LEFT/RIGHT rounds then stop
    python3 test_indicator.py --port /dev/ttyUSB0 -v
"""

import argparse
import json
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed. Run: python3 -m pip install pyserial")

BAUD = 115200
SEND_PERIOD = 0.2  # 5 Hz
DEFAULT_PORT = "/dev/camrod_light_mcu"


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=DEFAULT_PORT, help="serial port")
    ap.add_argument("--hold", type=float, default=10.0,
                    help="seconds to hold each side (default: 10)")
    ap.add_argument("--cycles", type=int, default=0,
                    help="LEFT/RIGHT rounds to run (default: 0 = loop forever)")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="print every ack frame")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, BAUD, timeout=0.2)
    except serial.SerialException as e:
        sys.exit(f"Cannot open {args.port}: {e}")

    # Opening the port auto-resets the Nano (DTR); wait for bootloader + setup().
    time.sleep(2.0)
    ser.reset_input_buffer()

    def stream(ind, label):
        print(f"\n▶ {label}: i={ind!r}  ({args.hold:.0f}s) - watch the strip blink")
        end = time.time() + args.hold
        last = None
        while time.time() < end:
            ser.write((json.dumps({"h": 0, "i": ind}) + "\n").encode())
            ser.flush()
            raw = ser.readline()
            if raw:
                try:
                    ack = json.loads(raw.decode(errors="replace").strip())
                    if isinstance(ack, dict) and "i" in ack:
                        last = ack
                        if args.verbose:
                            print(f"    <- {ack}")
                except (json.JSONDecodeError, ValueError):
                    pass
            time.sleep(SEND_PERIOD)
        ok = last is not None and last.get("i") == ind and last.get("e") == 0
        print(f"  ack: {last}  -> {'OK' if ok else 'CHECK'}")

    print(f"Opening {args.port} @ {BAUD} ... LEFT/RIGHT {args.hold:.0f}s each"
          + (f", {args.cycles} rounds" if args.cycles else ", loop (Ctrl+C)"))
    try:
        n = 0
        while True:
            n += 1
            print(f"\n===== round {n}"
                  + (f"/{args.cycles}" if args.cycles else "") + " =====")
            stream("L", "LEFT  (D5 + D6)")
            stream("R", "RIGHT (D7 + D8)")
            if args.cycles and n >= args.cycles:
                break
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        # Turn indicators off on exit (one short burst), then release the port.
        try:
            for _ in range(5):
                ser.write((json.dumps({"h": 0, "i": "O"}) + "\n").encode())
                ser.flush()
                time.sleep(SEND_PERIOD)
        except Exception:
            pass
        ser.close()
    print("Done. (After the port closes, the strips enter failsafe hazard "
          "until the PC sends again.)")


if __name__ == "__main__":
    main()
