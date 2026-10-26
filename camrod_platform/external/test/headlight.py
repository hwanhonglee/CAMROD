#!/usr/bin/env python3
"""Hold the CAMROD headlight relay (D10) ON or OFF over JSON, strips dark.

The relay keeps its last commanded state when the serial link drops, so one
frame is enough to flip it. This helper streams the command at 5 Hz anyway,
with the indicator forced OFF ("i":"O"), so the MCU sees a live link and
reports no error while you work on the headlight.

Usage:
    python3 headlight.py on      # relay ON,  hold until Ctrl+C
    python3 headlight.py off     # relay OFF, hold until Ctrl+C
    python3 headlight.py on --port /dev/ttyUSB5
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
DEFAULT_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG003KJ3-if00-port0"


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("state", choices=["on", "off"], help="headlight relay state")
    ap.add_argument("--port", default=DEFAULT_PORT, help="serial port")
    args = ap.parse_args()

    h = 1 if args.state == "on" else 0
    frame = (json.dumps({"h": h, "i": "O"}, separators=(",", ":")) + "\n").encode()

    try:
        ser = serial.Serial(args.port, BAUD, timeout=0.2)
    except serial.SerialException as e:
        sys.exit(f"Cannot open {args.port}: {e}")

    # Opening the port auto-resets the Nano (DTR); wait for the bootloader +
    # setup() so the first frames are not dropped into the bootloader.
    time.sleep(2.0)
    ser.reset_input_buffer()

    print(f"Headlight relay {args.state.upper()} - streaming at 5 Hz to keep the "
          f"link alive (no hazard). Ctrl+C to stop.")
    try:
        while True:
            ser.write(frame)
            ser.flush()
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nStopped. (After ~1 s of silence the strips enter failsafe "
              "hazard; the relay holds its last state.)")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
