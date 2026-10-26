#!/usr/bin/env python3
"""Headlight relay (D10) on/off test for camrod_light_mcu.ino, over JSON.

Speaks the same serial contract as the firmware:

    PC  -> MCU @ 5 Hz : {"h":0|1,"i":"O","q":<seq>}   (h = headlight on/off)
    MCU -> PC  @ 1 Hz : {"a":<last q>,"h":0|1,"i":..,"e":<error>}

It toggles the headlight through several ON/OFF cycles, holding each state so
you can hear the relay click and see the headlight. After each state it reads
the MCU ack back and checks the "h" field echoes what was commanded, i.e. the
firmware actually latched the relay state we sent over JSON.

The indicator is kept at "O" (off) throughout so only the relay moves.

Default port is the fixed udev symlink /dev/camrod_light_mcu.

Usage:
    python3 test_headlight_relay.py
    python3 test_headlight_relay.py --cycles 5 --hold 1.5
    python3 test_headlight_relay.py --port /dev/ttyUSB5 -v
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
SEND_HZ = 5
SEND_PERIOD = 1.0 / SEND_HZ
ERR_NONE = 0

GREEN, RED, DIM, RESET = "\033[32m", "\033[31m", "\033[2m", "\033[0m"


class RelayTester:
    def __init__(self, port, verbose):
        self.ser = serial.Serial(port, BAUD, timeout=0.2)
        self.seq = 0
        self.verbose = verbose
        self.passed = 0
        self.failed = 0
        # Arduino auto-resets on open; wait for the bootloader + setup().
        time.sleep(2.0)
        self.ser.reset_input_buffer()

    def send(self, headlight):
        """Send one JSON frame with the headlight bit, indicator forced OFF."""
        self.seq += 1
        frame = {"h": 1 if headlight else 0, "i": "O", "q": self.seq}
        self.ser.write((json.dumps(frame, separators=(",", ":")) + "\n").encode())
        self.ser.flush()
        return self.seq

    def read_ack(self, deadline):
        while time.time() < deadline:
            raw = self.ser.readline()
            if not raw:
                continue
            try:
                ack = json.loads(raw.decode(errors="replace").strip())
            except (json.JSONDecodeError, ValueError):
                continue
            if isinstance(ack, dict) and "h" in ack and "e" in ack:
                if self.verbose:
                    print(f"{DIM}    <- {ack}{RESET}")
                return ack
        return None

    def drive(self, headlight, hold_s):
        """Stream the headlight command at 5 Hz for hold_s; return latest ack."""
        last_ack = None
        end = time.time() + hold_s
        while time.time() < end:
            self.send(headlight)
            ack = self.read_ack(time.time() + SEND_PERIOD)
            if ack is not None:
                last_ack = ack
        return last_ack

    def check(self, label, condition, detail=""):
        tag = f"{GREEN}PASS{RESET}" if condition else f"{RED}FAIL{RESET}"
        if condition:
            self.passed += 1
        else:
            self.failed += 1
        print(f"  [{tag}] {label}{(' - ' + detail) if detail else ''}")
        return condition

    def test_state(self, headlight, hold_s):
        name = "ON " if headlight else "OFF"
        want = 1 if headlight else 0
        print(f"\nHEADLIGHT {name}: send h={want} (listen for relay click)")
        ack = self.drive(headlight, hold_s)
        if not self.check(f"HEADLIGHT {name}: ack received", ack is not None):
            return
        self.check(f"HEADLIGHT {name}: h echo == {want}",
                   ack.get("h") == want, f"got h={ack.get('h')}")
        self.check(f"HEADLIGHT {name}: error == NONE",
                   ack.get("e") == ERR_NONE, f"got e={ack.get('e')}")

    def close(self):
        # Leave the headlight OFF on exit.
        try:
            self.drive(False, 0.5)
            self.ser.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/camrod_light_mcu",
                    help="serial port (default: /dev/camrod_light_mcu)")
    ap.add_argument("--cycles", type=int, default=3,
                    help="number of ON/OFF cycles (default: 3)")
    ap.add_argument("--hold", type=float, default=1.5,
                    help="seconds to hold each state (default: 1.5)")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="print every ack frame")
    args = ap.parse_args()

    print(f"Opening {args.port} @ {BAUD} ...")
    try:
        t = RelayTester(args.port, args.verbose)
    except serial.SerialException as e:
        sys.exit(f"{RED}Cannot open {args.port}: {e}{RESET}")

    try:
        for i in range(args.cycles):
            print(f"\n--- cycle {i + 1}/{args.cycles} ---")
            t.test_state(True, args.hold)
            t.test_state(False, args.hold)
    finally:
        t.close()

    print(f"\n{'='*40}")
    total = t.passed + t.failed
    colour = GREEN if t.failed == 0 else RED
    print(f"{colour}RESULT: {t.passed}/{total} checks passed, "
          f"{t.failed} failed{RESET}")
    print(f"{DIM}(Relay click/headlight is physical - confirm it by ear/eye.){RESET}")
    sys.exit(1 if t.failed else 0)


if __name__ == "__main__":
    main()
