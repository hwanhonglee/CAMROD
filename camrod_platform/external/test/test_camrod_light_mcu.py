#!/usr/bin/env python3
"""Protocol test for the CAMROD exterior light MCU (camrod_light_mcu.ino).

Exercises the serial contract the firmware implements:

    PC  -> MCU @ 5 Hz : {"h":0|1,"i":"O"|"L"|"R"|"H","q":<seq>}
    MCU -> PC  @ 1 Hz : {"a":<last q>,"h":0|1,"i":"<mode>","e":<error>}

It drives the MCU through each indicator mode + headlight state, reads the
acks back, and checks the MCU echoes what we commanded. It also verifies the
link-loss behaviour: stop sending and the MCU must drive the strips dark and
report error == 2 (ERR_LINK_LOST) in the ack.

Default port is the fixed udev symlink /dev/camrod_light_mcu.

Usage:
    python3 test_camrod_light_mcu.py
    python3 test_camrod_light_mcu.py --port /dev/ttyUSB5
    python3 test_camrod_light_mcu.py --hold 3     # seconds per mode (watch LEDs)
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
SEND_HZ = 5                       # PC -> MCU rate the firmware expects
SEND_PERIOD = 1.0 / SEND_HZ
LINK_TIMEOUT_S = 1.0             # LINK_TIMEOUT_MS in the firmware
ERR_NONE, ERR_JSON, ERR_LINK_LOST = 0, 1, 2

# ANSI colours for a readable PASS/FAIL log.
GREEN, RED, DIM, RESET = "\033[32m", "\033[31m", "\033[2m", "\033[0m"


class Tester:
    def __init__(self, port, verbose):
        self.ser = serial.Serial(port, BAUD, timeout=0.2)
        self.seq = 0
        self.verbose = verbose
        self.passed = 0
        self.failed = 0
        # Arduino auto-resets when the port opens; wait for the bootloader +
        # setup() before the first frame, and drop the boot noise.
        time.sleep(2.0)
        self.ser.reset_input_buffer()

    def send(self, headlight, indicator):
        """Send one command frame, return the seq used."""
        self.seq += 1
        frame = {"h": 1 if headlight else 0, "i": indicator, "q": self.seq}
        self.ser.write((json.dumps(frame, separators=(",", ":")) + "\n").encode())
        self.ser.flush()
        return self.seq

    def read_ack(self, deadline):
        """Read lines until a parseable ack arrives or `deadline` passes."""
        while time.time() < deadline:
            raw = self.ser.readline()
            if not raw:
                continue
            try:
                ack = json.loads(raw.decode(errors="replace").strip())
            except (json.JSONDecodeError, ValueError):
                continue
            if isinstance(ack, dict) and "i" in ack and "e" in ack:
                if self.verbose:
                    print(f"{DIM}    <- {ack}{RESET}")
                return ack
        return None

    def check(self, label, condition, detail=""):
        tag = f"{GREEN}PASS{RESET}" if condition else f"{RED}FAIL{RESET}"
        if condition:
            self.passed += 1
        else:
            self.failed += 1
        print(f"  [{tag}] {label}{(' - ' + detail) if detail else ''}")
        return condition

    def drive(self, headlight, indicator, hold_s):
        """Stream commands at 5 Hz for hold_s, return the latest ack seen.

        Acks arrive at 1 Hz, so most 0.2 s send windows see none; keep the most
        recent non-empty ack instead of overwriting it with the last window.
        """
        last_ack = None
        last_seq = 0
        end = time.time() + hold_s
        while time.time() < end:
            last_seq = self.send(headlight, indicator)
            ack = self.read_ack(time.time() + SEND_PERIOD)
            if ack is not None:
                last_ack = ack
        return last_ack, last_seq

    def test_mode(self, name, headlight, indicator, hold_s):
        print(f"\n{name}: send h={int(headlight)} i={indicator!r}")
        ack, last_seq = self.drive(headlight, indicator, hold_s)
        if not self.check(f"{name}: ack received", ack is not None):
            return
        self.check(f"{name}: indicator echo == {indicator!r}",
                   ack.get("i") == indicator, f"got {ack.get('i')!r}")
        self.check(f"{name}: headlight echo == {int(headlight)}",
                   ack.get("h") == (1 if headlight else 0), f"got {ack.get('h')}")
        self.check(f"{name}: error == NONE", ack.get("e") == ERR_NONE,
                   f"got e={ack.get('e')}")
        # Ack should reference a recently sent sequence (within a short window).
        a = ack.get("a", -1)
        self.check(f"{name}: ack seq tracks ours", 0 < a <= last_seq,
                   f"a={a}, last_seq={last_seq}")

    def test_failsafe(self):
        print(f"\nFAILSAFE: stop sending, expect strips DARK + ERR_LINK_LOST")
        # Make sure the link is up first.
        self.drive(False, "O", 1.0)
        # Go silent and wait past the timeout. The MCU keeps sending acks at 1 Hz
        # during the lapse, so flush those queued (still e=0) frames and read the
        # NEXT fresh ack, which is generated after link_lost latches.
        time.sleep(LINK_TIMEOUT_S + 0.5)
        self.ser.reset_input_buffer()
        ack = self.read_ack(time.time() + 2.0)
        if not self.check("FAILSAFE: ack still received while silent",
                          ack is not None):
            return
        # Primary serial-observable signal: the firmware raises ERR_LINK_LOST and
        # drives the strips dark (indicators are command-only; there is no
        # autonomous hazard blink). Note the ack's "i" echoes the
        # last *commanded* mode (firmware sends commanded_mode, not active_mode),
        # so it stays at whatever was last sent rather than reporting "H".
        self.check("FAILSAFE: error == LINK_LOST", ack.get("e") == ERR_LINK_LOST,
                   f"got e={ack.get('e')}")
        print(f"{DIM}    (note: LEDs are dark; ack 'i' echoes last "
              f"commanded mode = {ack.get('i')!r}){RESET}")

    def test_recovery(self):
        print(f"\nRECOVERY: resume sending, expect error clears")
        ack, _ = self.drive(False, "O", 1.5)
        if not self.check("RECOVERY: ack received", ack is not None):
            return
        self.check("RECOVERY: error back to NONE", ack.get("e") == ERR_NONE,
                   f"got e={ack.get('e')}")
        self.check("RECOVERY: mode == OFF", ack.get("i") == "O",
                   f"got {ack.get('i')!r}")

    def visual_demo(self, hold_s, confirm):
        """Walk the indicator/hazard modes slowly so a human can watch the
        strips blink. Blink is generated on the MCU (~1.25 Hz), so this is a
        visual check: the script only selects the mode and keeps the link alive.
        """
        steps = [
            ("OFF",    "O", "both strips DARK (no blink)"),
            ("LEFT",   "L", "LEFT strip BLINKS ~1.25 Hz, RIGHT dark"),
            ("RIGHT",  "R", "RIGHT strip BLINKS ~1.25 Hz, LEFT dark"),
            ("HAZARD", "H", "BOTH strips BLINK together ~1.25 Hz"),
        ]
        print(f"\n{'='*52}\n VISUAL CHECK - watch the LED strips\n{'='*52}")
        all_ok = True
        for name, ind, expect in steps:
            print(f"\n▶ {name}: expect -> {expect}")
            print(f"{DIM}  holding {hold_s:.0f}s ...{RESET}")
            self.drive(False, ind, hold_s)
            if confirm:
                ans = input(f"  Did you see it? [Y/n] ").strip().lower()
                ok = ans in ("", "y", "yes")
                self.check(f"{name}: visual confirm", ok)
                all_ok = all_ok and ok
        # Leave the strips dark on exit.
        self.drive(False, "O", 0.5)
        if not confirm:
            print(f"\n{DIM}(no --confirm: watched only, nothing asserted){RESET}")
        return all_ok

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/camrod_light_mcu",
                    help="serial port (default: /dev/camrod_light_mcu)")
    ap.add_argument("--hold", type=float, default=2.0,
                    help="seconds to hold each mode (default: 2.0)")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="print every ack frame")
    args = ap.parse_args()

    print(f"Opening {args.port} @ {BAUD} ...")
    try:
        t = Tester(args.port, args.verbose)
    except serial.SerialException as e:
        sys.exit(f"{RED}Cannot open {args.port}: {e}{RESET}")

    try:
        # Indicator modes (watch the strips during each `hold`).
        t.test_mode("OFF",   False, "O", args.hold)
        t.test_mode("LEFT",  False, "L", args.hold)
        t.test_mode("RIGHT", False, "R", args.hold)
        t.test_mode("HAZARD", False, "H", args.hold)
        # Headlight relay on + left indicator together.
        t.test_mode("HEADLIGHT+LEFT", True, "L", args.hold)
        # Failsafe + recovery.
        t.test_failsafe()
        t.test_recovery()
    finally:
        t.close()

    print(f"\n{'='*40}")
    total = t.passed + t.failed
    colour = GREEN if t.failed == 0 else RED
    print(f"{colour}RESULT: {t.passed}/{total} checks passed, "
          f"{t.failed} failed{RESET}")
    sys.exit(1 if t.failed else 0)


if __name__ == "__main__":
    main()
