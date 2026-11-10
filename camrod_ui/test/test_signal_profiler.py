"""HJ_261110 - SIGUSR1/SIGUSR2 main-thread profiler for ui_backend."""

import os
import signal
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime" / "python"))

from camrod_ui.signal_profiler import SignalProfiler  # noqa: E402


def _busy_function() -> int:
    return sum(index * index for index in range(20000))


class SignalProfilerTest(unittest.TestCase):
    def test_signals_start_and_write_cumulative_stats(self) -> None:
        previous = (signal.getsignal(signal.SIGUSR1), signal.getsignal(signal.SIGUSR2))
        try:
            with tempfile.TemporaryDirectory() as directory:
                output = Path(directory) / "profile.txt"
                messages = []
                profiler = SignalProfiler(str(output), log=messages.append)
                profiler.install()
                os.kill(os.getpid(), signal.SIGUSR1)
                self.assertTrue(profiler.active)
                _busy_function()
                os.kill(os.getpid(), signal.SIGUSR2)
                self.assertFalse(profiler.active)
                text = output.read_text()
                self.assertIn("_busy_function", text)
                self.assertIn("cumulative", text)
                self.assertEqual(len(messages), 2)
        finally:
            signal.signal(signal.SIGUSR1, previous[0])
            signal.signal(signal.SIGUSR2, previous[1])

    def test_stop_without_start_writes_nothing(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "profile.txt"
            self.assertIsNone(SignalProfiler(str(output)).stop())
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
