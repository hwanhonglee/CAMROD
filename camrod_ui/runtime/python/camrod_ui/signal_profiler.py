"""HJ_261110 - On-demand cProfile of the backend ROS main thread.

py-spy cannot attach on the robot (ptrace is restricted), so the backend
profiles itself: ``kill -USR1 <pid>`` starts, ``kill -USR2 <pid>`` stops and
writes cumulative-time stats. Signal handlers run on the main thread, which is
the thread running ``rclpy.spin``, so that is the thread being profiled.
Idle cost is zero; nothing is profiled until SIGUSR1.
"""

from __future__ import annotations

import cProfile
import io
import pstats
import signal
import time
from typing import Callable, Optional

DEFAULT_OUTPUT = "/tmp/ui_backend_profile.txt"


class SignalProfiler:
    def __init__(self, output_path: str = DEFAULT_OUTPUT, top: int = 60,
                 log: Optional[Callable[[str], None]] = None) -> None:
        self.output_path = output_path
        self.top = top
        self._log = log or (lambda _message: None)
        self._profile: Optional[cProfile.Profile] = None
        self._started = 0.0

    @property
    def active(self) -> bool:
        return self._profile is not None

    def start(self) -> None:
        if self._profile is not None:
            return
        self._profile = cProfile.Profile()
        self._started = time.monotonic()
        self._profile.enable()
        self._log(f"profiling started; send SIGUSR2 to write {self.output_path}")

    def stop(self) -> Optional[str]:
        profile, self._profile = self._profile, None
        if profile is None:
            return None
        profile.disable()
        stream = io.StringIO()
        stream.write(f"ui_backend main-thread profile, {time.monotonic() - self._started:.1f} s\n")
        pstats.Stats(profile, stream=stream).sort_stats("cumulative").print_stats(self.top)
        with open(self.output_path, "w", encoding="utf-8") as handle:
            handle.write(stream.getvalue())
        self._log(f"profile written to {self.output_path}")
        return self.output_path

    def install(self) -> None:
        signal.signal(signal.SIGUSR1, lambda _signum, _frame: self.start())
        signal.signal(signal.SIGUSR2, lambda _signum, _frame: self.stop())
