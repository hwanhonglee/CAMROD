"""HJ_261118 - bringup 종료: ros2 launch 조상 찾기와 차단 조건."""

import os
import tempfile
import unittest

from camrod_ui.bringup_shutdown_api import (
    bringup_shutdown_blockers, find_launch_ancestor, is_ros2_launch_cmdline)


def _proc(root, pid, ppid, argv, comm="python3"):
    path = os.path.join(root, str(pid))
    os.makedirs(path)
    with open(os.path.join(path, "stat"), "w", encoding="utf-8") as handle:
        handle.write(f"{pid} ({comm}) S {ppid} 1 1 0 -1")
    with open(os.path.join(path, "cmdline"), "wb") as handle:
        handle.write(b"\0".join(part.encode() for part in argv) + b"\0")


class BringupShutdownTest(unittest.TestCase):
    def test_cmdline_matcher(self):
        self.assertTrue(is_ros2_launch_cmdline(
            ["/usr/bin/python3", "/opt/ros/humble/bin/ros2", "launch", "camrod_bringup", "bringup.launch.py"]))
        self.assertFalse(is_ros2_launch_cmdline(["/opt/ros/humble/bin/ros2", "run", "camrod_ui", "x"]))
        self.assertFalse(is_ros2_launch_cmdline(["bash", "launch"]))

    def test_finds_nearest_launch_ancestor_even_through_a_wrapper(self):
        with tempfile.TemporaryDirectory() as root:
            _proc(root, 10, 1, ["/usr/bin/python3", "/opt/ros/humble/bin/ros2", "launch", "camrod_bringup", "b.py"])
            _proc(root, 20, 10, ["/bin/sh", "-c", "ui_backend_node"], comm="s h (x)")
            _proc(root, 30, 20, ["/usr/bin/python3", "ui_backend_node"])
            self.assertEqual(find_launch_ancestor(30, proc_root=root), 10)

    def test_no_launch_ancestor_returns_zero(self):
        with tempfile.TemporaryDirectory() as root:
            _proc(root, 20, 1, ["bash"])
            _proc(root, 30, 20, ["/usr/bin/python3", "ui_backend_node"])
            self.assertEqual(find_launch_ancestor(30, proc_root=root), 0)

    def _motion(self, speed):
        return {"source_stamp_s": 100.0, "received_monotonic_s": 50.0, "speed_mps": speed,
                "yaw_rate_radps": 0.0, "estop": False, "error_code": 0, "vehicle_state": 0}

    def test_blockers(self):
        self.assertEqual(bringup_shutdown_blockers(False, self._motion(0.0), 100.1, 50.1), [])
        self.assertEqual(bringup_shutdown_blockers(True, self._motion(0.0), 100.1, 50.1), ["mission_active"])
        self.assertEqual(bringup_shutdown_blockers(False, self._motion(0.5), 100.1, 50.1),
                         ["robot_not_stationary"])
        # Missing/stale feedback must not trap the operator.
        self.assertEqual(bringup_shutdown_blockers(False, None, 100.1, 50.1), [])


if __name__ == "__main__":
    unittest.main()
