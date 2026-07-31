from pathlib import Path
import unittest

import yaml

# HH_260723 - Guard the contiguous GNSS position/yaw inputs required by robot_localization.

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config" / "filter"


class EkfSensorInputTest(unittest.TestCase):
    def test_gnss_position_and_heading_inputs_are_contiguous(self):
        for config_name in ("ekf.yaml", "ekf_sim.yaml"):
            with self.subTest(config_name=config_name):
                self._assert_gnss_inputs(config_name)

    def test_real_ranger_wheel_input_includes_measured_yaw_rate(self):
        with (CONFIG_DIR / "ekf.yaml").open(encoding="utf-8") as stream:
            parameters = yaml.safe_load(stream)["/**"]["ros__parameters"]

        wheel_config = parameters["odom0_config"]
        self.assertTrue(wheel_config[6])
        self.assertTrue(wheel_config[7])
        # HH_260731 - Field rollback: wheel yaw-rate assists IMU yaw-rate and
        # carries non-zero driver covariance, so it is not fused as exact.
        self.assertTrue(wheel_config[11])

    def _assert_gnss_inputs(self, config_name):
        with (CONFIG_DIR / config_name).open(encoding="utf-8") as stream:
            parameters = yaml.safe_load(stream)["/**"]["ros__parameters"]

        pose_indices = sorted(
            int(name.removeprefix("pose"))
            for name in parameters
            if name.startswith("pose") and name.removeprefix("pose").isdigit()
        )
        self.assertEqual(pose_indices, list(range(len(pose_indices))))

        expected_topic = "/sensing/gnss/pose_with_covariance_ros"
        self.assertEqual(parameters["pose0"], expected_topic)
        self.assertEqual(parameters["pose1"], expected_topic)
        self.assertEqual(
            parameters["pose0_config"][:6],
            [True, True, True, False, False, False],
        )
        self.assertEqual(
            parameters["pose1_config"][:6],
            [False, False, False, False, False, True],
        )


if __name__ == "__main__":
    unittest.main()
