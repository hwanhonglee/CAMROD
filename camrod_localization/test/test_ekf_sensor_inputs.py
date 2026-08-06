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

    def test_real_filter_retains_delayed_measurements_at_controller_rate(self):
        """Cover delayed samples while matching the 20 Hz controller cadence."""
        with (CONFIG_DIR / "ekf.yaml").open(encoding="utf-8") as stream:
            parameters = yaml.safe_load(stream)["/**"]["ros__parameters"]

        # HH_260806 - Keep 1 Hz dual-GNSS independent from the 20 Hz prediction
        # loop; Jetson moving-load acceptance remains a field requirement.
        self.assertEqual(parameters["frequency"], 20.0)
        self.assertTrue(parameters["smooth_lagged_data"])
        self.assertTrue(parameters["predict_to_current_time"])
        self.assertGreaterEqual(parameters["history_length"], 0.75)

    def test_real_filter_configures_imu_and_wheel_prediction_sources(self):
        """Keep both high-rate prediction inputs active between 1 Hz GNSS fixes."""
        with (CONFIG_DIR / "ekf.yaml").open(encoding="utf-8") as stream:
            parameters = yaml.safe_load(stream)["/**"]["ros__parameters"]

        self.assertEqual(parameters["imu0"], "/sensing/imu/data_ros")
        self.assertEqual(
            parameters["odom0"], "/localization/input/wheel_odometry_ros"
        )
        # HH_260806 - The message config exposes xyz angular rates, but
        # robot_localization two_d_mode clamps roll/pitch and their rates.
        self.assertTrue(parameters["two_d_mode"])
        self.assertEqual(parameters["imu0_config"][3:6], [True, True, False])
        self.assertEqual(parameters["imu0_config"][9:12], [True, True, True])
        self.assertEqual(parameters["odom0_config"][6:9], [True, True, False])
        self.assertTrue(parameters["odom0_config"][11])

    def test_twenty_hz_change_preserves_ekf_weighting_contract(self):
        """Do not retune Q/R merely because the publication clock changed."""
        with (CONFIG_DIR / "ekf.yaml").open(encoding="utf-8") as stream:
            parameters = yaml.safe_load(stream)["/**"]["ros__parameters"]

        process_noise = parameters["process_noise_covariance"]
        diagonal = [process_noise[index * 15 + index] for index in range(15)]

        # HH_260806 - The bundled EKF multiplies Q by elapsed time. Preserve
        # measured weighting until one moving bag provides common residuals.
        self.assertTrue(parameters["two_d_mode"])
        self.assertEqual(parameters["pose0_rejection_threshold"], 3.0)
        self.assertEqual(parameters["pose1_rejection_threshold"], 1000.0)
        self.assertEqual(
            diagonal,
            [
                0.01, 0.01, 0.1, 0.1, 0.1, 0.1,
                0.05, 0.05, 0.05, 0.1, 0.1, 0.1,
                0.05, 0.05, 0.05,
            ],
        )

        ekf_source = (
            CONFIG_DIR.parent.parent
            / "external"
            / "robot_localization"
            / "src"
            / "ekf.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("delta_sec * (*process_noise_covariance)", ekf_source)

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
