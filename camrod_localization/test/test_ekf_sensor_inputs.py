from pathlib import Path
import unittest

import yaml


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config" / "filter"


class EkfSensorInputTest(unittest.TestCase):
    def test_gnss_position_and_heading_inputs_are_contiguous(self):
        for config_name in ("ekf.yaml", "ekf_sim.yaml"):
            with self.subTest(config_name=config_name):
                self._assert_gnss_inputs(config_name)

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
