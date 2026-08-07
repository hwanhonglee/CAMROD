from pathlib import Path
import unittest


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "localization_input_adapter_node.cpp"
).read_text(encoding="utf-8")


class GnssHeadingCacheContractTest(unittest.TestCase):
    def test_unavailable_heading_cannot_replace_latest_valid_sample(self):
        """Reject a high-covariance placeholder before updating either cache."""
        callback = SOURCE.split("void onGnssHeading", maxsplit=1)[1].split(
            "bool selectHeading", maxsplit=1
        )[0]

        rejection = callback.index("yaw_cov > gnss_heading_max_covariance_")
        rejection_return = callback.index("return;", rejection)
        current_heading_update = callback.index("gnss_heading_ = candidate;")
        last_heading_update = callback.index("rememberHeading(candidate);")
        fallback_anchor_update = callback.index("anchorWithValidGnss")

        # HH_260807 - RELPOSNED can interleave a covariance=1000 unavailable
        # placeholder after a valid heading. Both heading caches must retain the
        # last valid sample so strict lever-arm correction keeps accepting fixes.
        self.assertLess(rejection, rejection_return)
        self.assertLess(rejection_return, current_heading_update)
        self.assertLess(rejection_return, last_heading_update)
        self.assertLess(rejection_return, fallback_anchor_update)

    def test_ekf_fallback_never_promotes_published_gnss_yaw(self):
        """Lever fallback must not affect the receiver-heading validity bit."""
        callback = SOURCE.split("void onNavSatFix", maxsplit=1)[1].split(
            "void onUtmPose", maxsplit=1
        )[0]

        fresh_selection = callback.index("const bool has_fresh_heading")
        lever_selection = callback.index("gnss_heading_fallback_.select")
        yaw_covariance_gate = callback.index(
            "pose_cov.pose.covariance[35] = has_fresh_heading"
        )

        # HH_260807 - The EKF-derived yaw rotates only the 0.45 m lever arm.
        # Published orientation remains the display/receiver selection, while
        # covariance stays unavailable unless selectHeading returned fresh GNSS.
        self.assertLess(fresh_selection, lever_selection)
        self.assertLess(lever_selection, yaw_covariance_gate)
        self.assertNotIn(
            "heading_orientation = lever_heading",
            callback[lever_selection:yaw_covariance_gate],
        )


if __name__ == "__main__":
    unittest.main()
