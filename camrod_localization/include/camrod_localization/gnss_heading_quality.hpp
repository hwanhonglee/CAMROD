#pragma once

#include <cmath>
#include <cstdint>

namespace camrod_localization
{

// HJ_261116 - Dual-antenna heading quality from NAV-RELPOSNED. The moving-baseline heading is only right with an
// RTK FIXED carrier solution: replays of the 2026-11-11 C-site bags show |GNSS - NDT| p95 1.9-2.9 deg when fixed but
// median 13-52 deg when float, while the receiver keeps relPosHeadingValid set and reports 3 deg accuracy. So the
// heading is trusted only when the latest RELPOSNED epoch is fresh, carrSoln == FIXED, relPosValid and
// relPosHeadingValid are set, the baseline length matches the mounted one, and that has held for settle_s.
// Kept free of ROS types so the rule can be unit-tested.
class GnssHeadingQuality
{
public:
  static constexpr uint32_t kRelPosValid = 4;
  static constexpr uint32_t kCarrSolnMask = 24;
  static constexpr uint32_t kCarrSolnFixed = 16;
  static constexpr uint32_t kRelPosHeadingValid = 256;

  void configure(double settle_s, double timeout_s, double baseline_m, double baseline_tol_m)
  {
    settle_ns_ = static_cast<int64_t>(settle_s * 1e9);
    timeout_ns_ = static_cast<int64_t>(timeout_s * 1e9);
    baseline_m_ = baseline_m;
    baseline_tol_m_ = baseline_tol_m;
  }

  // One RELPOSNED epoch. length_m = relPosLength + relPosHPLength.
  void update(int64_t now_ns, uint32_t flags, double length_m)
  {
    last_ns_ = now_ns;
    has_sample_ = true;
    if (epochFixed(flags, length_m)) {
      if (!fixed_) {
        fixed_since_ns_ = now_ns;
      }
      fixed_ = true;
    } else {
      fixed_ = false;
    }
  }

  bool epochFixed(uint32_t flags, double length_m) const
  {
    if ((flags & kCarrSolnMask) != kCarrSolnFixed || !(flags & kRelPosValid) || !(flags & kRelPosHeadingValid)) {
      return false;
    }
    if (baseline_m_ > 0.0 && !(std::isfinite(length_m) && std::abs(length_m - baseline_m_) <= baseline_tol_m_)) {
      return false;
    }
    return true;
  }

  // Heading trustworthy at now_ns: fresh, fixed, and fixed for at least settle_s.
  bool trusted(int64_t now_ns) const
  {
    return has_sample_ && fixed_ && (now_ns - last_ns_) <= timeout_ns_ &&
           (now_ns - fixed_since_ns_) >= settle_ns_;
  }

  bool hasSample() const {return has_sample_;}

private:
  int64_t settle_ns_{1000000000};
  int64_t timeout_ns_{1000000000};
  double baseline_m_{0.0};
  double baseline_tol_m_{0.05};
  bool has_sample_{false};
  bool fixed_{false};
  int64_t last_ns_{0};
  int64_t fixed_since_ns_{0};
};

}  // namespace camrod_localization
