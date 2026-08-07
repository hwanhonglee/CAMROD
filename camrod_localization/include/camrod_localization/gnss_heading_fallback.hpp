#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iterator>
#include <limits>
#include <optional>

namespace camrod_localization
{

enum class LeverArmHeadingSource
{
  kUnavailable,
  kFreshGnss,
  kEkfDelta,
};

struct LeverArmHeadingSelection
{
  LeverArmHeadingSource source{LeverArmHeadingSource::kUnavailable};
  double yaw{0.0};
  double anchor_age_s{std::numeric_limits<double>::infinity()};
  double sample_offset_s{std::numeric_limits<double>::infinity()};

  bool usable() const
  {
    return source != LeverArmHeadingSource::kUnavailable;
  }

  // HH_260807 - Only a fresh receiver heading may be exposed as usable yaw to
  // robot_localization.  EKF-delta fallback is solely an internal lever-arm aid.
  bool gnssYawUsable() const
  {
    return source == LeverArmHeadingSource::kFreshGnss;
  }
};

// HH_260807 - Keep a short time-indexed EKF-yaw history so a delayed NavSatFix
// can rotate the antenna lever arm at its own measurement epoch.  A fallback is
// impossible until a valid dual-GNSS heading anchors EKF yaw to map yaw.
class GnssHeadingFallback
{
public:
  void configure(double anchor_max_age_s, double history_s, double match_tolerance_s)
  {
    anchor_max_age_s_ = std::max(0.0, anchor_max_age_s);
    history_s_ = std::max(0.0, history_s);
    match_tolerance_s_ = std::max(0.0, match_tolerance_s);
    pruneHistory();
  }

  void recordEkfYaw(
    int64_t stamp_ns, double yaw, double yaw_covariance,
    double max_yaw_covariance)
  {
    if (!std::isfinite(yaw) || !std::isfinite(yaw_covariance) ||
      yaw_covariance <= 0.0 || !std::isfinite(max_yaw_covariance) ||
      max_yaw_covariance <= 0.0 || yaw_covariance > max_yaw_covariance)
    {
      return;
    }

    const TimedYaw sample{stamp_ns, normalizeYaw(yaw)};
    if (!history_.empty() && history_.back().stamp_ns == stamp_ns) {
      history_.back() = sample;
    } else if (history_.empty() || history_.back().stamp_ns < stamp_ns) {
      history_.push_back(sample);
    } else {
      const auto position = std::upper_bound(
        history_.begin(), history_.end(), stamp_ns,
        [](int64_t stamp, const TimedYaw & item) {return stamp < item.stamp_ns;});
      history_.insert(position, sample);
    }
    pruneHistory();
  }

  bool anchorWithValidGnss(int64_t stamp_ns, double gnss_yaw)
  {
    if (!std::isfinite(gnss_yaw)) {
      return false;
    }
    const auto matched = nearestEkfYaw(stamp_ns);
    if (!matched.has_value()) {
      return false;
    }

    anchor_.emplace(Anchor{
      stamp_ns,
      normalizeYaw(gnss_yaw),
      matched->sample.yaw,
    });
    return true;
  }

  LeverArmHeadingSelection select(
    int64_t fix_stamp_ns, const std::optional<double> & fresh_gnss_yaw) const
  {
    // A fresh receiver heading always wins, even when an EKF anchor is present.
    if (fresh_gnss_yaw.has_value() && std::isfinite(*fresh_gnss_yaw)) {
      return {
        LeverArmHeadingSource::kFreshGnss,
        normalizeYaw(*fresh_gnss_yaw),
        0.0,
        0.0,
      };
    }

    if (!anchor_.has_value()) {
      return {};
    }
    const double anchor_age_s = secondsBetween(anchor_->stamp_ns, fix_stamp_ns);
    if (anchor_age_s < 0.0 || anchor_age_s > anchor_max_age_s_) {
      return {};
    }

    const auto matched = nearestEkfYaw(fix_stamp_ns);
    if (!matched.has_value()) {
      return {};
    }

    const double ekf_delta = normalizeYaw(matched->sample.yaw - anchor_->ekf_yaw);
    return {
      LeverArmHeadingSource::kEkfDelta,
      normalizeYaw(anchor_->gnss_yaw + ekf_delta),
      anchor_age_s,
      matched->offset_s,
    };
  }

private:
  struct TimedYaw
  {
    int64_t stamp_ns{0};
    double yaw{0.0};
  };

  struct MatchedYaw
  {
    TimedYaw sample;
    double offset_s{0.0};
  };

  struct Anchor
  {
    int64_t stamp_ns{0};
    double gnss_yaw{0.0};
    double ekf_yaw{0.0};
  };

  static double normalizeYaw(double yaw)
  {
    return std::atan2(std::sin(yaw), std::cos(yaw));
  }

  static double secondsBetween(int64_t first_ns, int64_t second_ns)
  {
    return static_cast<double>(second_ns - first_ns) * 1.0e-9;
  }

  std::optional<MatchedYaw> nearestEkfYaw(int64_t stamp_ns) const
  {
    if (history_.empty()) {
      return std::nullopt;
    }

    const auto next = std::lower_bound(
      history_.begin(), history_.end(), stamp_ns,
      [](const TimedYaw & item, int64_t stamp) {return item.stamp_ns < stamp;});

    auto best = next;
    if (best == history_.end()) {
      best = std::prev(history_.end());
    }
    if (next != history_.begin()) {
      const auto previous = std::prev(next);
      if (best == history_.end() ||
        std::abs(stamp_ns - previous->stamp_ns) <= std::abs(best->stamp_ns - stamp_ns))
      {
        best = previous;
      }
    }

    const double offset_s = std::abs(secondsBetween(stamp_ns, best->stamp_ns));
    if (offset_s > match_tolerance_s_) {
      return std::nullopt;
    }
    return MatchedYaw{*best, offset_s};
  }

  void pruneHistory()
  {
    if (history_.empty()) {
      return;
    }
    const int64_t newest_stamp_ns = history_.back().stamp_ns;
    const int64_t history_ns = static_cast<int64_t>(history_s_ * 1.0e9);
    while (!history_.empty() && newest_stamp_ns - history_.front().stamp_ns > history_ns) {
      history_.pop_front();
    }
  }

  double anchor_max_age_s_{3.0};
  double history_s_{5.0};
  double match_tolerance_s_{0.2};
  std::deque<TimedYaw> history_;
  std::optional<Anchor> anchor_;
};

}  // namespace camrod_localization
