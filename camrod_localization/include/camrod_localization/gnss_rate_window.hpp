#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>

namespace camrod_localization
{

// HH_260807 - Estimate GNSS cadence over unique measurement epochs instead of
// treating one delayed/missed 5 Hz epoch as an immediate localization failure.
class GnssRateWindow
{
public:
  static constexpr std::size_t kMinimumSamples = 3;

  explicit GnssRateWindow(double window_seconds = 2.0)
  {
    setWindowSeconds(window_seconds);
  }

  void setWindowSeconds(double window_seconds)
  {
    const double valid_window =
      std::isfinite(window_seconds) && window_seconds > 0.0 ? window_seconds : 2.0;
    window_nanoseconds_ = static_cast<std::int64_t>(valid_window * 1.0e9);
    stamps_nanoseconds_.clear();
  }

  // Returns true only when a new, monotonically increasing GNSS epoch was
  // recorded. Pose and pose-with-covariance mirrors share a stamp and therefore
  // contribute only once.
  bool record(std::int64_t stamp_nanoseconds)
  {
    if (!stamps_nanoseconds_.empty() && stamp_nanoseconds <= stamps_nanoseconds_.back()) {
      return false;
    }

    stamps_nanoseconds_.push_back(stamp_nanoseconds);
    while (stamps_nanoseconds_.size() > 1 &&
      stamp_nanoseconds - stamps_nanoseconds_.front() > window_nanoseconds_)
    {
      stamps_nanoseconds_.pop_front();
    }
    return true;
  }

  bool ready() const
  {
    return stamps_nanoseconds_.size() >= kMinimumSamples;
  }

  double rateHz() const
  {
    if (stamps_nanoseconds_.size() < 2) {
      return 0.0;
    }
    const double span_seconds =
      static_cast<double>(stamps_nanoseconds_.back() - stamps_nanoseconds_.front()) / 1.0e9;
    if (span_seconds <= 0.0) {
      return 0.0;
    }
    return static_cast<double>(stamps_nanoseconds_.size() - 1) / span_seconds;
  }

  std::size_t sampleCount() const
  {
    return stamps_nanoseconds_.size();
  }

private:
  std::int64_t window_nanoseconds_{2'000'000'000};
  std::deque<std::int64_t> stamps_nanoseconds_;
};

}  // namespace camrod_localization
