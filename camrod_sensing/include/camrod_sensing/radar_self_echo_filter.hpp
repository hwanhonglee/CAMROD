#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace camrod::sensing::radar_self_echo_filter
{

struct Band
{
  std::size_t sensor_index{0U};
  double center_m{0.0};
  double half_width_m{0.0};
};

enum class StartupCalibrationAction
{
  kWait,
  kFinalizeSamples,
  kRejectNoTimelySample,
  kDone,
};

// HH_260728 - Give every radar a complete learning interval starting at its
// first valid sample. A separate absolute start timeout prevents a disconnected
// port from becoming eligible for learning much later in vehicle operation.
inline StartupCalibrationAction startupCalibrationAction(
  const bool finalized,
  const double first_sample_elapsed_s,
  const double now_elapsed_s,
  const double collection_duration_s,
  const double first_sample_timeout_s)
{
  if (finalized) {
    return StartupCalibrationAction::kDone;
  }
  const bool has_first_sample =
    std::isfinite(first_sample_elapsed_s) && first_sample_elapsed_s >= 0.0;
  const bool bounded_first_sample =
    std::isfinite(first_sample_timeout_s) && first_sample_timeout_s >= 0.0;
  // HH_260728 - A port first seen at/after the absolute deadline must never
  // reopen startup learning. This closes the late-connect/restart path where a
  // real obstacle could otherwise become a learned notch during operation.
  if (has_first_sample && bounded_first_sample &&
    first_sample_elapsed_s >= first_sample_timeout_s)
  {
    return StartupCalibrationAction::kRejectNoTimelySample;
  }
  if (!has_first_sample) {
    if (std::isfinite(now_elapsed_s) && bounded_first_sample &&
      now_elapsed_s >= first_sample_timeout_s)
    {
      return StartupCalibrationAction::kRejectNoTimelySample;
    }
    return StartupCalibrationAction::kWait;
  }
  if (!std::isfinite(now_elapsed_s) || now_elapsed_s < first_sample_elapsed_s) {
    return StartupCalibrationAction::kWait;
  }
  const double duration =
    std::isfinite(collection_duration_s) ? std::max(0.0, collection_duration_s) : 0.0;
  return now_elapsed_s - first_sample_elapsed_s >= duration ?
    StartupCalibrationAction::kFinalizeSamples :
    StartupCalibrationAction::kWait;
}

// HH_260728 - Convert the flat ROS parameter arrays into validated notch
// bands. Invalid configuration disables the complete filter so a malformed
// self-echo profile cannot silently hide real obstacles.
inline bool buildBands(
  const std::vector<std::int64_t> & sensor_indices,
  const std::vector<double> & centers_m,
  const std::vector<double> & half_widths_m,
  const std::size_t sensor_count,
  std::vector<Band> & bands,
  std::string & error)
{
  bands.clear();
  error.clear();

  if (sensor_indices.size() != centers_m.size() ||
    sensor_indices.size() != half_widths_m.size())
  {
    error = "self-echo parameter arrays must have equal lengths";
    return false;
  }

  std::vector<Band> validated;
  validated.reserve(sensor_indices.size());
  for (std::size_t i = 0; i < sensor_indices.size(); ++i) {
    const auto sensor_index = sensor_indices[i];
    const double center_m = centers_m[i];
    const double half_width_m = half_widths_m[i];
    if (sensor_index < 0 || static_cast<std::size_t>(sensor_index) >= sensor_count) {
      error = "self-echo sensor index is outside input_topics";
      return false;
    }
    if (!std::isfinite(center_m) || center_m <= 0.0) {
      error = "self-echo center must be finite and positive";
      return false;
    }
    if (!std::isfinite(half_width_m) || half_width_m <= 0.0) {
      error = "self-echo half width must be finite and positive";
      return false;
    }
    validated.push_back(
      Band{static_cast<std::size_t>(sensor_index), center_m, half_width_m});
  }

  bands = std::move(validated);
  return true;
}

inline bool matches(
  const std::size_t sensor_index,
  const double range_m,
  const std::vector<Band> & bands)
{
  if (!std::isfinite(range_m)) {
    return false;
  }
  for (const auto & band : bands) {
    if (band.sensor_index != sensor_index) {
      continue;
    }
    if (std::abs(range_m - band.center_m) <= band.half_width_m + 1e-9) {
      return true;
    }
  }
  return false;
}

// HH_260728 - Learn one dominant, tight startup cluster for a sensor. This is
// deliberately a notch rather than a lower-distance floor: other distances
// remain obstacles. Broad or weak clusters are rejected instead of creating
// an unsafe blind interval.
inline bool learnDominantBand(
  const std::size_t sensor_index,
  const std::vector<double> & samples,
  const std::size_t minimum_samples,
  const double cluster_gap_m,
  const double minimum_cluster_fraction,
  const double minimum_half_width_m,
  const double maximum_half_width_m,
  const double margin_m,
  Band & band,
  std::string & error)
{
  error.clear();
  if (minimum_samples == 0U || !std::isfinite(cluster_gap_m) || cluster_gap_m <= 0.0 ||
    !std::isfinite(minimum_cluster_fraction) || minimum_cluster_fraction <= 0.0 ||
    minimum_cluster_fraction > 1.0 ||
    !std::isfinite(minimum_half_width_m) || minimum_half_width_m <= 0.0 ||
    !std::isfinite(maximum_half_width_m) ||
    maximum_half_width_m < minimum_half_width_m ||
    !std::isfinite(margin_m) || margin_m < 0.0)
  {
    error = "invalid startup self-echo learning parameters";
    return false;
  }

  std::vector<double> sorted;
  sorted.reserve(samples.size());
  for (const double sample : samples) {
    if (std::isfinite(sample) && sample > 0.0) {
      sorted.push_back(sample);
    }
  }
  if (sorted.size() < minimum_samples) {
    error = "not enough valid startup samples";
    return false;
  }
  std::sort(sorted.begin(), sorted.end());

  struct Cluster
  {
    std::size_t begin{0U};
    std::size_t end{0U};
  };
  std::vector<Cluster> clusters;
  std::size_t begin = 0U;
  for (std::size_t i = 1U; i < sorted.size(); ++i) {
    if (sorted[i] - sorted[i - 1U] > cluster_gap_m) {
      clusters.push_back(Cluster{begin, i});
      begin = i;
    }
  }
  clusters.push_back(Cluster{begin, sorted.size()});

  const auto dominant = std::max_element(
    clusters.begin(), clusters.end(), [](const Cluster & lhs, const Cluster & rhs) {
      return (lhs.end - lhs.begin) < (rhs.end - rhs.begin);
    });
  const std::size_t cluster_size = dominant->end - dominant->begin;
  if (cluster_size < minimum_samples ||
    static_cast<double>(cluster_size) /
    static_cast<double>(sorted.size()) < minimum_cluster_fraction)
  {
    error = "no dominant startup self-echo cluster";
    return false;
  }

  const auto valueAtFraction = [&](const double fraction) {
      const auto offset = static_cast<std::size_t>(
        std::round(fraction * static_cast<double>(cluster_size - 1U)));
      return sorted[dominant->begin + std::min(offset, cluster_size - 1U)];
    };
  const double center_m = valueAtFraction(0.50);
  const double lower_m = valueAtFraction(0.05);
  const double upper_m = valueAtFraction(0.95);
  const double required_half_width_m =
    std::max(center_m - lower_m, upper_m - center_m) + margin_m;
  if (required_half_width_m > maximum_half_width_m) {
    error = "dominant startup cluster is too broad";
    return false;
  }

  band = Band{
    sensor_index, center_m, std::max(minimum_half_width_m, required_half_width_m)};
  return true;
}

}  // namespace camrod::sensing::radar_self_echo_filter
