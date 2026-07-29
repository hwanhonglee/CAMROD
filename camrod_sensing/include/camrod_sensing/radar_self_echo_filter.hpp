#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace camrod::sensing::radar_self_echo_filter
{

// HH_260729 - A fixed self-return exclusion is a narrow measured notch.
// Reject broader entries as likely unit/decimal mistakes so a readable config
// typo cannot create a large undetectable obstacle interval.
constexpr double kMaxNamedBandWidthM = 0.10;

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

// HH_260729 - Derive the operator-facing sensor name from the topic immediately
// above its range leaf (for example /radar/front1/range -> FRONT1). Keeping the
// name tied to input_topics prevents a readable calibration profile from
// silently drifting away from the subscription order.
inline bool sensorLabelFromTopic(
  const std::string & topic,
  std::string & label,
  std::string & error)
{
  label.clear();
  error.clear();

  std::size_t topic_end = topic.size();
  while (topic_end > 0U && topic[topic_end - 1U] == '/') {
    --topic_end;
  }
  if (topic_end == 0U) {
    error = "input topic is empty";
    return false;
  }

  const auto leaf_separator = topic.rfind('/', topic_end - 1U);
  if (leaf_separator == std::string::npos || leaf_separator == 0U) {
    error = "input topic must contain a sensor segment and a range leaf";
    return false;
  }

  std::size_t sensor_end = leaf_separator;
  while (sensor_end > 0U && topic[sensor_end - 1U] == '/') {
    --sensor_end;
  }
  if (sensor_end == 0U) {
    error = "input topic has no sensor segment";
    return false;
  }
  const auto sensor_separator = topic.rfind('/', sensor_end - 1U);
  const std::size_t sensor_begin =
    sensor_separator == std::string::npos ? 0U : sensor_separator + 1U;
  if (sensor_begin >= sensor_end) {
    error = "input topic has no sensor segment";
    return false;
  }

  label = topic.substr(sensor_begin, sensor_end - sensor_begin);
  for (char & character : label) {
    const auto value = static_cast<unsigned char>(character);
    if (!std::isalnum(value) && character != '_') {
      label.clear();
      error = "sensor topic segment must contain only letters, digits, or underscore";
      return false;
    }
    character = static_cast<char>(std::toupper(value));
  }
  return true;
}

// HH_260729 - Parse the preferred human-readable fixed profile. Each entry is
// SENSOR:min_m:max_m, where SENSOR is derived from input_topics. Any malformed
// or unknown entry clears the complete result so a typo cannot create a
// partial blind zone.
inline bool buildBandsFromSpecs(
  const std::vector<std::string> & specs,
  const std::vector<std::string> & input_topics,
  std::vector<Band> & bands,
  std::string & error)
{
  bands.clear();
  error.clear();

  std::vector<std::string> sensor_labels;
  sensor_labels.reserve(input_topics.size());
  for (std::size_t i = 0U; i < input_topics.size(); ++i) {
    std::string label;
    std::string label_error;
    if (!sensorLabelFromTopic(input_topics[i], label, label_error)) {
      error = "cannot derive sensor label for input topic " +
        std::to_string(i) + ": " + label_error;
      return false;
    }
    if (std::find(sensor_labels.begin(), sensor_labels.end(), label) !=
      sensor_labels.end())
    {
      error = "duplicate sensor label derived from input_topics: " + label;
      return false;
    }
    sensor_labels.push_back(std::move(label));
  }

  std::vector<std::int64_t> sensor_indices;
  std::vector<double> centers_m;
  std::vector<double> half_widths_m;
  sensor_indices.reserve(specs.size());
  centers_m.reserve(specs.size());
  half_widths_m.reserve(specs.size());

  for (std::size_t i = 0U; i < specs.size(); ++i) {
    const auto & spec = specs[i];
    if (spec.empty() ||
      std::any_of(spec.begin(), spec.end(), [](const char character) {
        return std::isspace(static_cast<unsigned char>(character)) != 0;
      }))
    {
      error = "self-echo band spec " + std::to_string(i) +
        " must be nonempty and contain no whitespace";
      return false;
    }

    const auto first_separator = spec.find(':');
    const auto second_separator =
      first_separator == std::string::npos ?
      std::string::npos : spec.find(':', first_separator + 1U);
    if (first_separator == std::string::npos ||
      second_separator == std::string::npos ||
      spec.find(':', second_separator + 1U) != std::string::npos ||
      first_separator == 0U ||
      second_separator == first_separator + 1U ||
      second_separator + 1U >= spec.size())
    {
      error = "self-echo band spec " + std::to_string(i) +
        " must use SENSOR:min_m:max_m";
      return false;
    }

    std::string label = spec.substr(0U, first_separator);
    for (char & character : label) {
      const auto value = static_cast<unsigned char>(character);
      if (!std::isalnum(value) && character != '_') {
        error = "self-echo band spec " + std::to_string(i) +
          " has an invalid sensor label";
        return false;
      }
      character = static_cast<char>(std::toupper(value));
    }
    const auto sensor = std::find(sensor_labels.begin(), sensor_labels.end(), label);
    if (sensor == sensor_labels.end()) {
      error = "self-echo band spec " + std::to_string(i) +
        " references unknown sensor " + label;
      return false;
    }

    double minimum_m = 0.0;
    double maximum_m = 0.0;
    try {
      std::size_t parsed = 0U;
      const std::string minimum_text =
        spec.substr(first_separator + 1U, second_separator - first_separator - 1U);
      minimum_m = std::stod(minimum_text, &parsed);
      if (parsed != minimum_text.size()) {
        error = "self-echo band spec " + std::to_string(i) +
          " has an invalid minimum range";
        return false;
      }
      const std::string maximum_text = spec.substr(second_separator + 1U);
      maximum_m = std::stod(maximum_text, &parsed);
      if (parsed != maximum_text.size()) {
        error = "self-echo band spec " + std::to_string(i) +
          " has an invalid maximum range";
        return false;
      }
    } catch (...) {
      error = "self-echo band spec " + std::to_string(i) +
        " has a non-numeric range";
      return false;
    }

    const double width_m = maximum_m - minimum_m;
    const double center_m = minimum_m + width_m * 0.5;
    if (!std::isfinite(minimum_m) || !std::isfinite(maximum_m) ||
      minimum_m <= 0.0 || maximum_m <= minimum_m ||
      !std::isfinite(width_m) || !std::isfinite(center_m))
    {
      error = "self-echo band spec " + std::to_string(i) +
        " requires finite ranges with 0 < min_m < max_m";
      return false;
    }
    if (width_m > kMaxNamedBandWidthM + 1e-9) {
      error = "self-echo band spec " + std::to_string(i) +
        " is wider than the 0.10 m fixed-return safety limit";
      return false;
    }

    sensor_indices.push_back(
      static_cast<std::int64_t>(std::distance(sensor_labels.begin(), sensor)));
    centers_m.push_back(center_m);
    half_widths_m.push_back(width_m * 0.5);
  }

  return buildBands(
    sensor_indices, centers_m, half_widths_m, input_topics.size(), bands, error);
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
