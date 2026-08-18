#pragma once

// HH_260818 - Safety cost may be created only by a semantic detector result.
// Euclidean-only and malformed detections remain visible for diagnostics but
// cannot become an early path stop.

#include <algorithm>
#include <cctype>
#include <set>
#include <string>

namespace camrod_perception
{

inline std::string NormalizeClassLabel(std::string label)
{
  const auto first = label.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = label.find_last_not_of(" \t\r\n");
  label = label.substr(first, last - first + 1U);
  std::transform(
    label.begin(), label.end(), label.begin(),
    [](const unsigned char value) {return static_cast<char>(std::tolower(value));});
  return label;
}

inline std::string ResolveClassLabel(
  const std::string & detection_id,
  const std::string & hypothesis_class_id)
{
  return !detection_id.empty() ? detection_id : hypothesis_class_id;
}

inline bool IsClassifiedDetection(
  const std::string & label,
  const std::set<std::string> & unknown_labels)
{
  const auto normalized = NormalizeClassLabel(label);
  return !normalized.empty() && unknown_labels.count(normalized) == 0U;
}

}  // namespace camrod_perception
