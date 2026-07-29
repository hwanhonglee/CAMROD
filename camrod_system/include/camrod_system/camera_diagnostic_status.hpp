#pragma once

#include <cstdint>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace camrod_system
{

struct CameraDiagnosticStatus
{
  int8_t level{diagnostic_msgs::msg::DiagnosticStatus::OK};
  std::string message{"OK"};
};

inline void raise_camera_diagnostic(
  CameraDiagnosticStatus & status,
  int8_t candidate_level,
  const std::string & candidate_message)
{
  // Preserve the first (higher-priority) reason when two checks have the same
  // severity. In particular, a matching encoding must never rename FPS WARN.
  if (candidate_level > status.level) {
    status.level = candidate_level;
    status.message = candidate_message;
  }
}

inline CameraDiagnosticStatus evaluate_camera_diagnostic(
  int8_t fps_level,
  bool resolution_mismatch,
  bool encoding_mismatch)
{
  CameraDiagnosticStatus status;

  if (fps_level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
    raise_camera_diagnostic(
      status, fps_level, "FPS critically low");
  } else if (fps_level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
    raise_camera_diagnostic(status, fps_level, "FPS low");
  }

  if (resolution_mismatch) {
    raise_camera_diagnostic(
      status, diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Resolution mismatch");
  }
  if (encoding_mismatch) {
    raise_camera_diagnostic(
      status, diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Encoding mismatch");
  }

  return status;
}

}  // namespace camrod_system
