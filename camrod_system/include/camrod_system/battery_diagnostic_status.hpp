#pragma once

#include <cstdint>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace camrod_system
{

inline int8_t batterySocDiagnosticLevel(const int8_t level, const bool error_enabled)
{
  // SOC alone must not raise planning ERROR_STOP during a low-battery return.
  // Apply this only to the SOC component before taking max(voltage, SOC, temp).
  // A STALE input stays STALE. Voltage/temperature and hardware-fault diagnostics
  // do not pass through this SOC-only helper and retain their original severity.
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  return !error_enabled && level == Status::ERROR ? Status::WARN : level;
}

}  // namespace camrod_system
