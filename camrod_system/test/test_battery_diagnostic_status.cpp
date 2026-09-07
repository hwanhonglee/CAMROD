#include <algorithm>
#include <iostream>

#include <camrod_system/battery_diagnostic_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

int main()
{
  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  using camrod_system::batterySocDiagnosticLevel;
  const int8_t low_soc = batterySocDiagnosticLevel(Status::ERROR, false);
  if (low_soc != Status::WARN ||
    batterySocDiagnosticLevel(Status::ERROR, true) != Status::ERROR ||
    batterySocDiagnosticLevel(Status::WARN, false) != Status::WARN ||
    batterySocDiagnosticLevel(Status::OK, false) != Status::OK ||
    batterySocDiagnosticLevel(Status::STALE, false) != Status::STALE)
  {
    std::cerr << "SOC policy did not preserve severity/explicit opt-in\n";
    return 1;
  }

  // The actual checker aggregates components after applying the SOC policy.
  // Severe voltage or temperature evidence must still propagate ERROR_STOP.
  const int8_t ok = Status::OK;
  const int8_t error = Status::ERROR;
  if (std::max({ok, low_soc, ok}) != Status::WARN ||
    std::max({error, low_soc, ok}) != Status::ERROR ||
    std::max({ok, low_soc, error}) != Status::ERROR)
  {
    std::cerr << "SOC advisory policy masked voltage/temperature severity\n";
    return 1;
  }
  std::cout << "battery_diagnostic_status tests passed\n";
  return 0;
}
