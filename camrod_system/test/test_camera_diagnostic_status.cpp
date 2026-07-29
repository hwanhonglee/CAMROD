#include <cassert>
#include <iostream>

#include <camrod_system/camera_diagnostic_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

int main()
{
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

  {
    const auto result = camrod_system::evaluate_camera_diagnostic(
      DiagnosticStatus::WARN, false, false);
    assert(result.level == DiagnosticStatus::WARN);
    assert(result.message == "FPS low");
  }

  {
    const auto result = camrod_system::evaluate_camera_diagnostic(
      DiagnosticStatus::WARN, false, true);
    assert(result.level == DiagnosticStatus::WARN);
    assert(result.message == "FPS low");
  }

  {
    const auto result = camrod_system::evaluate_camera_diagnostic(
      DiagnosticStatus::OK, false, true);
    assert(result.level == DiagnosticStatus::WARN);
    assert(result.message == "Encoding mismatch");
  }

  {
    const auto result = camrod_system::evaluate_camera_diagnostic(
      DiagnosticStatus::ERROR, true, true);
    assert(result.level == DiagnosticStatus::ERROR);
    assert(result.message == "FPS critically low");
  }

  std::cout << "camera_diagnostic_status tests passed\n";
  return 0;
}
