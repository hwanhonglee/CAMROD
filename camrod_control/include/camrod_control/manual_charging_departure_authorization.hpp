#pragma once

// CARLA can keep its simulated charger contact asserted until the Ranger has
// physically moved away.  Keep the narrowly scoped manual departure lease in
// a ROS-free policy so source ownership, expiry and revocation stay directly
// testable.  The production/default configuration is disabled.

#include <cmath>

namespace camrod_control
{

struct ManualChargingDepartureAuthorizationConfig
{
  bool enabled{false};
  double command_timeout_s{0.35};
};

struct ManualChargingDepartureContext
{
  bool dedicated_manual_input_configured{false};
  bool manual_source_active{false};
  bool maneuver_active{false};
  bool manual_engaged{false};
  bool mission_engaged{false};
  bool platform_drive_enabled{false};
  bool charging{false};
  bool battery_ready_for_departure{false};
};

inline bool manualChargingDepartureAuthorizationConfigIsValid(
  const ManualChargingDepartureAuthorizationConfig & config)
{
  return std::isfinite(config.command_timeout_s) &&
         config.command_timeout_s >= 0.10 &&
         config.command_timeout_s <= 1.00;
}

class ManualChargingDepartureAuthorization
{
public:
  explicit ManualChargingDepartureAuthorization(
    ManualChargingDepartureAuthorizationConfig config = {})
  : config_(config)
  {
  }

  void setConfig(const ManualChargingDepartureAuthorizationConfig & config)
  {
    config_ = config;
    reset();
  }

  bool observeAcceptedDedicatedManual(
    const double now_sec, const ManualChargingDepartureContext & context)
  {
    if (!eligible(now_sec, context)) {
      reset();
      return false;
    }
    active_until_sec_ = now_sec + config_.command_timeout_s;
    return true;
  }

  bool isActive(
    const double now_sec, const ManualChargingDepartureContext & context) const
  {
    return eligible(now_sec, context) && active_until_sec_ > now_sec;
  }

  double remainingSec(
    const double now_sec, const ManualChargingDepartureContext & context) const
  {
    return isActive(now_sec, context)
             ? active_until_sec_ - now_sec
             : 0.0;
  }

  void reset()
  {
    active_until_sec_ = 0.0;
  }

  const ManualChargingDepartureAuthorizationConfig & config() const
  {
    return config_;
  }

private:
  bool eligible(
    const double now_sec, const ManualChargingDepartureContext & context) const
  {
    return config_.enabled &&
           manualChargingDepartureAuthorizationConfigIsValid(config_) &&
           std::isfinite(now_sec) &&
           context.dedicated_manual_input_configured &&
           context.manual_source_active && !context.maneuver_active &&
           context.manual_engaged && !context.mission_engaged &&
           context.platform_drive_enabled && context.charging &&
           context.battery_ready_for_departure;
  }

  ManualChargingDepartureAuthorizationConfig config_;
  double active_until_sec_{0.0};
};

}  // namespace camrod_control
