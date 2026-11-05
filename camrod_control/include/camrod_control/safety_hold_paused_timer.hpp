#pragma once

// HJ_261105 - Phase timeouts measure only time the safety gate let the robot
// move. While the gate holds for an obstacle, the phase waits without a
// deadline; the gate/voice path keeps announcing the obstacle.

#include <algorithm>
#include <string>

namespace camrod_control {

// True when cmd_vel_safety_gate status reports an obstacle/cost or route
// safety hold. Other holds (estop, disengage, platform) keep timing.
inline bool gateStatusIndicatesObstacleHold(const std::string &operating_state,
                                            const std::string &message) {
  return operating_state == "ROUTE_SAFETY_HOLD" ||
         message.find("cost_stop_latched") != std::string::npos ||
         message.find("cost_hold=") != std::string::npos;
}

class SafetyHoldPausedTimer {
public:
  void restart(const double now_sec) {
    start_sec_ = now_sec;
    last_sec_ = now_sec;
    paused_s_ = 0.0;
  }

  // Attributes the interval since the previous observation to `held`.
  void observe(const double now_sec, const bool held) {
    if (held && now_sec > last_sec_) {
      paused_s_ += now_sec - last_sec_;
    }
    last_sec_ = std::max(last_sec_, now_sec);
  }

  double activeElapsed(const double now_sec) const {
    return std::max(0.0, now_sec - start_sec_ - paused_s_);
  }

  double pausedSeconds() const { return paused_s_; }

private:
  double start_sec_{0.0};
  double last_sec_{0.0};
  double paused_s_{0.0};
};

}  // namespace camrod_control
