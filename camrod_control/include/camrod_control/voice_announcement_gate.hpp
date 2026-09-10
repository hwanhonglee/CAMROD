#pragma once

// HH_260910 - Hold a motion transition until its voice cue finishes playing,
// with a fail-open timeout so a stuck/crashed voice_announcer (or a launch
// profile that never brings camrod_voice up at all, e.g. sim/bench configs)
// can never strand the controller. ROS-free by design so the sequencing is
// testable without a full colcon workspace, matching the other policy
// headers in this package (see DropZoneChargingDepartureAuthorization).
//
// Unlike VoiceDepartureGate on the Python/mission-orchestration side, this
// gate never overrides a live safety condition: callers combine
// `releaseReady()` with their own unchanged safety checks (fresh pose,
// occupancy, cost stop, ...) rather than replacing them. It is meant for a
// controller phase that already exists specifically to let a spoken cue
// finish (e.g. RECALL_CLEARANCE_WAIT) — not for gating the real-time
// obstacle-hold release, which must stay driven by sensors alone.

#include <algorithm>
#include <string>
#include <utility>

namespace camrod_control
{

class VoiceAnnouncementGate
{
public:
  // Begin waiting for `key` to be confirmed finished playing. `timeout_s`
  // is measured from `now_s`; once it elapses, releaseReady() reports true
  // regardless of what voice_announcer ever reports (fail-open).
  void arm(std::string key, const double now_s, const double timeout_s)
  {
    key_ = std::move(key);
    armed_ = true;
    seen_playing_ = false;
    released_ = false;
    released_via_timeout_ = false;
    deadline_s_ = now_s + std::max(0.0, timeout_s);
  }

  void reset()
  {
    key_.clear();
    armed_ = false;
    seen_playing_ = false;
    released_ = false;
    released_via_timeout_ = false;
  }

  // Feed one avg_msgs/VoiceState sample (state==PLAYING, current_key).
  void onVoiceState(
    const bool playing, const std::string & current_key, const double /*now_s*/)
  {
    if (!armed_ || released_) {
      return;
    }
    if (!seen_playing_) {
      // The armed key must actually be seen playing first: an unrelated
      // higher-priority cue passing through must not falsely confirm this
      // one.
      if (playing && current_key == key_) {
        seen_playing_ = true;
      }
      return;
    }
    if (!playing) {
      // Seen playing, now stopped: it finished (or was interrupted, which is
      // an acceptable trade — waiting forever behind a stalled interrupt is
      // worse).
      released_ = true;
    }
  }

  // Call once per control-loop tick so a dead voice pipeline cannot block
  // the controller past `timeout_s`.
  void tick(const double now_s)
  {
    if (!armed_ || released_) {
      return;
    }
    if (now_s >= deadline_s_) {
      released_ = true;
      released_via_timeout_ = true;
    }
  }

  // True when nothing is armed (arm() was never called, or was reset), or
  // the armed key has finished playing, or the fail-open timeout elapsed.
  bool releaseReady() const { return !armed_ || released_; }

  // True only once, meaningfully, right after a release caused by the
  // timeout rather than a confirmed playback — useful for one warning log.
  bool releasedViaTimeout() const { return released_ && released_via_timeout_; }

  bool armed() const { return armed_; }

  const std::string & key() const { return key_; }

private:
  std::string key_;
  bool armed_{false};
  bool seen_playing_{false};
  bool released_{false};
  bool released_via_timeout_{false};
  double deadline_s_{0.0};
};

}  // namespace camrod_control
