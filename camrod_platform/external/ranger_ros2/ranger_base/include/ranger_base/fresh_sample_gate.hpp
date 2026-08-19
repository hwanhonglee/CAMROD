// Copyright 2026 hwanhonglee
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the copyright holder nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef RANGER_BASE__FRESH_SAMPLE_GATE_HPP_
#define RANGER_BASE__FRESH_SAMPLE_GATE_HPP_

// HH_260819 - A Ranger feedback loop runs at 50 Hz, but BMS CAN 0x361 has its
// own sensor cadence.  Gate cached SDK state by the timestamp updated in the
// decoder so one physical frame produces exactly one ROS BatteryState sample.

namespace westonrobot
{

template<typename StampT>
class FreshSampleGate
{
public:
  bool accept(const StampT & stamp)
  {
    if (stamp == StampT{} || (initialized_ && stamp == last_stamp_)) {
      return false;
    }
    initialized_ = true;
    last_stamp_ = stamp;
    return true;
  }

  void reset()
  {
    initialized_ = false;
    last_stamp_ = StampT{};
  }

private:
  bool initialized_{false};
  StampT last_stamp_{};
};

}  // namespace westonrobot

#endif  // RANGER_BASE__FRESH_SAMPLE_GATE_HPP_
