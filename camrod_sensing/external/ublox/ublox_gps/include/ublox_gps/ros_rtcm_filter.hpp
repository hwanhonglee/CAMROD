#ifndef UBLOX_GPS__ROS_RTCM_FILTER_HPP_
#define UBLOX_GPS__ROS_RTCM_FILTER_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace ublox_node {

// HH_260722 - Keep the ROS/NTRIP RTCM policy testable without a connected F9P.
inline bool extractRtcm3MessageType(
  const std::vector<uint8_t> & frame, uint16_t & message_type)
{
  if (frame.size() < 5 || frame[0] != 0xD3) {
    return false;
  }
  const uint16_t payload_length =
    (static_cast<uint16_t>(frame[1] & 0x03) << 8) | static_cast<uint16_t>(frame[2]);
  if (payload_length < 2 || frame.size() < static_cast<size_t>(payload_length) + 6) {
    return false;
  }
  message_type =
    (static_cast<uint16_t>(frame[3]) << 4) | (static_cast<uint16_t>(frame[4]) >> 4);
  return true;
}

// HH_260722 - This policy is used only for ROS-delivered RTCM. Direct UART2
// moving-base messages never enter this function and therefore remain intact.
inline bool shouldDropRosRtcm(
  const std::vector<uint8_t> & frame,
  bool dual_antenna,
  const std::vector<int64_t> & blocked_message_ids,
  uint16_t & message_type)
{
  message_type = 0;
  return dual_antenna &&
         extractRtcm3MessageType(frame, message_type) &&
         std::find(
           blocked_message_ids.begin(), blocked_message_ids.end(),
           static_cast<int64_t>(message_type)) != blocked_message_ids.end();
}

}  // namespace ublox_node

#endif  // UBLOX_GPS__ROS_RTCM_FILTER_HPP_
