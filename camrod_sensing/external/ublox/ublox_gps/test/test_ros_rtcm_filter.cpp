#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "ublox_gps/ros_rtcm_filter.hpp"

namespace
{

// HH_260722 - Build the minimum structurally valid RTCM3 frame needed to test
// message-id routing; CRC contents are irrelevant to the driver's id filter.
std::vector<uint8_t> makeRtcm3Frame(uint16_t message_type)
{
  return {
    0xD3, 0x00, 0x02,
    static_cast<uint8_t>(message_type >> 4),
    static_cast<uint8_t>((message_type & 0x0F) << 4),
    0x00, 0x00, 0x00};
}

TEST(RosRtcmFilter, DropsConfiguredMovingBaseMessageOnRosInput)
{
  uint16_t parsed_type = 0;
  EXPECT_TRUE(ublox_node::shouldDropRosRtcm(
      makeRtcm3Frame(4072), true, {4072}, parsed_type));
  EXPECT_EQ(parsed_type, 4072);
}

TEST(RosRtcmFilter, AllowsCorsReferenceAndMsmMessages)
{
  // HH_260722 - Preserve external CORS station, MSM4, and GLONASS bias data.
  const std::vector<uint16_t> allowed_types = {
    1005, 1006, 1074, 1084, 1094, 1124, 1230};
  for (const uint16_t message_type : allowed_types) {
    uint16_t parsed_type = 0;
    EXPECT_FALSE(ublox_node::shouldDropRosRtcm(
        makeRtcm3Frame(message_type), true, {4072}, parsed_type));
    EXPECT_EQ(parsed_type, message_type);
  }
}

TEST(RosRtcmFilter, LeavesSingleAntennaRosInputUnfiltered)
{
  uint16_t parsed_type = 0;
  EXPECT_FALSE(ublox_node::shouldDropRosRtcm(
      makeRtcm3Frame(4072), false, {4072}, parsed_type));
  EXPECT_EQ(parsed_type, 0);
}

TEST(RosRtcmFilter, AllowsMalformedInputToReachExistingDriverHandling)
{
  uint16_t parsed_type = 99;
  EXPECT_FALSE(ublox_node::shouldDropRosRtcm(
      {0xD3, 0x00}, true, {4072}, parsed_type));
  EXPECT_EQ(parsed_type, 0);
}

}  // namespace
