#include "camrod_sensing/camera_front_contract.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace contract = camrod::sensing::camera_front_contract;

namespace
{

std::vector<std::uint8_t> structurallyValidJpeg()
{
  return {
    0xFF, 0xD8,
    0xFF, 0xC0, 0x00, 0x0B,
    0x08, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x11, 0x00,
    0xFF, 0xDA, 0x00, 0x08,
    0x01, 0x01, 0x00, 0x00, 0x3F, 0x00,
    0x2A,
    0xFF, 0xD9,
  };
}

}  // namespace

TEST(CameraFrontFrameContract, AcceptsExpectedBgrAndNv12Shapes)
{
  EXPECT_EQ(
    contract::validateBgrFrameShape(true, 1080, 1920, 3, 1U, 5760U, 1920, 1080),
    contract::FrameShapeStatus::kValid);
  EXPECT_EQ(
    contract::validateNv12FrameShape(true, 1620, 1920, 1, 1U, 1920U, 1920, 1080),
    contract::FrameShapeStatus::kValid);
}

TEST(CameraFrontFrameContract, RejectsUnsafeFrameShapes)
{
  EXPECT_EQ(
    contract::validateNv12FrameShape(false, 1620, 1920, 1, 1U, 1920U, 1920, 1080),
    contract::FrameShapeStatus::kNoData);
  EXPECT_EQ(
    contract::validateNv12FrameShape(true, 1620, 1280, 1, 1U, 1920U, 1920, 1080),
    contract::FrameShapeStatus::kUnexpectedWidth);
  EXPECT_EQ(
    contract::validateNv12FrameShape(true, 1080, 1920, 1, 1U, 1920U, 1920, 1080),
    contract::FrameShapeStatus::kUnexpectedHeight);
  EXPECT_EQ(
    contract::validateNv12FrameShape(true, 1620, 1920, 3, 1U, 5760U, 1920, 1080),
    contract::FrameShapeStatus::kUnexpectedChannels);
  EXPECT_EQ(
    contract::validateNv12FrameShape(true, 1620, 1920, 1, 2U, 3840U, 1920, 1080),
    contract::FrameShapeStatus::kUnexpectedElementSize);
  EXPECT_EQ(
    contract::validateNv12FrameShape(true, 1620, 1920, 1, 1U, 1919U, 1920, 1080),
    contract::FrameShapeStatus::kInsufficientRowStride);
  EXPECT_EQ(
    contract::validateNv12FrameShape(true, 719, 1280, 1, 1U, 1280U, 1280, 479),
    contract::FrameShapeStatus::kInvalidConfiguredDimensions);
  EXPECT_EQ(
    contract::validateNv12FrameShape(
      true, 1, 2, 1, 1U, 2U, 2, std::numeric_limits<int>::max() - 1),
    contract::FrameShapeStatus::kInvalidConfiguredDimensions);
}

TEST(CameraFrontJpegContract, ComputesBoundedPayloadLimit)
{
  EXPECT_EQ(
    contract::maxJpegPayloadBytes(1920, 1080),
    1920U * 1080U * contract::kMaxJpegBytesPerPixel +
    contract::kJpegHeaderAllowanceBytes);
  EXPECT_EQ(contract::maxJpegPayloadBytes(0, 1080), 0U);
  EXPECT_EQ(contract::maxJpegPayloadBytes(1919, 1080), 0U);
  EXPECT_EQ(
    contract::maxJpegPayloadBytes(
      std::numeric_limits<int>::max() - 1,
      std::numeric_limits<int>::max() - 1),
    0U);
}

TEST(CameraFrontJpegContract, AcceptsJpegEnvelopeWithFrameAndScan)
{
  const auto jpeg = structurallyValidJpeg();
  EXPECT_EQ(
    contract::validateJpegPayload(jpeg.data(), jpeg.size(), jpeg.size()),
    contract::JpegPayloadStatus::kValid);
}

TEST(CameraFrontJpegContract, RejectsEmptyOversizedAndInvalidEnvelopes)
{
  const auto jpeg = structurallyValidJpeg();
  EXPECT_EQ(
    contract::validateJpegPayload(nullptr, 0U, jpeg.size()),
    contract::JpegPayloadStatus::kEmpty);
  EXPECT_EQ(
    contract::validateJpegPayload(jpeg.data(), jpeg.size(), jpeg.size() - 1U),
    contract::JpegPayloadStatus::kTooLarge);

  auto invalid = jpeg;
  invalid[0] = 0x00;
  EXPECT_EQ(
    contract::validateJpegPayload(invalid.data(), invalid.size(), invalid.size()),
    contract::JpegPayloadStatus::kMissingStartOfImage);

  invalid = jpeg;
  invalid.back() = 0x00;
  EXPECT_EQ(
    contract::validateJpegPayload(invalid.data(), invalid.size(), invalid.size()),
    contract::JpegPayloadStatus::kMissingEndOfImage);

  invalid = jpeg;
  invalid[4] = 0x00;
  invalid[5] = 0x01;
  EXPECT_EQ(
    contract::validateJpegPayload(invalid.data(), invalid.size(), invalid.size()),
    contract::JpegPayloadStatus::kMalformedMarker);
}

TEST(CameraFrontJpegContract, RequiresFrameScanAndEntropyData)
{
  auto jpeg = structurallyValidJpeg();
  jpeg[3] = 0xE0;
  EXPECT_EQ(
    contract::validateJpegPayload(jpeg.data(), jpeg.size(), jpeg.size()),
    contract::JpegPayloadStatus::kMissingStartOfFrame);

  jpeg = structurallyValidJpeg();
  jpeg.erase(jpeg.begin() + 15, jpeg.end() - 2);
  EXPECT_EQ(
    contract::validateJpegPayload(jpeg.data(), jpeg.size(), jpeg.size()),
    contract::JpegPayloadStatus::kMissingStartOfScan);

  jpeg = structurallyValidJpeg();
  jpeg.erase(jpeg.end() - 3);
  EXPECT_EQ(
    contract::validateJpegPayload(jpeg.data(), jpeg.size(), jpeg.size()),
    contract::JpegPayloadStatus::kEmptyScanData);
}
