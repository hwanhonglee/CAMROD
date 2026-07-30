#ifndef CAMROD_SENSING__CAMERA_FRONT_CONTRACT_HPP_
#define CAMROD_SENSING__CAMERA_FRONT_CONTRACT_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>

namespace camrod::sensing::camera_front_contract
{

enum class FrameShapeStatus
{
  kValid,
  kNoData,
  kInvalidConfiguredDimensions,
  kUnexpectedWidth,
  kUnexpectedHeight,
  kUnexpectedChannels,
  kUnexpectedElementSize,
  kInsufficientRowStride,
};

inline const char * frameShapeStatusName(FrameShapeStatus status)
{
  switch (status) {
    case FrameShapeStatus::kValid:
      return "valid";
    case FrameShapeStatus::kNoData:
      return "no data";
    case FrameShapeStatus::kInvalidConfiguredDimensions:
      return "invalid configured dimensions";
    case FrameShapeStatus::kUnexpectedWidth:
      return "unexpected width";
    case FrameShapeStatus::kUnexpectedHeight:
      return "unexpected height";
    case FrameShapeStatus::kUnexpectedChannels:
      return "unexpected channel count";
    case FrameShapeStatus::kUnexpectedElementSize:
      return "unexpected element size";
    case FrameShapeStatus::kInsufficientRowStride:
      return "insufficient row stride";
  }
  return "unknown";
}

inline bool validYuv420Dimensions(int width, int height)
{
  return width > 0 && height > 0 && width % 2 == 0 && height % 2 == 0;
}

inline FrameShapeStatus validateFrameShape(
  bool has_data,
  int rows,
  int columns,
  int channels,
  std::size_t element_size,
  std::size_t row_stride,
  int expected_width,
  int expected_rows,
  int expected_channels)
{
  if (!has_data) {
    return FrameShapeStatus::kNoData;
  }
  if (expected_width <= 0 || expected_rows <= 0 || expected_channels <= 0) {
    return FrameShapeStatus::kInvalidConfiguredDimensions;
  }
  if (columns != expected_width) {
    return FrameShapeStatus::kUnexpectedWidth;
  }
  if (rows != expected_rows) {
    return FrameShapeStatus::kUnexpectedHeight;
  }
  if (channels != expected_channels) {
    return FrameShapeStatus::kUnexpectedChannels;
  }
  if (element_size != 1U) {
    return FrameShapeStatus::kUnexpectedElementSize;
  }

  const auto width = static_cast<std::size_t>(expected_width);
  const auto channel_count = static_cast<std::size_t>(expected_channels);
  if (width > std::numeric_limits<std::size_t>::max() / channel_count ||
    row_stride < width * channel_count)
  {
    return FrameShapeStatus::kInsufficientRowStride;
  }
  return FrameShapeStatus::kValid;
}

inline FrameShapeStatus validateBgrFrameShape(
  bool has_data,
  int rows,
  int columns,
  int channels,
  std::size_t element_size,
  std::size_t row_stride,
  int expected_width,
  int expected_height)
{
  if (!validYuv420Dimensions(expected_width, expected_height)) {
    return FrameShapeStatus::kInvalidConfiguredDimensions;
  }
  return validateFrameShape(
    has_data, rows, columns, channels, element_size, row_stride,
    expected_width, expected_height, 3);
}

inline FrameShapeStatus validateNv12FrameShape(
  bool has_data,
  int rows,
  int columns,
  int channels,
  std::size_t element_size,
  std::size_t row_stride,
  int expected_width,
  int expected_height)
{
  if (!validYuv420Dimensions(expected_width, expected_height)) {
    return FrameShapeStatus::kInvalidConfiguredDimensions;
  }
  if (expected_height >
    std::numeric_limits<int>::max() - expected_height / 2)
  {
    return FrameShapeStatus::kInvalidConfiguredDimensions;
  }
  return validateFrameShape(
    has_data, rows, columns, channels, element_size, row_stride,
    expected_width, expected_height + expected_height / 2, 1);
}

constexpr std::size_t kJpegHeaderAllowanceBytes = 64U * 1024U;
constexpr std::size_t kMaxJpegBytesPerPixel = 8U;

inline std::size_t maxJpegPayloadBytes(int width, int height)
{
  if (!validYuv420Dimensions(width, height)) {
    return 0U;
  }

  const auto unsigned_width = static_cast<std::size_t>(width);
  const auto unsigned_height = static_cast<std::size_t>(height);
  if (unsigned_width > std::numeric_limits<std::size_t>::max() / unsigned_height) {
    return 0U;
  }

  const std::size_t pixels = unsigned_width * unsigned_height;
  const std::size_t payload_limit =
    std::numeric_limits<std::size_t>::max() - kJpegHeaderAllowanceBytes;
  if (pixels > payload_limit / kMaxJpegBytesPerPixel) {
    return 0U;
  }
  return pixels * kMaxJpegBytesPerPixel + kJpegHeaderAllowanceBytes;
}

enum class JpegPayloadStatus
{
  kValid,
  kEmpty,
  kTooSmall,
  kTooLarge,
  kNoData,
  kMissingStartOfImage,
  kMissingEndOfImage,
  kMalformedMarker,
  kMissingStartOfFrame,
  kMissingStartOfScan,
  kEmptyScanData,
};

inline const char * jpegPayloadStatusName(JpegPayloadStatus status)
{
  switch (status) {
    case JpegPayloadStatus::kValid:
      return "valid";
    case JpegPayloadStatus::kEmpty:
      return "empty payload";
    case JpegPayloadStatus::kTooSmall:
      return "payload too small";
    case JpegPayloadStatus::kTooLarge:
      return "payload too large";
    case JpegPayloadStatus::kNoData:
      return "null payload";
    case JpegPayloadStatus::kMissingStartOfImage:
      return "missing SOI marker";
    case JpegPayloadStatus::kMissingEndOfImage:
      return "missing EOI marker";
    case JpegPayloadStatus::kMalformedMarker:
      return "malformed JPEG marker";
    case JpegPayloadStatus::kMissingStartOfFrame:
      return "missing SOF marker";
    case JpegPayloadStatus::kMissingStartOfScan:
      return "missing SOS marker";
    case JpegPayloadStatus::kEmptyScanData:
      return "empty scan data";
  }
  return "unknown";
}

inline bool isStartOfFrameMarker(std::uint8_t marker)
{
  return marker >= 0xC0U && marker <= 0xCFU &&
         marker != 0xC4U && marker != 0xC8U && marker != 0xCCU;
}

inline JpegPayloadStatus validateJpegPayload(
  const std::uint8_t * data,
  std::size_t size,
  std::size_t maximum_size)
{
  if (size == 0U) {
    return JpegPayloadStatus::kEmpty;
  }
  if (maximum_size == 0U || size > maximum_size) {
    return JpegPayloadStatus::kTooLarge;
  }
  if (data == nullptr) {
    return JpegPayloadStatus::kNoData;
  }
  if (size < 10U) {
    return JpegPayloadStatus::kTooSmall;
  }
  if (data[0] != 0xFFU || data[1] != 0xD8U) {
    return JpegPayloadStatus::kMissingStartOfImage;
  }
  if (data[size - 2U] != 0xFFU || data[size - 1U] != 0xD9U) {
    return JpegPayloadStatus::kMissingEndOfImage;
  }

  bool saw_start_of_frame = false;
  std::size_t offset = 2U;
  const std::size_t end_of_image_offset = size - 2U;
  while (offset < end_of_image_offset) {
    if (data[offset] != 0xFFU) {
      return JpegPayloadStatus::kMalformedMarker;
    }
    while (offset < end_of_image_offset && data[offset] == 0xFFU) {
      ++offset;
    }
    if (offset >= end_of_image_offset) {
      return JpegPayloadStatus::kMalformedMarker;
    }

    const std::uint8_t marker = data[offset++];
    if (marker == 0x00U || marker == 0x01U || marker == 0xD8U ||
      (marker >= 0xD0U && marker <= 0xD9U))
    {
      return JpegPayloadStatus::kMalformedMarker;
    }
    if (offset + 2U > end_of_image_offset) {
      return JpegPayloadStatus::kMalformedMarker;
    }

    const std::size_t segment_length =
      (static_cast<std::size_t>(data[offset]) << 8U) |
      static_cast<std::size_t>(data[offset + 1U]);
    if (segment_length < 2U || segment_length > end_of_image_offset - offset) {
      return JpegPayloadStatus::kMalformedMarker;
    }

    if (isStartOfFrameMarker(marker)) {
      saw_start_of_frame = true;
    }

    offset += segment_length;
    if (marker == 0xDAU) {
      if (!saw_start_of_frame) {
        return JpegPayloadStatus::kMissingStartOfFrame;
      }
      if (offset >= end_of_image_offset) {
        return JpegPayloadStatus::kEmptyScanData;
      }
      return JpegPayloadStatus::kValid;
    }
  }
  return JpegPayloadStatus::kMissingStartOfScan;
}

}  // namespace camrod::sensing::camera_front_contract

#endif  // CAMROD_SENSING__CAMERA_FRONT_CONTRACT_HPP_
