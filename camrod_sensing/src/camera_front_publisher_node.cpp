#include "camrod_sensing/camera_front_publisher_node.hpp"
#include "camrod_sensing/camera_front_contract.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <limits>
#include <stdexcept>
#include <sys/ioctl.h>
#include <vpi/Status.h>
#include <std_msgs/msg/header.hpp>

namespace camrod::sensing
{
namespace
{
std::string normalizeModeToken(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

const char * nvjpegStatusName(nvjpegStatus_t status)
{
  switch (status) {
    case NVJPEG_STATUS_SUCCESS:
      return "success";
    case NVJPEG_STATUS_NOT_INITIALIZED:
      return "not initialized";
    case NVJPEG_STATUS_INVALID_PARAMETER:
      return "invalid parameter";
    case NVJPEG_STATUS_BAD_JPEG:
      return "bad JPEG";
    case NVJPEG_STATUS_JPEG_NOT_SUPPORTED:
      return "JPEG not supported";
    case NVJPEG_STATUS_ALLOCATOR_FAILURE:
      return "allocator failure";
    case NVJPEG_STATUS_EXECUTION_FAILED:
      return "execution failed";
    case NVJPEG_STATUS_ARCH_MISMATCH:
      return "architecture mismatch";
    case NVJPEG_STATUS_INTERNAL_ERROR:
      return "internal error";
    case NVJPEG_STATUS_IMPLEMENTATION_NOT_SUPPORTED:
      return "implementation not supported";
    case NVJPEG_STATUS_INCOMPLETE_BITSTREAM:
      return "incomplete bitstream";
  }
  return "unknown";
}

sensor_msgs::msg::Image makeBgrImageMessage(
  const cv::Mat & bgr,
  const std_msgs::msg::Header & header)
{
  if (bgr.empty() || bgr.type() != CV_8UC3) {
    throw std::invalid_argument("Raw front camera image must be non-empty CV_8UC3");
  }

  const std::size_t row_bytes =
    static_cast<std::size_t>(bgr.cols) * bgr.elemSize();
  const std::size_t rows = static_cast<std::size_t>(bgr.rows);
  if (row_bytes > std::numeric_limits<std::uint32_t>::max() ||
    rows > std::numeric_limits<std::size_t>::max() / row_bytes)
  {
    throw std::length_error("Raw front camera image exceeds sensor_msgs/Image limits");
  }

  sensor_msgs::msg::Image message;
  message.header = header;
  message.height = static_cast<std::uint32_t>(bgr.rows);
  message.width = static_cast<std::uint32_t>(bgr.cols);
  message.encoding = "bgr8";
  message.is_bigendian = false;
  message.step = static_cast<std::uint32_t>(row_bytes);
  message.data.resize(rows * row_bytes);

  for (int row = 0; row < bgr.rows; ++row) {
    std::memcpy(
      message.data.data() + static_cast<std::size_t>(row) * row_bytes,
      bgr.ptr(row), row_bytes);
  }
  return message;
}
}  // namespace

CameraFrontPublisherNode::CameraFrontPublisherNode(const rclcpp::NodeOptions & options)
: Node("camera_front_publisher", options)
{
  this->declare_parameter<std::string>("camera_name", "camera");
  this->declare_parameter<std::string>("camera_frame_id", "camera_frame");
  this->declare_parameter<std::string>("device_path", "/dev/video0");
  this->declare_parameter<int>("image_width", 640);
  this->declare_parameter<int>("image_height", 480);
  this->declare_parameter<int>("fps", 30);
  // HH_260526: Replace use_custom_intrinsics toggle with explicit source mode.
  // intrinsics_source options: none | custom.
  this->declare_parameter<std::string>("intrinsics_source", "none");
  this->declare_parameter<std::vector<double>>("camera_matrix", std::vector<double>{});
  this->declare_parameter<std::vector<double>>("distortion_coefficients", std::vector<double>{});
  this->declare_parameter<int>("exposure_time_us", 25000);
  this->declare_parameter<int>("jpeg_quality", 80);

  std::string camera_name;
  this->get_parameter("camera_name", camera_name);
  this->get_parameter("camera_frame_id", camera_frame_id_);
  this->get_parameter("device_path", device_path_);
  this->get_parameter("image_width", image_width_);
  this->get_parameter("image_height", image_height_);
  this->get_parameter("fps", fps_);
  intrinsics_source_ = normalizeModeToken(
    this->get_parameter("intrinsics_source").as_string());
  this->get_parameter("camera_matrix", camera_matrix_);
  this->get_parameter("distortion_coefficients", distortion_coefficients_);
  int exposure_time_us;
  this->get_parameter("exposure_time_us", exposure_time_us);
  this->get_parameter("jpeg_quality", jpeg_quality_);
  max_jpeg_payload_size_ =
    camera_front_contract::maxJpegPayloadBytes(image_width_, image_height_);
  if (max_jpeg_payload_size_ == 0U) {
    throw std::invalid_argument(
            "Front camera image_width/image_height must be positive, even YUV420 dimensions");
  }
  if (fps_ <= 0) {
    throw std::invalid_argument("Front camera fps must be positive");
  }
  if (jpeg_quality_ < 1 || jpeg_quality_ > 100) {
    throw std::invalid_argument("Front camera jpeg_quality must be in [1, 100]");
  }
  if (intrinsics_source_ != "none" && intrinsics_source_ != "custom") {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid intrinsics_source='%s'. Falling back to 'none'.",
      intrinsics_source_.c_str());
    intrinsics_source_ = "none";
  }

  RCLCPP_INFO(this->get_logger(), "Initializing camera: %s", camera_name.c_str());
  RCLCPP_INFO(this->get_logger(), "Device path: %s", device_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "Target FPS: %d", fps_);

  // HJ_260529: removed io-mode=4 (DMABUF) — ISX031 econ camera does not support it.
  // videorate before nvvidconv: drops UYVY frames first so VIC (nvvidconv) only
  // runs at fps_, not at the sensor's native 30fps. 3x reduction in VIC calls.
  std::string gst_pipeline =
    "v4l2src device=" + device_path_ +
    " ! video/x-raw,width=" + std::to_string(image_width_) +
    ",height=" + std::to_string(image_height_) +
    ",format=UYVY,framerate=30/1" +
    " ! videorate"
    " ! video/x-raw,format=UYVY,framerate=" + std::to_string(fps_) + "/1" +
    " ! nvvidconv"
    " ! video/x-raw,format=NV12"
    " ! appsink max-buffers=2 drop=true sync=false";

  RCLCPP_INFO(this->get_logger(), "Opening camera with GStreamer...");
  cap_.open(gst_pipeline, cv::CAP_GSTREAMER);

  if (!cap_.isOpened()) {
    RCLCPP_WARN(this->get_logger(), "GStreamer failed, falling back to V4L2 direct");
    cap_.open(device_path_, cv::CAP_V4L2);
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('U', 'Y', 'V', 'Y'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  image_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
    cap_.set(cv::CAP_PROP_FPS,          30);
    cap_.set(cv::CAP_PROP_BUFFERSIZE,   2);
    use_v4l2_fallback_ = true;
  }

  if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
    throw std::runtime_error("Failed to open camera");
  }

  RCLCPP_INFO(this->get_logger(), "Camera opened successfully (%s)",
    use_v4l2_fallback_ ? "V4L2 direct" : "GStreamer");

  // ISX031 default exposure is 33000us = 33ms, which equals the 30fps frame period.
  // The AE algorithm periodically exceeds this limit and drops to 18fps.
  // Fix: manual mode with exposure_time_us (default 25000us = 25ms),
  // leaving 8ms margin for frame overhead and ensuring stable 30fps.
  {
    int fd = open(device_path_.c_str(), O_RDWR);
    if (fd >= 0) {
      struct v4l2_control ctrl = {};
      ctrl.id = V4L2_CID_EXPOSURE_AUTO;
      ctrl.value = V4L2_EXPOSURE_MANUAL;
      ioctl(fd, VIDIOC_S_CTRL, &ctrl);

      ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
      ctrl.value = exposure_time_us;
      if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) < 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to set exposure time");
      } else {
        RCLCPP_INFO(this->get_logger(), "Exposure set to %d us", exposure_time_us);
      }
      close(fd);
    }
  }

  rect_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
    "~/image_rect/compressed", rclcpp::SensorDataQoS());
  image_raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "~/image_raw", rclcpp::SensorDataQoS());
  camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    "~/camera_info", rclcpp::SensorDataQoS());

  loadCameraInfo();

  u_plane_ = cv::Mat(image_height_ / 2, image_width_ / 2, CV_8UC1);
  v_plane_ = cv::Mat(image_height_ / 2, image_width_ / 2, CV_8UC1);

  compressed_msg_.header.frame_id = camera_frame_id_;
  compressed_msg_.format = "jpeg";

  // Initialize NvJPEG before VPI so a constructor-time CUDA/NvJPEG error cannot
  // strand raw VPI resources while the partially constructed object unwinds.
  initNvJpeg();
  initVpiRemap();

  // captureThread stays fast (never blocked by DDS serialization)
  // publishThread handles the slow path (DDS overhead from subscribers)
  capture_thread_ = std::thread(&CameraFrontPublisherNode::captureThread, this);
  publish_thread_ = std::thread(&CameraFrontPublisherNode::publishThread, this);

  RCLCPP_INFO(this->get_logger(), "Camera front publisher node started");
}

CameraFrontPublisherNode::~CameraFrontPublisherNode()
{
  stop_capture_ = true;
  frame_cv_.notify_all();

  if (capture_thread_.joinable()) capture_thread_.join();
  if (publish_thread_.joinable()) publish_thread_.join();

  if (cap_.isOpened()) cap_.release();

  // VPI cleanup after threads have stopped
  if (vpi_stream_)           { vpiStreamSync(vpi_stream_); vpiStreamDestroy(vpi_stream_); }
  if (vpi_nv12_in_wrapper_)  { vpiImageDestroy(vpi_nv12_in_wrapper_); }
  if (vpi_nv12_out_wrapper_) { vpiImageDestroy(vpi_nv12_out_wrapper_); }
  if (vpi_remap_payload_)    { vpiPayloadDestroy(vpi_remap_payload_); }
  cleanupNvJpeg();
}

void CameraFrontPublisherNode::initNvJpeg()
{
  const auto require_cuda = [this](cudaError_t status, const char * operation) {
      if (status == cudaSuccess) {
        return;
      }
      const char * error_name = cudaGetErrorName(status);
      const char * error_text = cudaGetErrorString(status);
      const std::string message =
        std::string(operation) + " failed: " +
        (error_name != nullptr ? error_name : "unknown CUDA error") + " (" +
        (error_text != nullptr ? error_text : "no error text") + ")";
      RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
      cleanupNvJpeg();
      throw std::runtime_error(message);
    };
  const auto require_nvjpeg = [this](nvjpegStatus_t status, const char * operation) {
      if (status == NVJPEG_STATUS_SUCCESS) {
        return;
      }
      const std::string message =
        std::string(operation) + " failed: " + nvjpegStatusName(status) +
        " (status=" + std::to_string(static_cast<int>(status)) + ")";
      RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
      cleanupNvJpeg();
      throw std::runtime_error(message);
    };

  require_cuda(cudaStreamCreate(&cuda_stream_), "cudaStreamCreate");
  require_nvjpeg(nvjpegCreateSimple(&nvjpeg_handle_), "nvjpegCreateSimple");
  require_nvjpeg(
    nvjpegEncoderStateCreate(nvjpeg_handle_, &nvjpeg_state_, cuda_stream_),
    "nvjpegEncoderStateCreate");
  require_nvjpeg(
    nvjpegEncoderParamsCreate(nvjpeg_handle_, &nvjpeg_params_, cuda_stream_),
    "nvjpegEncoderParamsCreate");
  require_nvjpeg(
    nvjpegEncoderParamsSetQuality(nvjpeg_params_, jpeg_quality_, cuda_stream_),
    "nvjpegEncoderParamsSetQuality");
  require_nvjpeg(
    nvjpegEncoderParamsSetSamplingFactors(
      nvjpeg_params_, NVJPEG_CSS_420, cuda_stream_),
    "nvjpegEncoderParamsSetSamplingFactors");

  const std::size_t y_size =
    static_cast<std::size_t>(image_width_) * static_cast<std::size_t>(image_height_);
  const std::size_t uv_size = y_size / 4U;
  require_cuda(
    cudaMalloc(reinterpret_cast<void **>(&d_y_), y_size),
    "cudaMalloc(Y)");
  require_cuda(
    cudaMalloc(reinterpret_cast<void **>(&d_u_), uv_size),
    "cudaMalloc(U)");
  require_cuda(
    cudaMalloc(reinterpret_cast<void **>(&d_v_), uv_size),
    "cudaMalloc(V)");
  require_cuda(
    cudaStreamSynchronize(cuda_stream_),
    "cudaStreamSynchronize(NvJPEG initialization)");
}

void CameraFrontPublisherNode::cleanupNvJpeg()
{
  const auto warn_cuda = [this](cudaError_t status, const char * operation) {
      if (status != cudaSuccess) {
        RCLCPP_WARN(
          this->get_logger(), "%s failed during cleanup: %s (%s)",
          operation, cudaGetErrorName(status), cudaGetErrorString(status));
      }
    };
  const auto warn_nvjpeg = [this](nvjpegStatus_t status, const char * operation) {
      if (status != NVJPEG_STATUS_SUCCESS) {
        RCLCPP_WARN(
          this->get_logger(), "%s failed during cleanup: %s (status=%d)",
          operation, nvjpegStatusName(status), static_cast<int>(status));
      }
    };

  if (d_v_ != nullptr) {
    warn_cuda(cudaFree(d_v_), "cudaFree(V)");
    d_v_ = nullptr;
  }
  if (d_u_ != nullptr) {
    warn_cuda(cudaFree(d_u_), "cudaFree(U)");
    d_u_ = nullptr;
  }
  if (d_y_ != nullptr) {
    warn_cuda(cudaFree(d_y_), "cudaFree(Y)");
    d_y_ = nullptr;
  }
  if (nvjpeg_params_ != nullptr) {
    warn_nvjpeg(
      nvjpegEncoderParamsDestroy(nvjpeg_params_),
      "nvjpegEncoderParamsDestroy");
    nvjpeg_params_ = nullptr;
  }
  if (nvjpeg_state_ != nullptr) {
    warn_nvjpeg(
      nvjpegEncoderStateDestroy(nvjpeg_state_),
      "nvjpegEncoderStateDestroy");
    nvjpeg_state_ = nullptr;
  }
  if (nvjpeg_handle_ != nullptr) {
    warn_nvjpeg(nvjpegDestroy(nvjpeg_handle_), "nvjpegDestroy");
    nvjpeg_handle_ = nullptr;
  }
  if (cuda_stream_ != nullptr) {
    warn_cuda(cudaStreamDestroy(cuda_stream_), "cudaStreamDestroy");
    cuda_stream_ = nullptr;
  }
}

bool CameraFrontPublisherNode::encodeRectifiedJpeg()
{
  const auto cuda_ok = [this](cudaError_t status, const char * operation) {
      if (status == cudaSuccess) {
        return true;
      }
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Front JPEG dropped: %s failed: %s (%s)",
        operation, cudaGetErrorName(status), cudaGetErrorString(status));
      return false;
    };
  const auto nvjpeg_ok = [this](nvjpegStatus_t status, const char * operation) {
      if (status == NVJPEG_STATUS_SUCCESS) {
        return true;
      }
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Front JPEG dropped: %s failed: %s (status=%d)",
        operation, nvjpegStatusName(status), static_cast<int>(status));
      return false;
    };

  const std::size_t y_size =
    static_cast<std::size_t>(image_width_) * static_cast<std::size_t>(image_height_);
  const std::size_t uv_size = y_size / 4U;
  if (!cuda_ok(
      cudaMemcpyAsync(
        d_y_, nv12_rect_buf_.data, y_size,
        cudaMemcpyHostToDevice, cuda_stream_),
      "cudaMemcpyAsync(Y)") ||
    !cuda_ok(
      cudaMemcpyAsync(
        d_u_, u_plane_.data, uv_size,
        cudaMemcpyHostToDevice, cuda_stream_),
      "cudaMemcpyAsync(U)") ||
    !cuda_ok(
      cudaMemcpyAsync(
        d_v_, v_plane_.data, uv_size,
        cudaMemcpyHostToDevice, cuda_stream_),
      "cudaMemcpyAsync(V)"))
  {
    return false;
  }

  nvjpegImage_t yuv_img = {};
  yuv_img.channel[0] = d_y_;
  yuv_img.pitch[0] = static_cast<unsigned int>(image_width_);
  yuv_img.channel[1] = d_u_;
  yuv_img.pitch[1] = static_cast<unsigned int>(image_width_ / 2);
  yuv_img.channel[2] = d_v_;
  yuv_img.pitch[2] = static_cast<unsigned int>(image_width_ / 2);

  const bool encode_ok = nvjpeg_ok(
    nvjpegEncodeYUV(
      nvjpeg_handle_, nvjpeg_state_, nvjpeg_params_,
      &yuv_img, NVJPEG_CSS_420, image_width_, image_height_, cuda_stream_),
    "nvjpegEncodeYUV");
  const bool encode_sync_ok = cuda_ok(
    cudaStreamSynchronize(cuda_stream_),
    "cudaStreamSynchronize(encode)");
  if (!encode_ok || !encode_sync_ok) {
    return false;
  }

  std::size_t jpeg_size = 0U;
  const bool size_retrieve_ok = nvjpeg_ok(
    nvjpegEncodeRetrieveBitstream(
      nvjpeg_handle_, nvjpeg_state_, nullptr, &jpeg_size, cuda_stream_),
    "nvjpegEncodeRetrieveBitstream(size)");
  const bool size_sync_ok = cuda_ok(
    cudaStreamSynchronize(cuda_stream_),
    "cudaStreamSynchronize(size retrieve)");
  if (!size_retrieve_ok || !size_sync_ok) {
    return false;
  }
  if (jpeg_size == 0U || jpeg_size > max_jpeg_payload_size_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Front JPEG dropped: invalid queried payload size %zu (maximum %zu)",
      jpeg_size, max_jpeg_payload_size_);
    return false;
  }

  try {
    compressed_msg_.data.resize(jpeg_size);
  } catch (const std::exception & error) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Front JPEG dropped: could not allocate %zu-byte payload: %s",
      jpeg_size, error.what());
    return false;
  }

  const std::size_t buffer_size = compressed_msg_.data.size();
  const bool data_retrieve_ok = nvjpeg_ok(
    nvjpegEncodeRetrieveBitstream(
      nvjpeg_handle_, nvjpeg_state_, compressed_msg_.data.data(),
      &jpeg_size, cuda_stream_),
    "nvjpegEncodeRetrieveBitstream(data)");
  const bool data_sync_ok = cuda_ok(
    cudaStreamSynchronize(cuda_stream_),
    "cudaStreamSynchronize(data retrieve)");
  if (!data_retrieve_ok || !data_sync_ok) {
    compressed_msg_.data.clear();
    return false;
  }
  if (jpeg_size == 0U || jpeg_size > buffer_size ||
    jpeg_size > max_jpeg_payload_size_)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Front JPEG dropped: invalid retrieved payload size %zu "
      "(buffer %zu, maximum %zu)",
      jpeg_size, buffer_size, max_jpeg_payload_size_);
    compressed_msg_.data.clear();
    return false;
  }
  compressed_msg_.data.resize(jpeg_size);

  const auto payload_status = camera_front_contract::validateJpegPayload(
    compressed_msg_.data.data(), compressed_msg_.data.size(),
    max_jpeg_payload_size_);
  if (payload_status != camera_front_contract::JpegPayloadStatus::kValid) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Front JPEG dropped: invalid %zu-byte payload (%s)",
      compressed_msg_.data.size(),
      camera_front_contract::jpegPayloadStatusName(payload_status));
    compressed_msg_.data.clear();
    return false;
  }
  return true;
}

void CameraFrontPublisherNode::captureThread()
{
  while (!stop_capture_) {
    cv::Mat frame;

    if (!cap_.read(frame) || frame.empty()) {
      continue;
    }

    if (use_v4l2_fallback_) {
      // V4L2 direct returns BGR; VPI/NvJPEG pipeline requires NV12 semi-planar.
      // BGR → I420 (planar YUV420) → NV12 (interleave U and V into semi-planar UV).
      const auto bgr_status = camera_front_contract::validateBgrFrameShape(
        frame.data != nullptr, frame.rows, frame.cols, frame.channels(),
        frame.elemSize1(), frame.step[0], image_width_, image_height_);
      if (bgr_status != camera_front_contract::FrameShapeStatus::kValid) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "V4L2 frame dropped: %s (actual rows=%d cols=%d channels=%d step=%zu; "
          "expected BGR %dx%d)",
          camera_front_contract::frameShapeStatusName(bgr_status),
          frame.rows, frame.cols, frame.channels(), frame.step[0],
          image_width_, image_height_);
        continue;
      }

      const int src_h = frame.rows;
      const int src_w = frame.cols;
      cv::Mat i420;
      cv::cvtColor(frame, i420, cv::COLOR_BGR2YUV_I420);
      cv::Mat nv12(src_h * 3 / 2, src_w, CV_8UC1);
      const size_t y_bytes = static_cast<size_t>(src_w) * src_h;
      std::memcpy(nv12.data, i420.data, y_bytes);
      const int uv_count = (src_w / 2) * (src_h / 2);
      const uint8_t* src_u = i420.data + y_bytes;
      const uint8_t* src_v = src_u + uv_count;
      uint8_t* dst_uv = nv12.data + y_bytes;
      for (int i = 0; i < uv_count; ++i) {
        dst_uv[2 * i]     = src_u[i];
        dst_uv[2 * i + 1] = src_v[i];
      }
      frame = std::move(nv12);
    }

    const auto nv12_status = camera_front_contract::validateNv12FrameShape(
      frame.data != nullptr, frame.rows, frame.cols, frame.channels(),
      frame.elemSize1(), frame.step[0], image_width_, image_height_);
    if (nv12_status != camera_front_contract::FrameShapeStatus::kValid) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Captured NV12 frame dropped: %s "
        "(actual rows=%d cols=%d channels=%d step=%zu; expected %dx%d NV12)",
        camera_front_contract::frameShapeStatusName(nv12_status),
        frame.rows, frame.cols, frame.channels(), frame.step[0],
        image_width_, image_height_);
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_nv12_frame_ = std::move(frame);
      new_frame_available_ = true;
    }
    frame_cv_.notify_one();
  }
}

void CameraFrontPublisherNode::publishThread()
{

  while (!stop_capture_) {
    cv::Mat nv12_frame;

    {
      std::unique_lock<std::mutex> lock(frame_mutex_);
      frame_cv_.wait(lock, [this] { return new_frame_available_ || stop_capture_; });

      if (stop_capture_) break;

      nv12_frame = std::move(latest_nv12_frame_);
      new_frame_available_ = false;
    }

    const auto nv12_status = camera_front_contract::validateNv12FrameShape(
      nv12_frame.data != nullptr, nv12_frame.rows, nv12_frame.cols,
      nv12_frame.channels(), nv12_frame.elemSize1(), nv12_frame.step[0],
      image_width_, image_height_);
    if (nv12_status != camera_front_contract::FrameShapeStatus::kValid) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Queued NV12 frame dropped: %s",
        camera_front_contract::frameShapeStatusName(nv12_status));
      continue;
    }

    auto timestamp = this->now();
    compressed_msg_.header.stamp = timestamp;

    camera_info_msg_.header.stamp = timestamp;
    camera_info_pub_->publish(camera_info_msg_);

    if (vpi_ready_) {
      // Wrap NV12 frame (H*3/2 × W, CV_8UC1) — VPI understands this as NV12 semi-planar
      if (!vpi_nv12_in_wrapper_) {
        vpiImageCreateWrapperOpenCVMat(
          nv12_frame, VPI_IMAGE_FORMAT_NV12,
          VPI_BACKEND_CPU | VPI_BACKEND_VIC, &vpi_nv12_in_wrapper_);
      } else {
        vpiImageSetWrappedOpenCVMat(vpi_nv12_in_wrapper_, nv12_frame);
      }

      // Fisheye undistortion on VIC (zero CPU): NV12 → NV12_rect
      vpiSubmitRemap(vpi_stream_, VPI_BACKEND_VIC, vpi_remap_payload_,
        vpi_nv12_in_wrapper_, vpi_nv12_out_wrapper_,
        VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0);
      vpiStreamSync(vpi_stream_);

      // Cache coherency: flush VIC writes to nv12_rect_buf_ into CPU view
      vpiImageLock(vpi_nv12_out_wrapper_, VPI_LOCK_READ);

      // NEON deinterleave NV12 UV → planar U, V (16 pixels per iteration)
      const uint8_t* uv_src = nv12_rect_buf_.data + image_width_ * image_height_;
      uint8_t* u_dst = u_plane_.data;
      uint8_t* v_dst = v_plane_.data;
      const int uv_count = (image_width_ / 2) * (image_height_ / 2);
      int i = 0;
      for (; i <= uv_count - 16; i += 16) {
        uint8x16x2_t uv = vld2q_u8(uv_src + 2 * i);
        vst1q_u8(u_dst + i, uv.val[0]);
        vst1q_u8(v_dst + i, uv.val[1]);
      }
      for (; i < uv_count; ++i) {
        u_dst[i] = uv_src[2 * i];
        v_dst[i] = uv_src[2 * i + 1];
      }

      vpiImageUnlock(vpi_nv12_out_wrapper_);

      // image_raw: unrectified BGR, subscriber-gated (~6 MB/frame — skip when no consumers)
      if (image_raw_pub_->get_subscription_count() > 0) {
        cv::Mat bgr;
        cv::cvtColor(nv12_frame, bgr, cv::COLOR_YUV2BGR_NV12);
        image_raw_pub_->publish(
          makeBgrImageMessage(bgr, compressed_msg_.header));
      }

      if (!encodeRectifiedJpeg()) {
        continue;
      }

      // HH_260805 - Move the encoded payload into a unique message so the
      // camera+YOLO component container can hand it over intra-process without
      // another JPEG-vector copy. The reusable encoder buffer is resized next frame.
      auto rect_message = std::make_unique<sensor_msgs::msg::CompressedImage>();
      rect_message->header = compressed_msg_.header;
      rect_message->format = compressed_msg_.format;
      rect_message->data.swap(compressed_msg_.data);
      rect_pub_->publish(std::move(rect_message));
    } else if (image_raw_pub_->get_subscription_count() > 0) {
      // VPI pipeline not ready (no calibration / V4L2 fallback): publish unrectified BGR
      cv::Mat bgr;
      cv::cvtColor(nv12_frame, bgr, cv::COLOR_YUV2BGR_NV12);
      image_raw_pub_->publish(
        makeBgrImageMessage(bgr, compressed_msg_.header));
    }
  }
}

void CameraFrontPublisherNode::loadCameraInfo()
{
  camera_info_msg_.header.frame_id = camera_frame_id_;
  camera_info_msg_.width = image_width_;
  camera_info_msg_.height = image_height_;
  camera_info_msg_.distortion_model = "equidistant";

  if (intrinsics_source_ == "custom" && camera_matrix_.size() == 9) {
    RCLCPP_INFO(this->get_logger(), "Using custom intrinsics (fisheye/equidistant)");
    for (int i = 0; i < 9; ++i) {
      camera_info_msg_.k[i] = camera_matrix_[i];
    }
    // Fisheye model requires exactly 4 coefficients [k1, k2, k3, k4]
    if (distortion_coefficients_.size() >= 4) {
      camera_info_msg_.d = std::vector<double>(
        distortion_coefficients_.begin(), distortion_coefficients_.begin() + 4);
    }
    camera_info_msg_.r[0] = 1.0;
    camera_info_msg_.r[4] = 1.0;
    camera_info_msg_.r[8] = 1.0;

    cv::Mat K_cv = (cv::Mat_<double>(3, 3)
      << camera_matrix_[0], 0.0, camera_matrix_[2],
         0.0, camera_matrix_[4], camera_matrix_[5],
         0.0, 0.0, 1.0);
    // fisheye::estimateNewCameraMatrixForUndistortRectify requires D as 1x4
    cv::Mat D_cv = (cv::Mat_<double>(1, 4)
      << distortion_coefficients_[0], distortion_coefficients_[1],
         distortion_coefficients_[2], distortion_coefficients_[3]);

    // balance=0: crop to valid pixels only (no black borders)
    // balance=1: show all pixels (black borders visible at edges)
    cv::Mat P_cv;
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
      K_cv, D_cv, cv::Size(image_width_, image_height_),
      cv::Mat::eye(3, 3, CV_64F), P_cv, 0.0);

    // P_cv is 3x3 for fisheye; CameraInfo P field is row-major 3x4
    camera_info_msg_.p[0]  = P_cv.at<double>(0, 0);
    camera_info_msg_.p[2]  = P_cv.at<double>(0, 2);
    camera_info_msg_.p[5]  = P_cv.at<double>(1, 1);
    camera_info_msg_.p[6]  = P_cv.at<double>(1, 2);
    camera_info_msg_.p[10] = 1.0;
    RCLCPP_INFO(this->get_logger(),
      "Fisheye P: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
      P_cv.at<double>(0,0), P_cv.at<double>(1,1),
      P_cv.at<double>(0,2), P_cv.at<double>(1,2));

  } else {
    RCLCPP_WARN(this->get_logger(), "No calibration loaded");
  }
}

void CameraFrontPublisherNode::initVpiRemap()
{
  if (intrinsics_source_ != "custom" || camera_matrix_.size() < 9 ||
      distortion_coefficients_.size() < 4) {
    RCLCPP_WARN(this->get_logger(), "VPI remap skipped: no calibration");
    return;
  }

  if (vpiStreamCreate(VPI_BACKEND_VIC, &vpi_stream_) != VPI_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create VPI VIC stream");
    return;
  }

  // Sparse warp grid (8-pixel intervals): ~33K control points for 1920x1080,
  // VIC bicubic-interpolates between them — sufficient accuracy for smooth fisheye.
  VPIWarpMap warp_map = {};
  warp_map.grid.numHorizRegions  = 1;
  warp_map.grid.numVertRegions   = 1;
  warp_map.grid.regionWidth[0]   = image_width_;
  warp_map.grid.regionHeight[0]  = image_height_;
  warp_map.grid.horizInterval[0] = 8;
  warp_map.grid.vertInterval[0]  = 8;
  vpiWarpMapAllocData(&warp_map);

  // Input intrinsics from calibration
  VPICameraIntrinsic Kin = {};
  Kin[0][0] = static_cast<float>(camera_matrix_[0]);  // fx
  Kin[0][2] = static_cast<float>(camera_matrix_[2]);  // cx
  Kin[1][1] = static_cast<float>(camera_matrix_[4]);  // fy
  Kin[1][2] = static_cast<float>(camera_matrix_[5]);  // cy

  // Output intrinsics from the estimated P matrix (set by loadCameraInfo)
  VPICameraIntrinsic Kout = {};
  Kout[0][0] = static_cast<float>(camera_info_msg_.p[0]);  // fx_new
  Kout[0][2] = static_cast<float>(camera_info_msg_.p[2]);  // cx_new
  Kout[1][1] = static_cast<float>(camera_info_msg_.p[5]);  // fy_new
  Kout[1][2] = static_cast<float>(camera_info_msg_.p[6]);  // cy_new

  // Identity extrinsic (monocular, no rotation/translation)
  VPICameraExtrinsic X = {};
  X[0][0] = X[1][1] = X[2][2] = 1.0f;

  // Fisheye equidistant model (matches camera_params.yaml distortion_model)
  VPIFisheyeLensDistortionModel dist = {};
  dist.mapping = VPI_FISHEYE_EQUIDISTANT;
  dist.k1 = static_cast<float>(distortion_coefficients_[0]);
  dist.k2 = static_cast<float>(distortion_coefficients_[1]);
  dist.k3 = static_cast<float>(distortion_coefficients_[2]);
  dist.k4 = static_cast<float>(distortion_coefficients_[3]);

  vpiWarpMapGenerateFromFisheyeLensDistortionModel(Kin, X, Kout, &dist, &warp_map);

  if (vpiCreateRemap(VPI_BACKEND_VIC, &warp_map, &vpi_remap_payload_) != VPI_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create VPI remap payload (VIC)");
    vpiWarpMapFreeData(&warp_map);
    return;
  }
  vpiWarpMapFreeData(&warp_map);

  // Pre-allocate contiguous NV12 output buffer (H*3/2 × W); VIC writes directly here
  nv12_rect_buf_ = cv::Mat(image_height_ * 3 / 2, image_width_, CV_8UC1);
  if (vpiImageCreateWrapperOpenCVMat(
        nv12_rect_buf_, VPI_IMAGE_FORMAT_NV12,
        VPI_BACKEND_CPU | VPI_BACKEND_VIC, &vpi_nv12_out_wrapper_) != VPI_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create VPI NV12 output wrapper");
    return;
  }

  vpi_ready_ = true;
  RCLCPP_INFO(this->get_logger(),
    "VPI VIC fisheye remap ready (%dx%d, equidistant, interval=8)",
    image_width_, image_height_);
}

}  // namespace camrod::sensing

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camrod::sensing::CameraFrontPublisherNode)
