#ifndef CAMERA_FRONT_PUBLISHER_NODE_HPP_
#define CAMERA_FRONT_PUBLISHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <cuda_runtime.h>
#include <nvjpeg.h>
#include <arm_neon.h>
#include <vpi/Stream.h>
#include <vpi/Image.h>
#include <vpi/LensDistortionModels.h>
#include <vpi/OpenCVInterop.hpp>
#include <vpi/algo/Remap.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <string>
#include <vector>

namespace camrod::sensing
{

class CameraFrontPublisherNode : public rclcpp::Node
{
public:
  explicit CameraFrontPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraFrontPublisherNode();

private:
  void captureThread();
  void publishThread();
  void loadCameraInfo();
  void initVpiRemap();

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rect_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  cv::VideoCapture cap_;

  std::string camera_frame_id_;
  std::string device_path_;
  int image_width_;
  int image_height_;
  int fps_;

  sensor_msgs::msg::CameraInfo camera_info_msg_;

  std::string intrinsics_source_;
  std::vector<double> camera_matrix_;
  std::vector<double> distortion_coefficients_;

  // NVJPEG: GPU hardware JPEG encoding (replaces TurboJPEG CPU encoding)
  nvjpegHandle_t        nvjpeg_handle_{nullptr};
  nvjpegEncoderState_t  nvjpeg_state_{nullptr};
  nvjpegEncoderParams_t nvjpeg_params_{nullptr};
  cudaStream_t          cuda_stream_{nullptr};
  uint8_t*              d_y_{nullptr};
  uint8_t*              d_u_{nullptr};
  uint8_t*              d_v_{nullptr};
  cv::Mat               u_plane_;
  cv::Mat               v_plane_;
  int                   jpeg_quality_{80};

  sensor_msgs::msg::CompressedImage compressed_msg_;

  // VPI VIC fisheye remap
  VPIStream   vpi_stream_{nullptr};
  VPIPayload  vpi_remap_payload_{nullptr};
  VPIImage    vpi_nv12_in_wrapper_{nullptr};
  VPIImage    vpi_nv12_out_wrapper_{nullptr};
  cv::Mat     nv12_rect_buf_;   // contiguous NV12 output buffer wrapped by vpi_nv12_out_wrapper_
  bool        vpi_ready_{false};

  // Shared NV12 frame between captureThread and publishThread
  cv::Mat latest_nv12_frame_;
  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  bool new_frame_available_{false};

  std::thread capture_thread_;
  std::thread publish_thread_;
  std::atomic<bool> stop_capture_{false};
};

}  // namespace camrod::sensing

#endif  // CAMERA_FRONT_PUBLISHER_NODE_HPP_
