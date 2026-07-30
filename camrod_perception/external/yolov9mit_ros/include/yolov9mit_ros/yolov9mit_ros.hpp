// Copyright 2026 hwanhonglee
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once

#include <chrono>
#include <cstddef>
#include <cmath>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "yolov9mit/utils.hpp"
#include "yolov9mit/yolov9mit.hpp"

namespace yolov9mit_ros
{

class YOLOV9MIT_Node : public rclcpp::Node
{
public:
    YOLOV9MIT_Node(const rclcpp::NodeOptions &);

private:
    std::unique_ptr<yolov9mit::AbcYOLOV9MIT> yolo_;
    std::vector<std::string> class_names_;
    bool imshow_ = false;
    const std::string window_name_ = "yolov9mit_ros";
    std::size_t max_compressed_payload_bytes_ = 16U * 1024U * 1024U;

    // FPS throttle: skip frames when 0 < throttle_fps < camera FPS
    std::chrono::duration<double> throttle_interval_{0.0};
    std::chrono::steady_clock::time_point last_inference_time_{};

    // HJ_260529: direct CompressedImage subscriber (used when transport_hint=="compressed")
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_compressed_;
    image_transport::Subscriber sub_image_;
    image_transport::Publisher pub_image_;

    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_bboxes_;

    void process_image(const cv::Mat & image, const std_msgs::msg::Header & header);
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &);
    void compressed_image_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &);
    void log_compressed_frame_error(
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr &,
        const char * stage,
        const std::string & detail);
    vision_msgs::msg::Detection2DArray::SharedPtr objects_to_bboxes(
        const std::vector<yolov9mit::Object> &, const std_msgs::msg::Header &);
};
} // namespace yolov9mit_ros
