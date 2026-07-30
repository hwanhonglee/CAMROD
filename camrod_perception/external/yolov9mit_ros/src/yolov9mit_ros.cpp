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

#include "yolov9mit_ros/yolov9mit_ros.hpp"

#include <cstring>
#include <exception>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <stdexcept>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include "yolov9mit/utils.hpp"

namespace yolov9mit_ros
{
namespace
{
sensor_msgs::msg::Image::SharedPtr makeBgrImageMessage(
    const cv::Mat & image,
    const std_msgs::msg::Header & header)
{
    auto message = std::make_shared<sensor_msgs::msg::Image>();
    message->header = header;
    message->height = static_cast<uint32_t>(image.rows);
    message->width = static_cast<uint32_t>(image.cols);
    message->encoding = sensor_msgs::image_encodings::BGR8;
    message->is_bigendian = false;
    message->step = static_cast<uint32_t>(image.cols * image.elemSize());
    message->data.resize(
        static_cast<std::size_t>(message->step) *
        static_cast<std::size_t>(message->height));
    for (int row = 0; row < image.rows; ++row)
    {
        std::memcpy(
            message->data.data() +
            static_cast<std::size_t>(row) * message->step,
            image.ptr(row),
            message->step);
    }
    return message;
}

bool decodeRawImage(
    const sensor_msgs::msg::Image & message,
    cv::Mat & output,
    std::string & error)
{
    int cv_type = 0;
    int conversion = -1;
    std::size_t bytes_per_pixel = 0U;
    if (message.encoding == sensor_msgs::image_encodings::BGR8)
    {
        cv_type = CV_8UC3;
        bytes_per_pixel = 3U;
    }
    else if (message.encoding == sensor_msgs::image_encodings::RGB8)
    {
        cv_type = CV_8UC3;
        bytes_per_pixel = 3U;
        conversion = cv::COLOR_RGB2BGR;
    }
    else if (message.encoding == sensor_msgs::image_encodings::MONO8)
    {
        cv_type = CV_8UC1;
        bytes_per_pixel = 1U;
        conversion = cv::COLOR_GRAY2BGR;
    }
    else if (message.encoding == sensor_msgs::image_encodings::BGRA8)
    {
        cv_type = CV_8UC4;
        bytes_per_pixel = 4U;
        conversion = cv::COLOR_BGRA2BGR;
    }
    else if (message.encoding == sensor_msgs::image_encodings::RGBA8)
    {
        cv_type = CV_8UC4;
        bytes_per_pixel = 4U;
        conversion = cv::COLOR_RGBA2BGR;
    }
    else
    {
        error = "unsupported encoding='" + message.encoding + "'";
        return false;
    }

    const std::size_t minimum_step =
        static_cast<std::size_t>(message.width) * bytes_per_pixel;
    const std::size_t required_bytes =
        static_cast<std::size_t>(message.step) *
        static_cast<std::size_t>(message.height);
    if (
        message.width == 0U || message.height == 0U ||
        message.step < minimum_step || message.data.size() < required_bytes)
    {
        error =
            "invalid raw layout width=" + std::to_string(message.width) +
            " height=" + std::to_string(message.height) +
            " step=" + std::to_string(message.step) +
            " bytes=" + std::to_string(message.data.size());
        return false;
    }

    const cv::Mat view(
        static_cast<int>(message.height),
        static_cast<int>(message.width),
        cv_type,
        const_cast<unsigned char *>(message.data.data()),
        static_cast<std::size_t>(message.step));
    if (conversion < 0)
    {
        output = view.clone();
    }
    else
    {
        cv::cvtColor(view, output, conversion);
    }
    return !output.empty();
}
}  // namespace

YOLOV9MIT_Node::YOLOV9MIT_Node(const rclcpp::NodeOptions &options) : Node("yolov9mit_ros", options)
{
    const auto model_path = this->declare_parameter(
        "model_path",
        "/home/nvidia/Workspace/perception_ws/src/YOLOv9MIT-ROS/yolov9mit_ros/v9-s.vec2box.sim.engine");
    const auto min_iou = this->declare_parameter("min_iou", 0.5f);
    const auto min_confidence = this->declare_parameter("min_confidence", 0.5f);
    const auto class_label_path = this->declare_parameter("class_label_path", "");
    const auto model_type = this->declare_parameter("model_type", "tensorrt");
    const auto tensorrt_device = this->declare_parameter("tensorrt_device", 0);
    const auto input_image_topic =
        this->declare_parameter("input_image_topic", "/camera/image_rect");
    const auto transport_hint = this->declare_parameter("transport_hint", "compressed");
    const auto output_image_topic =
        this->declare_parameter("output_image_topic", "yolov9mit_ros/image_raw");
    const auto output_boundingbox_topic =
        this->declare_parameter("output_boundingbox_topic", "yolov9mit_ros/detections");
    this->imshow_ = this->declare_parameter("imshow", false);
    const auto throttle_fps = this->declare_parameter("throttle_fps", 0.0);
    const auto max_compressed_payload_bytes = this->declare_parameter<int64_t>(
        "max_compressed_payload_bytes",
        static_cast<int64_t>(max_compressed_payload_bytes_));
    if (max_compressed_payload_bytes <= 0)
    {
        throw std::invalid_argument("max_compressed_payload_bytes must be positive");
    }
    max_compressed_payload_bytes_ =
        static_cast<std::size_t>(max_compressed_payload_bytes);
    if (throttle_fps > 0.0)
    {
        this->throttle_interval_ = std::chrono::duration<double>(1.0 / throttle_fps);
        this->last_inference_time_ =
            std::chrono::steady_clock::now() - std::chrono::seconds(10);
    }

    {
        RCLCPP_INFO(this->get_logger(), "Params: ");
        RCLCPP_INFO(this->get_logger(), " - model_path: %s", model_path.c_str());
        RCLCPP_INFO(this->get_logger(), " - min_iou: %f", min_iou);
        RCLCPP_INFO(this->get_logger(), " - min_confidence: %f", min_confidence);
        RCLCPP_INFO(this->get_logger(), " - class_label_path: %s", class_label_path.c_str());
        RCLCPP_INFO(this->get_logger(), " - model_type: %s", model_type.c_str());
        RCLCPP_INFO(this->get_logger(), " - tensorrt_device: %ld", tensorrt_device);
        RCLCPP_INFO(this->get_logger(), " - input_image_topic: %s", input_image_topic.c_str());
        RCLCPP_INFO(this->get_logger(), " - transport_hint: %s", transport_hint.c_str());
        RCLCPP_INFO(this->get_logger(), " - output_image_topic: %s", output_image_topic.c_str());
        RCLCPP_INFO(this->get_logger(), " - output_boundingbox_topic: %s",
                    output_boundingbox_topic.c_str());
        RCLCPP_INFO(this->get_logger(), " - imshow: %s", imshow_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), " - throttle_fps: %f", throttle_fps);
    }

    {
        if (model_path == "")
        {
            std::string msg = "model_path is not set.";
            throw std::runtime_error(msg);
        }
        if (!std::filesystem::exists(model_path))
        {
            std::string msg = "model_path[" + model_path + "] is not exist.";
            throw std::runtime_error(msg);
        }

        if (class_label_path == "")
        {
            std::string msg = "class_label_path is not set.";
            throw std::runtime_error(msg);
        }
        if (!std::filesystem::exists(class_label_path))
        {
            std::string msg = "class_label_path[" + class_label_path + "] is not exist.";
            throw std::runtime_error(msg);
        }

        this->class_names_ = yolov9mit::utils::read_class_labels(class_label_path);

        this->pub_bboxes_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
            output_boundingbox_topic, 10);
        this->pub_image_ = image_transport::create_publisher(this, output_image_topic);
        auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
        // HJ_260529: when transport_hint=="compressed", subscribe directly to CompressedImage
        // (camera_front_publisher_node uses a direct publisher, not image_transport)
        if (transport_hint == "compressed") {
            this->sub_compressed_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                input_image_topic + "/compressed",
                qos,
                std::bind(&YOLOV9MIT_Node::compressed_image_callback, this, std::placeholders::_1));
        } else {
            this->sub_image_ = image_transport::create_subscription(
                this, input_image_topic,
                std::bind(&YOLOV9MIT_Node::image_callback, this, std::placeholders::_1),
                transport_hint, qos.get_rmw_qos_profile());
        }

        if (this->imshow_)
        {
            cv::namedWindow(this->window_name_, cv::WINDOW_NORMAL);
        }
    }

    {
        if (model_type == "tensorrt")
        {
#ifdef ENABLE_TENSORRT
            RCLCPP_INFO(this->get_logger(), "Model Type is TensorRT");
            this->yolo_ = std::make_unique<yolov9mit::YOLOV9MIT_TensorRT>(
                model_path, tensorrt_device, min_iou, min_confidence, this->class_names_.size());
#else
            RCLCPP_ERROR(this->get_logger(), "yolov9mit is not built with TensorRT");
            rclcpp::shutdown();
#endif
        }
        if (!this->yolo_)
        {
            RCLCPP_ERROR(this->get_logger(), "yolov9mit is not initialized.");
            rclcpp::shutdown();
        }
    }

    RCLCPP_INFO(this->get_logger(), "initialized.");
}

void YOLOV9MIT_Node::process_image(const cv::Mat & image, const std_msgs::msg::Header & header)
{
    auto t0_inf = std::chrono::system_clock::now();
    const auto objects = this->yolo_->inference(image);
    auto t1_inf = std::chrono::system_clock::now();

    const auto bboxes = objects_to_bboxes(objects, header);
    this->pub_bboxes_->publish(*bboxes);

    const bool publish_image = this->pub_image_.getNumSubscribers() > 0 || this->imshow_;
    if (publish_image)
    {
        cv::Mat draw = image.clone();
        yolov9mit::utils::draw_objects(draw, objects, this->class_names_);
        // HH_260730 / TODOLIST 2 - Construct the ROS image directly so this
        // component does not pull the workspace OpenCV 4.5 cv_bridge into the
        // same process as the TensorRT/OpenCV 4.8 detector.
        const auto pub_img_msg = makeBgrImageMessage(draw, header);
        this->pub_image_.publish(pub_img_msg);
    }

    auto inf_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t1_inf - t0_inf);
    RCLCPP_DEBUG(this->get_logger(), "Inference: %.3f ms | Detections: %ld",
                 (float)inf_elapsed.count() * 0.001, objects.size());

    if (this->imshow_)
    {
        cv::Mat draw = image.clone();
        yolov9mit::utils::draw_objects(draw, objects, this->class_names_);
        cv::imshow(this->window_name_, draw);
        if (cv::waitKey(1) == 113) { cv::destroyWindow(this->window_name_); rclcpp::shutdown(); }
    }
}

void YOLOV9MIT_Node::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    if (throttle_interval_.count() > 0.0)
    {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_inference_time_ < throttle_interval_) return;
        last_inference_time_ = now;
    }
    cv::Mat image;
    std::string error;
    if (!decodeRawImage(*msg, image, error))
    {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Dropping raw image: %s", error.c_str());
        return;
    }
    process_image(image, msg->header);
}

// HJ_260529: direct CompressedImage callback — decodes JPEG then runs inference
void YOLOV9MIT_Node::compressed_image_callback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg)
{
    if (!msg)
    {
        log_compressed_frame_error(msg, "input", "null CompressedImage pointer");
        return;
    }
    if (msg->data.empty())
    {
        log_compressed_frame_error(msg, "input", "empty compressed payload");
        return;
    }
    if (msg->data.size() > max_compressed_payload_bytes_)
    {
        log_compressed_frame_error(
            msg, "input",
            "payload exceeds max_compressed_payload_bytes=" +
            std::to_string(max_compressed_payload_bytes_));
        return;
    }

    if (throttle_interval_.count() > 0.0)
    {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_inference_time_ < throttle_interval_) return;
        last_inference_time_ = now;
    }

    try
    {
        // HH_260730 / TODOLIST 2 - Decode with the detector's one OpenCV ABI.
        // cv_bridge linked a second OpenCV ABI into this component container.
        const cv::Mat encoded(
            1, static_cast<int>(msg->data.size()), CV_8UC1,
            const_cast<unsigned char *>(msg->data.data()));
        const cv::Mat decoded = cv::imdecode(encoded, cv::IMREAD_COLOR);
        if (decoded.empty())
        {
            log_compressed_frame_error(
                msg, "decode", "OpenCV returned an empty image");
            return;
        }
        if (decoded.type() != CV_8UC3)
        {
            log_compressed_frame_error(
                msg, "decode",
                "unexpected decoded type=" + std::to_string(decoded.type()) +
                " (expected CV_8UC3)");
            return;
        }

        process_image(decoded, msg->header);
    }
    catch (const cv::Exception & e)
    {
        log_compressed_frame_error(msg, "opencv", e.what());
    }
    catch (const std::exception & e)
    {
        log_compressed_frame_error(msg, "processing", e.what());
    }
    catch (...)
    {
        log_compressed_frame_error(msg, "processing", "unknown exception");
    }
}

void YOLOV9MIT_Node::log_compressed_frame_error(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg,
    const char * stage,
    const std::string & detail)
{
    const size_t payload_size = msg ? msg->data.size() : 0U;
    const char * format = msg ? msg->format.c_str() : "<null>";
    const char * frame_id = msg ? msg->header.frame_id.c_str() : "<null>";
    const int32_t stamp_sec = msg ? msg->header.stamp.sec : 0;
    const uint32_t stamp_nanosec = msg ? msg->header.stamp.nanosec : 0U;

    // HH_260729 - A malformed camera frame must not terminate the component
    // container shared by the camera publisher and YOLO node.
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dropping compressed image: stage=%s detail='%s' bytes=%zu "
        "format='%s' frame_id='%s' stamp=%d.%09u",
        stage, detail.c_str(), payload_size, format, frame_id,
        stamp_sec, stamp_nanosec);
}

vision_msgs::msg::Detection2DArray::SharedPtr YOLOV9MIT_Node::objects_to_bboxes(
    const std::vector<yolov9mit::Object> &objects, const std_msgs::msg::Header &header)
{
    vision_msgs::msg::Detection2DArray::SharedPtr msg(new vision_msgs::msg::Detection2DArray);
    msg->header = header;

    const auto objects_size = objects.size();
    msg->detections.resize(objects_size);
    for (size_t i = 0; i < objects_size; ++i)
    {
        const auto &obj = objects[i];
        vision_msgs::msg::Detection2D det;
        det.header = header;
        det.id = class_names_[obj.class_id];
        det.bbox.center.position.x = obj.rect.x + obj.rect.width * 0.5;
        det.bbox.center.position.y = obj.rect.y + obj.rect.height * 0.5;
        det.bbox.size_x = obj.rect.width;
        det.bbox.size_y = obj.rect.height;
        vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
        // HH_260723 - Publish the semantic label, not the TensorRT class-array index.
        // Downstream fusion and campsite occupancy must be able to distinguish
        // e.g. "tent" without duplicating this model's label table.
        hypothesis.hypothesis.class_id = class_names_[obj.class_id];
        hypothesis.hypothesis.score = (double)obj.confidence;
        det.results.push_back(hypothesis);

        msg->detections[i] = det;
    }
    return msg;
}
} // namespace yolov9mit_ros

RCLCPP_COMPONENTS_REGISTER_NODE(yolov9mit_ros::YOLOV9MIT_Node)
