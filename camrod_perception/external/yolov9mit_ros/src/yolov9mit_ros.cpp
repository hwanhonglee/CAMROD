#include "yolov9mit_ros/yolov9mit_ros.hpp"

#include <exception>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include "yolov9mit/utils.hpp"
#include "yolov9mit_ros/cv_bridge_include.hpp"

// HJ_260529: cv_bridge CompressedImage support
#include <cv_bridge/cv_bridge.h>

namespace yolov9mit_ros
{

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
        const auto pub_img_msg = cv_bridge::CvImage(header, "bgr8", draw).toImageMsg();
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
    const cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
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

    if (throttle_interval_.count() > 0.0)
    {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_inference_time_ < throttle_interval_) return;
        last_inference_time_ = now;
    }

    try
    {
        const auto converted = cv_bridge::toCvCopy(msg, "bgr8");
        if (!converted || converted->image.empty())
        {
            log_compressed_frame_error(
                msg, "decode", "cv_bridge returned an empty image");
            return;
        }
        if (converted->image.type() != CV_8UC3)
        {
            log_compressed_frame_error(
                msg, "decode",
                "unexpected decoded type=" + std::to_string(converted->image.type()) +
                " (expected CV_8UC3)");
            return;
        }

        process_image(converted->image, msg->header);
    }
    catch (const cv_bridge::Exception & e)
    {
        log_compressed_frame_error(msg, "cv_bridge", e.what());
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
