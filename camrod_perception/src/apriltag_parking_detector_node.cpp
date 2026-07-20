// apriltag_parking_detector_node.cpp
// HH_260720 - Integrate rear-camera parking-tag perception under camrod_perception.
//
// HH_260720 - Pipeline: rectified image -> grayscale -> AprilTag detection ->
// IPPE square pose -> reprojection validation -> PoseStamped and optional TF.
// The launch owns image rectification and this node uses CameraInfo.P for pose estimation.

#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <avg_msgs/conversions.hpp>
#include <avg_msgs/msg/avg_april_tag_pose.hpp>
#include <avg_msgs/msg/avg_bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tagStandard41h12.h>
}

class AprilTagParkingDetectorNode : public rclcpp::Node
{
public:
  AprilTagParkingDetectorNode()
  : Node("apriltag_parking_detector")
  {
    // HH_260720 - Declare descriptive rear-camera parking detector parameters.
    image_topic_      = declare_parameter<std::string>("image_topic",
                          "/sensing/camera/econ_rear/image_rect");
    info_topic_       = declare_parameter<std::string>("camera_info_topic",
                          "/sensing/camera/econ_rear/camera_info");
    tag_family_       = declare_parameter<std::string>("tag_family", "tag36h11");
    target_tag_id_    = declare_parameter<int>("target_tag_id", 0);  // -1 accepts every tag.
    tag_size_m_       = declare_parameter<double>("tag_size", 0.15);  // Black-border size in meters.
    quad_decimate_    = declare_parameter<double>("quad_decimate", 2.0);
    n_threads_        = declare_parameter<int>("n_threads", 2);
    camera_frame_id_  = declare_parameter<std::string>("camera_frame_id", "camera_rear");
    publish_tf_       = declare_parameter<bool>("publish_tf", true);
    max_reproj_error_ = declare_parameter<double>("max_reproj_error_px", 2.0);
    publish_debug_image_ = declare_parameter<bool>("publish_debug_image", true);
    debug_jpeg_quality_  = declare_parameter<int>("debug_jpeg_quality", 80);

    // HH_260720 - Track the previous detection ROI and periodically reacquire globally.
    roi_scale_ = declare_parameter<double>("roi_scale", 3.0);
    roi_full_search_interval_ =
      declare_parameter<int>("roi_full_search_interval", 30);

    // HH_260720 - Initialize the selected AprilTag family.
    if (tag_family_ == "tag36h11") {
      tf_family_ = tag36h11_create();
      family_destroy_fn_ = tag36h11_destroy;
    } else if (tag_family_ == "tagStandard41h12") {
      tf_family_ = tagStandard41h12_create();
      family_destroy_fn_ = tagStandard41h12_destroy;
    } else {
      RCLCPP_FATAL(get_logger(), "unsupported tag family: %s", tag_family_.c_str());
      throw std::runtime_error("unsupported tag family");
    }

    td_ = apriltag_detector_create();
    apriltag_detector_add_family(td_, tf_family_);
    td_->quad_decimate = static_cast<float>(quad_decimate_);
    td_->quad_sigma    = 0.0f;
    td_->nthreads      = n_threads_;
    td_->refine_edges  = 1;

    // HH_260720 - Match IPPE square points to apriltag's detected corner order.
    const double s = tag_size_m_ / 2.0;
    obj_pts_ = {
      { -s,  s, 0.0 },
      {  s,  s, 0.0 },
      {  s, -s, 0.0 },
      { -s, -s, 0.0 }
    };

    // HH_260720 - Match the camera driver's best-effort sensor-data QoS.
    auto sensor_qos = rclcpp::SensorDataQoS();

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic_, sensor_qos,
      // HH_260720 - Bind callbacks to the renamed parking-specific detector class.
      std::bind(&AprilTagParkingDetectorNode::infoCallback, this, std::placeholders::_1));

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, sensor_qos,
      std::bind(&AprilTagParkingDetectorNode::imageCallback, this, std::placeholders::_1));

    // HH_260720 - Parking perception publishes generated CAMROD interfaces.
    pose_pub_ = create_publisher<avg_msgs::msg::AvgAprilTagPose>("~/tag_pose", 10);
    detected_pub_ = create_publisher<avg_msgs::msg::AvgBool>("~/tag_detected", 10);
    if (publish_debug_image_) {
      debug_img_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug_image", 1);
      debug_img_compressed_pub_ =
        create_publisher<sensor_msgs::msg::CompressedImage>("~/debug_image/compressed", 1);
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(),
      "AprilTag parking detector ready: family=%s target_id=%d size=%.3fm decimate=%.1f",
      tag_family_.c_str(), target_tag_id_, tag_size_m_, quad_decimate_);
  }

  ~AprilTagParkingDetectorNode() override
  {
    if (td_) apriltag_detector_destroy(td_);
    if (tf_family_ && family_destroy_fn_) family_destroy_fn_(tf_family_);
  }

private:
  // HH_260720 - Cache the projection matrix for the rectified image stream.
  void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (calib_ready_) return;

    // HH_260720 - Rectified pixels use CameraInfo.P and no distortion coefficients.
    if (msg->p[0] > 1e-6) {
      camera_matrix_ = (cv::Mat_<double>(3, 3) <<
            msg->p[0], msg->p[1], msg->p[2],
            msg->p[4], msg->p[5], msg->p[6],
            msg->p[8], msg->p[9], msg->p[10]);
    } else {
      // HH_260720 - Fall back to K for camera drivers that leave P empty.
      RCLCPP_WARN(get_logger(), "CameraInfo.P is empty; using K for rectified pixels");
      camera_matrix_ = (cv::Mat_<double>(3, 3) <<
            msg->k[0], msg->k[1], msg->k[2],
            msg->k[3], msg->k[4], msg->k[5],
            msg->k[6], msg->k[7], msg->k[8]);
    }

    calib_ready_ = true;
    RCLCPP_INFO(get_logger(),
      "rear camera calibration ready: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
      camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(1, 1),
      camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2));
  }

  // HH_260720 - Run detection, pose estimation, validation, and publication per image.
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!calib_ready_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "waiting for rear CameraInfo; skipping detection");
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "cv_bridge conversion failed (encoding='%s'): %s",
                            msg->encoding.c_str(), e.what());
      return;
    }
    const cv::Mat & gray = cv_ptr->image;
    if (gray.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "received empty rear image");
      return;
    }

    // HH_260720 - Use ROI tracking while forcing periodic full-frame reacquisition.
    bool use_roi = roi_valid_;
    if (use_roi && roi_full_search_interval_ > 0 &&
        ++frames_since_full_search_ >= roi_full_search_interval_) {
      use_roi = false;
    }

    cv::Rect roi = use_roi ? roi_ : cv::Rect();
    zarray_t * detections = detectInRegion(gray, roi);

    // HH_260720 - Fall back to a full-frame search immediately after an ROI miss.
    if (use_roi && !containsTarget(detections)) {
      apriltag_detections_destroy(detections);
      use_roi = false;
      detections = detectInRegion(gray, cv::Rect());
    }
    if (!use_roi) frames_since_full_search_ = 0;

    // HH_260720 - Render debug overlays only while a debug topic has subscribers.
    cv::Mat debug_img;
    const bool want_debug_raw = debug_img_pub_ &&
                                debug_img_pub_->get_subscription_count() > 0;
    const bool want_debug_compressed = debug_img_compressed_pub_ &&
                                       debug_img_compressed_pub_->get_subscription_count() > 0;
    const bool draw_debug = publish_debug_image_ &&
                            (want_debug_raw || want_debug_compressed);
    if (draw_debug) {
      cv::cvtColor(gray, debug_img, cv::COLOR_GRAY2BGR);
      if (use_roi) {
        cv::rectangle(debug_img, roi, cv::Scalar(255, 128, 0), 2);
      }
    }

    bool found = false;
    for (int i = 0; i < zarray_size(detections); ++i) {
      apriltag_detection_t * det;
      zarray_get(detections, i, &det);

      if (target_tag_id_ >= 0 && det->id != target_tag_id_) continue;

      // HH_260720 - Rectified corners require no additional distortion correction.
      std::vector<cv::Point2f> corners(4);
      for (int c = 0; c < 4; ++c) {
        corners[c] = cv::Point2f(
          static_cast<float>(det->p[c][0]),
          static_cast<float>(det->p[c][1]));
      }

      // HH_260720 - IPPE square returns both planar pose candidates.
      std::vector<cv::Mat> rvecs, tvecs;
      std::vector<double> reproj_errs;
      int n_sol = cv::solvePnPGeneric(
        obj_pts_, corners, camera_matrix_, cv::noArray(),
        rvecs, tvecs, false, cv::SOLVEPNP_IPPE_SQUARE,
        cv::noArray(), cv::noArray(), reproj_errs);

      if (n_sol < 1) continue;

      // HH_260720 - Select the candidate with minimum reprojection error.
      int best = 0;
      for (int s = 1; s < n_sol; ++s) {
        if (reproj_errs[s] < reproj_errs[best]) best = s;
      }

      if (reproj_errs[best] > max_reproj_error_) {
        RCLCPP_DEBUG(get_logger(), "id=%d reprojection error %.2fpx exceeds %.2fpx; rejected",
                     det->id, reproj_errs[best], max_reproj_error_);
        continue;
      }

      // HH_260720 - Use temporal rotation continuity when planar solutions are similarly likely.
      if (n_sol >= 2) {
        double ratio = reproj_errs[best == 0 ? 1 : 0] /
                       std::max(reproj_errs[best], 1e-9);
        if (ratio < 1.5 && has_prev_rvec_) {
          // HH_260720 - Prefer the candidate nearest the previous frame rotation.
          double d0 = cv::norm(rvecs[0] - prev_rvec_);
          double d1 = (n_sol >= 2) ? cv::norm(rvecs[1] - prev_rvec_) : 1e9;
          best = (d1 < d0) ? 1 : 0;
        }
      }
      prev_rvec_ = rvecs[best].clone();
      has_prev_rvec_ = true;

      if (draw_debug) {
        for (int c = 0; c < 4; ++c) {
          cv::line(debug_img, corners[c], corners[(c + 1) % 4],
                   cv::Scalar(0, 255, 0), 2);
          cv::circle(debug_img, corners[c], 4, cv::Scalar(0, 0, 255), -1);
        }
        // HH_260720 - Overlay tag ID, range, and camera-frame position.
        const cv::Mat & tv = tvecs[best];
        const double dist = cv::norm(tv);
        char label[96], pos_label[96];
        snprintf(label, sizeof(label), "id=%d  %.2fm  err=%.2fpx",
                 det->id, dist, reproj_errs[best]);
        snprintf(pos_label, sizeof(pos_label), "x=%.2f y=%.2f z=%.2f [m]",
                 tv.at<double>(0), tv.at<double>(1), tv.at<double>(2));
        const int cx = static_cast<int>(det->c[0]);
        const int cy = static_cast<int>(det->c[1]);
        cv::putText(debug_img, label, cv::Point(cx - 60, cy - 38),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        cv::putText(debug_img, pos_label, cv::Point(cx - 60, cy - 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
      }

      publishPose(msg->header.stamp, det->id, rvecs[best], tvecs[best]);
      updateRoi(corners, gray.size());
      found = true;
    }

    // HH_260720 - Report visible tags rejected by ID or reprojection filters.
    if (!found && zarray_size(detections) > 0) {
      std::string ids;
      for (int i = 0; i < zarray_size(detections); ++i) {
        apriltag_detection_t * det;
        zarray_get(detections, i, &det);
        ids += std::to_string(det->id) + " ";
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "visible tag IDs [%s] do not match target_tag_id=%d or failed reprojection validation",
        ids.c_str(), target_tag_id_);
    }

    apriltag_detections_destroy(detections);

    if (draw_debug) {
      if (want_debug_raw) {
        auto dbg_msg = cv_bridge::CvImage(msg->header,
                                          sensor_msgs::image_encodings::BGR8,
                                          debug_img).toImageMsg();
        debug_img_pub_->publish(*dbg_msg);
      }
      if (want_debug_compressed) {
        sensor_msgs::msg::CompressedImage comp;
        comp.header = msg->header;
        comp.format = "jpeg";
        cv::imencode(".jpg", debug_img, comp.data,
                     {cv::IMWRITE_JPEG_QUALITY, debug_jpeg_quality_});
        debug_img_compressed_pub_->publish(comp);
      }
    }

    avg_msgs::msg::AvgBool det_msg;
    det_msg.data = found;
    detected_pub_->publish(det_msg);

    if (!found) {
      // HH_260720 - Reset continuity and ROI tracking after a full detection miss.
      has_prev_rvec_ = false;
      roi_valid_ = false;
    }
  }

  // HH_260720 - Detect in an optional ROI and restore detections to full-image coordinates.
  zarray_t * detectInRegion(const cv::Mat & gray, const cv::Rect & roi)
  {
    cv::Mat view = (roi.area() > 0) ? gray(roi) : gray;
    // HH_260720 - Positional aggregate initialization supports apriltag's const dimensions in C++17.
    image_u8_t im{
      view.cols,
      view.rows,
      static_cast<int32_t>(view.step),
      view.data
    };
    zarray_t * detections = apriltag_detector_detect(td_, &im);

    if (roi.area() > 0) {
      for (int i = 0; i < zarray_size(detections); ++i) {
        apriltag_detection_t * det;
        zarray_get(detections, i, &det);
        for (int c = 0; c < 4; ++c) {
          det->p[c][0] += roi.x;
          det->p[c][1] += roi.y;
        }
        det->c[0] += roi.x;
        det->c[1] += roi.y;
      }
    }
    return detections;
  }

  bool containsTarget(zarray_t * detections) const
  {
    for (int i = 0; i < zarray_size(detections); ++i) {
      apriltag_detection_t * det;
      zarray_get(detections, i, &det);
      if (target_tag_id_ < 0 || det->id == target_tag_id_) return true;
    }
    return false;
  }

  // HH_260720 - Expand the tag bounding box into the next square tracking ROI.
  void updateRoi(const std::vector<cv::Point2f> & corners, const cv::Size & img_size)
  {
    cv::Rect2f bbox = cv::boundingRect(corners);
    const float side = std::max(bbox.width, bbox.height) * static_cast<float>(roi_scale_);
    const float cx = bbox.x + bbox.width  / 2.0f;
    const float cy = bbox.y + bbox.height / 2.0f;
    cv::Rect roi(static_cast<int>(cx - side / 2.0f),
                 static_cast<int>(cy - side / 2.0f),
                 static_cast<int>(side), static_cast<int>(side));
    roi &= cv::Rect(0, 0, img_size.width, img_size.height);
    // HH_260720 - Reject ROIs too small for stable quad detection.
    roi_valid_ = (roi.width >= 32 && roi.height >= 32);
    roi_ = roi;
  }

  // HH_260720 - Publish camera-frame tag pose and optional named TF.
  void publishPose(const rclcpp::Time & stamp, int tag_id,
                   const cv::Mat & rvec, const cv::Mat & tvec)
  {
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    tf2::Matrix3x3 tf_R(
      R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
      R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
      R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
    tf2::Quaternion q;
    tf_R.getRotation(q);
    q.normalize();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = camera_frame_id_;
    pose.pose.position.x = tvec.at<double>(0);
    pose.pose.position.y = tvec.at<double>(1);
    pose.pose.position.z = tvec.at<double>(2);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    avg_msgs::msg::AvgAprilTagPose tag_pose;
    tag_pose.header = avg_msgs::conversions::fromRos(pose.header);
    tag_pose.family = tag_family_;
    tag_pose.id = tag_id;
    tag_pose.tag_frame = "parking_tag_" + std::to_string(tag_id);
    tag_pose.pose = avg_msgs::conversions::fromRos(pose);
    pose_pub_->publish(tag_pose);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = stamp;
      t.header.frame_id = camera_frame_id_;
      // HH_260720 - Use parking terminology because this transform is not a charger manager.
      t.child_frame_id = "parking_tag_" + std::to_string(tag_id);
      t.transform.translation.x = pose.pose.position.x;
      t.transform.translation.y = pose.pose.position.y;
      t.transform.translation.z = pose.pose.position.z;
      t.transform.rotation = pose.pose.orientation;
      tf_broadcaster_->sendTransform(t);
    }
  }

  // HH_260720 - Detector parameters and runtime state.
  std::string image_topic_, info_topic_, tag_family_, camera_frame_id_;
  int target_tag_id_{0};
  double tag_size_m_{0.15}, quad_decimate_{2.0}, max_reproj_error_{2.0};
  int n_threads_{2};
  bool publish_tf_{true};
  bool publish_debug_image_{true};
  int debug_jpeg_quality_{80};
  double roi_scale_{3.0};
  int roi_full_search_interval_{30};

  // HH_260720 - ROI tracking state.
  cv::Rect roi_;
  bool roi_valid_{false};
  int frames_since_full_search_{0};

  // apriltag
  apriltag_detector_t * td_{nullptr};
  apriltag_family_t * tf_family_{nullptr};
  void (*family_destroy_fn_)(apriltag_family_t *){nullptr};

  // HH_260720 - Projection matrix for rectified image pixels.
  cv::Mat camera_matrix_;
  bool calib_ready_{false};

  // HH_260720 - Temporal state for planar pose ambiguity resolution.
  cv::Mat prev_rvec_;
  bool has_prev_rvec_{false};

  // HH_260720 - Tag-local 3D corner geometry.
  std::vector<cv::Point3d> obj_pts_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<avg_msgs::msg::AvgAprilTagPose>::SharedPtr pose_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgBool>::SharedPtr detected_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_img_compressed_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagParkingDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
