#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <set>
#include <string>
#include <vector>

// HH_260720 - Name sensor/vision ROS pipeline boundaries directly instead of avg_msgs aliases.
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp> // HJ_260529
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include "camrod_perception/classified_detection.hpp"

// Coordinate frames
//   LiDAR (vanjee_lidar_750): raw X forward, Y left, Z up
//     lidar_preprocessor sets frame_id="lidar_link" but does NOT rotate points,
//     so raw Vanjee axes are preserved in points_filtered.
//   CCW-90° around Z: eff_X = -raw_Y (right), eff_Y = raw_X (fwd), eff_Z =
//   raw_Z (up) Camera: X right, Y down, Z forward
//
// Extrinsic R (LiDAR eff → camera):
//   R = [[1, 0,  0],   cam_X =  eff_X
//        [0, 0, -1],   cam_Y = -eff_Z
//        [0, 1,  0]]   cam_Z =  eff_Y
//   t = -R * [extrinsic_x, extrinsic_y, extrinsic_z]^T
//   default: extrinsic_z = -0.075 (camera 7.5 cm below LiDAR)

class CameraLidarFusionNode : public rclcpp::Node
{
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::CompressedImage>;   // HJ_260529: compressed image

public:
  CameraLidarFusionNode()
  : Node("obstacle_fusion"), cam_ready_(false)
  {
    // Clustering / tracking parameters
    n_closest_ = declare_parameter<int>("n_closest", 5);
    min_pts_ = declare_parameter<int>("min_pts_in_bbox", 3);
    ema_alpha_ = declare_parameter<double>("ema_alpha", 0.4);
    assoc_dist_ = declare_parameter<double>("assoc_dist", 1.0);
    max_miss_ = declare_parameter<int>("max_miss", 5);

    // HJ_260529: expose min forward distance as a ROS parameter (replaces
    // hardcoded 0.3 m). Points closer than this in the effective forward axis
    // (eff_Y = raw_X) are ignored.
    lidar_min_forward_m_ = static_cast<float>(
      declare_parameter<double>("lidar_min_forward_m", 1.0));

    // Extrinsic translation: camera position relative to LiDAR effective frame
    // [m]
    extrinsic_x_ = declare_parameter<double>("extrinsic_x", 0.0);
    extrinsic_y_ = declare_parameter<double>("extrinsic_y", 0.0);
    extrinsic_z_ = declare_parameter<double>("extrinsic_z", -0.075);

    // Topic parameters
    input_cloud_topic_ = declare_parameter<std::string>(
      "input_cloud_topic", "/sensing/lidar/points_filtered");
    detection_topic_ = declare_parameter<std::string>(
      "detection_topic", "/perception/camera/detections_2d");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/sensing/camera/processed/camera_info");
    image_topic_ = declare_parameter<std::string>(
      "image_topic", "/sensing/camera/processed/image");
    bbox_topic_ = declare_parameter<std::string>(
      "bbox_topic",
      "/perception/lidar/bboxes");
    output_topic_ =
      declare_parameter<std::string>("output_topic", "/perception/obstacles");
    out_image_topic_ = declare_parameter<std::string>(
      "out_image_topic", "/perception/camera_lidar/image");
    out_det3d_topic_ = declare_parameter<std::string>(
      "out_det3d_topic", "/perception/camera_lidar/detections_3d");
    out_markers_topic_ = declare_parameter<std::string>(
      "out_markers_topic", "/perception/camera_lidar/markers");
    out_euclidean_topic_ = declare_parameter<std::string>(
      "out_euclidean_topic", "/perception/camera_lidar/euclidean_markers");
    // HH_260707: Keep fusion outputs enabled while avoiding stale image/cloud
    // backlog and expensive debug image work when RViz is not consuming it.
    sync_queue_size_ = std::max(
      1, static_cast<int>(declare_parameter<int>("sync_queue_size", 8)));
    debug_image_publish_rate_hz_ =
      declare_parameter<double>("debug_image_publish_rate_hz", 2.0);
    debug_draw_stride_ = std::max(
      1, static_cast<int>(declare_parameter<int>("debug_draw_stride", 4)));
    publish_debug_image_without_subscribers_ = declare_parameter<bool>(
      "publish_debug_image_without_subscribers", false);
    // HH_260818 - The safety-facing obstacle cloud accepts only current YOLO
    // classes. Unmatched Euclidean clusters stay on visualization topics.
    detection_max_age_s_ = std::clamp(
      declare_parameter<double>("detection_max_age_s", 0.50), 0.05, 2.0);
    const auto unknown_labels = declare_parameter<std::vector<std::string>>(
      "unknown_class_labels", {"", "?", "unknown"});
    for (const auto & label : unknown_labels) {
      unknown_class_labels_.insert(camrod_perception::NormalizeClassLabel(label));
    }
    image_width_ = declare_parameter<int>("image_width", 1920);
    image_height_ = declare_parameter<int>("image_height", 1080);

    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & p : params) {
          if (p.get_name() == "n_closest") {
            n_closest_ = p.as_int();
          }
          if (p.get_name() == "min_pts_in_bbox") {
            min_pts_ = p.as_int();
          }
          if (p.get_name() == "ema_alpha") {
            ema_alpha_ = p.as_double();
          }
          if (p.get_name() == "assoc_dist") {
            assoc_dist_ = p.as_double();
          }
          if (p.get_name() == "max_miss") {
            max_miss_ = p.as_int();
          }
          if (p.get_name() == "lidar_min_forward_m") {
            lidar_min_forward_m_ =
            static_cast<float>(p.as_double());       // HJ_260529
          }
          if (p.get_name() == "debug_image_publish_rate_hz") {
            debug_image_publish_rate_hz_ = p.as_double();
          }
          if (p.get_name() == "debug_draw_stride") {
            debug_draw_stride_ = std::max(1, static_cast<int>(p.as_int()));
          }
          if (p.get_name() == "publish_debug_image_without_subscribers") {
            publish_debug_image_without_subscribers_ = p.as_bool();
          }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

    initExtrinsic();
    initColorLut();

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg) {
        if (cam_ready_) {
          return;
        }
        P_ = (cv::Mat_<double>(3, 3) << msg->p[0], msg->p[1], msg->p[2],
          msg->p[4], msg->p[5], msg->p[6], msg->p[8], msg->p[9],
          msg->p[10]);
        D_zero_ = cv::Mat::zeros(4, 1, CV_64F);
        if (msg->width > 0 && msg->height > 0) {
          image_width_ = static_cast<int>(msg->width);
          image_height_ = static_cast<int>(msg->height);
        }
        cam_ready_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Camera info received: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
          msg->p[0], msg->p[5], msg->p[2], msg->p[6]);
      });

    det_sub_cache_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      detection_topic_, rclcpp::SensorDataQoS(),   // HJ_260529
      [this](const vision_msgs::msg::Detection2DArray::ConstSharedPtr & msg) {
        std::lock_guard<std::mutex> lock(det_mutex_);
        latest_det_ = msg;
        latest_det_receive_time_ = now();
      });

    euclidean_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      bbox_topic_, rclcpp::SensorDataQoS(),   // HJ_260529
      [this](
        const visualization_msgs::msg::MarkerArray::ConstSharedPtr & msg) {
        std::lock_guard<std::mutex> lock(euclidean_mutex_);
        latest_euclidean_ = msg;
      });

    // HJ_260529: both LiDAR and camera publish with SensorDataQoS (BEST_EFFORT)
    const auto sensor_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
    lidar_sub_.subscribe(this, input_cloud_topic_, sensor_qos);
    image_sub_.subscribe(this, image_topic_, sensor_qos);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(sync_queue_size_), lidar_sub_, image_sub_);
    sync_->registerCallback(
      std::bind(
        &CameraLidarFusionNode::callback, this,
        std::placeholders::_1,
        std::placeholders::_2));

    pub_obstacles_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
    pub_image_ =
      create_publisher<sensor_msgs::msg::Image>(out_image_topic_, 10);
    pub_det3d_ = create_publisher<vision_msgs::msg::Detection3DArray>(
      out_det3d_topic_, 10);
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      out_markers_topic_, 10);
    pub_euclidean_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      out_euclidean_topic_, 10);

    RCLCPP_INFO(
      get_logger(),
      "obstacle_fusion started: cloud=%s det=%s bbox=%s "
      "sync_queue=%d debug_image_rate=%.1fHz class_only=true max_det_age=%.2fs",
      input_cloud_topic_.c_str(), detection_topic_.c_str(),
      bbox_topic_.c_str(), sync_queue_size_,
      debug_image_publish_rate_hz_, detection_max_age_s_);
    RCLCPP_INFO(
      get_logger(), "extrinsic t=[%.3f, %.3f, %.3f]", extrinsic_x_,
      extrinsic_y_, extrinsic_z_);
  }

private:
  struct ProjPt
  {
    cv::Point2f uv;
    float x, y, z;    // camera frame [m]
    float lx, ly, lz; // raw LiDAR frame [m]
  };

  struct Track
  {
    std::string label;
    float lx, ly, lz;
    int miss_count;
    int id;
  };

  void initColorLut()
  {
    for (int i = 0; i <= 120; ++i) {
      cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(i, 255, 255));
      cv::Mat bgr;
      cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
      color_lut_[i] = cv::Scalar(bgr.data[0], bgr.data[1], bgr.data[2]);
    }
  }

  bool shouldPublishDebugImage()
  {
    if (!publish_debug_image_without_subscribers_ &&
      pub_image_->get_subscription_count() == 0 &&
      pub_image_->get_intra_process_subscription_count() == 0)
    {
      return false;
    }
    if (debug_image_publish_rate_hz_ <= 0.0) {
      return true;
    }
    const auto now_time = now();
    if (last_debug_image_pub_.nanoseconds() == 0) {
      last_debug_image_pub_ = now_time;
      return true;
    }
    const double period_s = 1.0 / std::max(0.1, debug_image_publish_rate_hz_);
    if ((now_time - last_debug_image_pub_).seconds() >= period_s) {
      last_debug_image_pub_ = now_time;
      return true;
    }
    return false;
  }

  void initExtrinsic()
  {
    cv::Mat R = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 0, -1, 0, 1, 0);
    cv::Mat p =
      (cv::Mat_<double>(3, 1) << extrinsic_x_, extrinsic_y_, extrinsic_z_);
    tvec_ = -R * p;
    cv::Rodrigues(R, rvec_);
  }

  void callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr
    & img_msg)                // HJ_260529
  {
    if (!cam_ready_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for camera_info on %s…",
        camera_info_topic_.c_str());
      return;
    }

    cloud_msg_frame_ = cloud_msg->header.frame_id;

    vision_msgs::msg::Detection2DArray::ConstSharedPtr det_msg;
    rclcpp::Time det_receive_time{0, 0, RCL_ROS_TIME};
    {
      std::lock_guard<std::mutex> lock(det_mutex_);
      det_msg = latest_det_;
      det_receive_time = latest_det_receive_time_;
    }
    if (det_msg && !detectionFresh(*det_msg, cloud_msg->header, det_receive_time)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring stale camera detections for safety obstacle cloud");
      det_msg.reset();
    }

    const bool publish_debug_image = shouldPublishDebugImage();
    cv::Mat img;
    int img_w = image_width_;
    int img_h = image_height_;
    if (publish_debug_image || img_w <= 0 || img_h <= 0) {
      img = cv_bridge::toCvCopy(img_msg, "bgr8")
        ->image;         // HJ_260529: cv_bridge handles CompressedImage
      img_w = img.cols;
      img_h = img.rows;
      image_width_ = img_w;
      image_height_ = img_h;
    }

    const auto proj = projectCloud(cloud_msg, img_w, img_h);
    if (publish_debug_image) {
      drawPoints(img, proj);
    }

    visualization_msgs::msg::MarkerArray markers;
    vision_msgs::msg::Detection3DArray out3d;
    if (det_msg) {
      out3d = associateDetections(
        proj, det_msg, publish_debug_image ? &img : nullptr, markers);
    }
    out3d.header.stamp = cloud_msg->header.stamp;
    out3d.header.frame_id = cloud_msg_frame_;

    publishObstacleCloud(proj, det_msg, cloud_msg->header);

    pub_det3d_->publish(out3d);
    pub_markers_->publish(markers);

    fuseEuclideanClusters(
      det_msg, cloud_msg->header.stamp, img_w, img_h,
      publish_debug_image ? &img : nullptr);

    if (publish_debug_image) {
      pub_image_->publish(
        *cv_bridge::CvImage(std_msgs::msg::Header{}, "bgr8", img)
        .toImageMsg());
    }
  }

  // Collects YOLO-bbox-filtered LiDAR points and publishes to output_topic_ for
  // Nav2 and cmd_vel safety cost.
  void publishObstacleCloud(
    const std::vector<ProjPt> & proj,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & det_msg,
    const std_msgs::msg::Header & header)
  {
    sensor_msgs::msg::PointCloud2 out;
    out.header = header;
    out.header.stamp = this->get_clock()->now();

    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2FieldsByString(1, "xyz");

    std::vector<std::array<float, 3>> pts;
    if (det_msg) {
      for (const auto & d2 : det_msg->detections) {
        const std::string hypothesis = d2.results.empty() ? "" :
          d2.results[0].hypothesis.class_id;
        const std::string label = camrod_perception::ResolveClassLabel(
          d2.id, hypothesis);
        if (!camrod_perception::IsClassifiedDetection(label, unknown_class_labels_)) {
          continue;
        }
        const float x0 = static_cast<float>(d2.bbox.center.position.x -
          d2.bbox.size_x / 2.0);
        const float x1 = static_cast<float>(d2.bbox.center.position.x +
          d2.bbox.size_x / 2.0);
        const float y0 = static_cast<float>(d2.bbox.center.position.y -
          d2.bbox.size_y / 2.0);
        const float y1 = static_cast<float>(d2.bbox.center.position.y +
          d2.bbox.size_y / 2.0);
        for (const auto & pp : proj) {
          if (pp.uv.x >= x0 && pp.uv.x <= x1 && pp.uv.y >= y0 &&
            pp.uv.y <= y1)
          {
            pts.push_back({pp.lx, pp.ly, pp.lz});
          }
        }
      }
    }

    mod.resize(pts.size());
    sensor_msgs::PointCloud2Iterator<float> ix(out, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(out, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(out, "z");
    for (const auto & p : pts) {
      *ix = p[0];
      ++ix;
      *iy = p[1];
      ++iy;
      *iz = p[2];
      ++iz;
    }
    out.is_dense = true;
    pub_obstacles_->publish(out);
  }

  bool detectionFresh(
    const vision_msgs::msg::Detection2DArray & detection,
    const std_msgs::msg::Header & cloud_header,
    const rclcpp::Time & receive_time) const
  {
    const auto stamp_seconds = [](const builtin_interfaces::msg::Time & stamp) {
        return static_cast<double>(stamp.sec) +
               static_cast<double>(stamp.nanosec) * 1.0e-9;
      };
    const double detection_stamp = stamp_seconds(detection.header.stamp);
    const double cloud_stamp = stamp_seconds(cloud_header.stamp);
    if (detection_stamp > 0.0 && cloud_stamp > 0.0) {
      return std::abs(cloud_stamp - detection_stamp) <= detection_max_age_s_;
    }
    return receive_time.nanoseconds() > 0 &&
           std::max(0.0, (now() - receive_time).seconds()) <= detection_max_age_s_;
  }

  std::vector<ProjPt>
  projectCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
    int img_w, int img_h)
  {
    scratch_obj_.clear();
    scratch_obj_.reserve(cloud_msg->width * cloud_msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (!std::isfinite(*iter_x)) {
        continue;
      }
      // CCW 90° around Z: Vanjee 750 raw (X fwd, Y left) → eff (X right, Y fwd)
      const float lx = -(*iter_y);
      const float ly = (*iter_x);
      const float lz = (*iter_z);
      if (ly < lidar_min_forward_m_) {
        continue; // HJ_260529
      }
      scratch_obj_.push_back({lx, ly, lz});
    }
    if (scratch_obj_.empty()) {
      // HH_260721 - Return an explicit empty projection result without formatter ambiguity.
      return std::vector<ProjPt>();
    }

    scratch_uv_.clear();
    cv::projectPoints(scratch_obj_, rvec_, tvec_, P_, D_zero_, scratch_uv_);

    std::vector<ProjPt> result;
    result.reserve(scratch_obj_.size());
    for (size_t i = 0; i < scratch_uv_.size(); ++i) {
      if (scratch_uv_[i].x < 0 || scratch_uv_[i].x >= img_w ||
        scratch_uv_[i].y < 0 || scratch_uv_[i].y >= img_h)
      {
        continue;
      }
      const auto & o = scratch_obj_[i];
      // o is in effective frame: eff_X=-raw_Y, eff_Y=raw_X, eff_Z=raw_Z
      // lx/ly/lz store raw sensor coords (raw_X=eff_Y, raw_Y=-eff_X,
      // raw_Z=eff_Z)
      result.push_back(
        {scratch_uv_[i], o.x,
          static_cast<float>(-o.z + extrinsic_z_), o.y, o.y, -o.x,
          o.z});
    }
    return result;
  }

  void drawPoints(cv::Mat & img, const std::vector<ProjPt> & pts)
  {
    constexpr float kMinD = 1.0f, kMaxD = 50.0f;
    const std::size_t stride =
      static_cast<std::size_t>(std::max(1, debug_draw_stride_));
    for (std::size_t i = 0; i < pts.size(); i += stride) {
      const auto & pp = pts[i];
      float t = std::clamp((pp.z - kMinD) / (kMaxD - kMinD), 0.0f, 1.0f);
      cv::circle(
        img, pp.uv, 2,
        color_lut_[static_cast<int>((1.0f - t) * 120.0f)], cv::FILLED);
    }
  }

  vision_msgs::msg::Detection3DArray associateDetections(
    const std::vector<ProjPt> & proj,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & det_msg,
    cv::Mat * img, visualization_msgs::msg::MarkerArray & markers)
  {
    const int kMinPts = min_pts_;
    const int kNClose = std::max(1, n_closest_);
    vision_msgs::msg::Detection3DArray out;

    visualization_msgs::msg::Marker del_all;
    del_all.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(del_all);

    std::vector<bool> matched(tracks_.size(), false);

    for (const auto & d2 : det_msg->detections) {
      const double cx = d2.bbox.center.position.x;
      const double cy = d2.bbox.center.position.y;
      const double bw = d2.bbox.size_x;
      const double bh = d2.bbox.size_y;
      const float x0 = static_cast<float>(cx - bw / 2);
      const float x1 = static_cast<float>(cx + bw / 2);
      const float y0 = static_cast<float>(cy - bh / 2);
      const float y1 = static_cast<float>(cy + bh / 2);

      const std::string label = !d2.id.empty() ? d2.id :
        (d2.results.empty() ? "?" : d2.results[0].hypothesis.class_id);
      struct BboxPt
      {
        float z, lx, ly, lz;
      };
      std::vector<BboxPt> bbox_pts;
      for (const auto & pp : proj) {
        if (pp.uv.x >= x0 && pp.uv.x <= x1 && pp.uv.y >= y0 && pp.uv.y <= y1) {
          bbox_pts.push_back({pp.z, pp.lx, pp.ly, pp.lz});
        }
      }

      const cv::Rect rect(static_cast<int>(x0), static_cast<int>(y0),
        static_cast<int>(bw), static_cast<int>(bh));

      if (static_cast<int>(bbox_pts.size()) < kMinPts) {
        if (img) {
          cv::rectangle(*img, rect, {0, 165, 255}, 2);
          cv::putText(
            *img, label,
            {static_cast<int>(x0), static_cast<int>(y0) - 5},
            cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 165, 255}, 2);
        }
        continue;
      }

      const size_t n_use =
        std::min(static_cast<size_t>(kNClose), bbox_pts.size());
      if (bbox_pts.size() > n_use) {
        std::nth_element(
          bbox_pts.begin(),
          bbox_pts.begin() + static_cast<std::ptrdiff_t>(n_use),
          bbox_pts.end(), [](const BboxPt & a, const BboxPt & b) {
            return a.z < b.z;
          });
      }

      float lsx = 0, lsy = 0, lsz = 0;
      for (size_t i = 0; i < n_use; ++i) {
        lsx += bbox_pts[i].lx;
        lsy += bbox_pts[i].ly;
        lsz += bbox_pts[i].lz;
      }
      float lpos_x = lsx / static_cast<float>(n_use);
      float lpos_y = lsy / static_cast<float>(n_use);
      float lpos_z = lsz / static_cast<float>(n_use);

      int best_idx = -1;
      float best_dist2 = static_cast<float>(assoc_dist_ * assoc_dist_);
      for (size_t ti = 0; ti < tracks_.size(); ++ti) {
        if (matched[ti] || tracks_[ti].label != label) {
          continue;
        }
        float dx = tracks_[ti].lx - lpos_x;
        float dy = tracks_[ti].ly - lpos_y;
        float dz = tracks_[ti].lz - lpos_z;
        float d2t = dx * dx + dy * dy + dz * dz;
        if (d2t < best_dist2) {
          best_dist2 = d2t;
          best_idx = static_cast<int>(ti);
        }
      }

      int marker_id;
      if (best_idx >= 0) {
        Track & tk = tracks_[best_idx];
        const float a = static_cast<float>(ema_alpha_);
        tk.lx = a * lpos_x + (1.0f - a) * tk.lx;
        tk.ly = a * lpos_y + (1.0f - a) * tk.ly;
        tk.lz = a * lpos_z + (1.0f - a) * tk.lz;
        tk.miss_count = 0;
        matched[best_idx] = true;
        lpos_x = tk.lx;
        lpos_y = tk.ly;
        lpos_z = tk.lz;
        marker_id = tk.id * 2;
      } else {
        tracks_.push_back({label, lpos_x, lpos_y, lpos_z, 0, next_track_id_++});
        matched.push_back(true);
        marker_id = tracks_.back().id * 2;
      }

      vision_msgs::msg::Detection3D d3;
      d3.header.frame_id = cloud_msg_frame_;
      d3.header.stamp = det_msg->header.stamp;
      d3.id = std::to_string(marker_id / 2);
      if (!d2.results.empty()) {
        auto res = d2.results[0];
        // HH_260723 - Detection3D is declared in the LiDAR cloud frame, so its pose must
        // remain in raw LiDAR coordinates. The previous camera-frame values
        // under a LiDAR header made map transforms and semantic geofencing invalid.
        res.pose.pose.position.x = lpos_x;
        res.pose.pose.position.y = lpos_y;
        res.pose.pose.position.z = lpos_z;
        res.pose.pose.orientation.w = 1.0;
        if (!d2.id.empty()) {
          res.hypothesis.class_id = d2.id;
        }
        d3.results.push_back(res);
      }
      d3.bbox.center.position.x = lpos_x;
      d3.bbox.center.position.y = lpos_y;
      d3.bbox.center.position.z = lpos_z;
      d3.bbox.center.orientation.w = 1.0;
      d3.bbox.size.x = 0.4;
      d3.bbox.size.y = 0.4;
      d3.bbox.size.z = 0.4;
      out.detections.push_back(d3);

      const rclcpp::Time stamp = det_msg->header.stamp;
      const float lidar_dist =
        std::sqrt(lpos_x * lpos_x + lpos_y * lpos_y + lpos_z * lpos_z);

      visualization_msgs::msg::Marker sphere;
      sphere.header.frame_id = cloud_msg_frame_;
      sphere.header.stamp = stamp;
      sphere.ns = "fusion_det";
      sphere.id = marker_id;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position.x = lpos_x;
      sphere.pose.position.y = lpos_y;
      sphere.pose.position.z = lpos_z;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.4;
      sphere.color.r = 0.0f;
      sphere.color.g = 1.0f;
      sphere.color.b = 0.0f;
      sphere.color.a = 0.8f;
      sphere.lifetime = rclcpp::Duration::from_seconds(0.2);
      markers.markers.push_back(sphere);

      char buf[64];
      std::snprintf(buf, sizeof(buf), "%s\n%.2f m", label.c_str(), lidar_dist);
      visualization_msgs::msg::Marker text;
      text.header.frame_id = cloud_msg_frame_;
      text.header.stamp = stamp;
      text.ns = "fusion_det";
      text.id = marker_id + 1;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = lpos_x;
      text.pose.position.y = lpos_y;
      text.pose.position.z = lpos_z + 0.5;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.3;
      text.color.r = 1.0f;
      text.color.g = 1.0f;
      text.color.b = 1.0f;
      text.color.a = 1.0f;
      text.text = buf;
      text.lifetime = rclcpp::Duration::from_seconds(0.2);
      markers.markers.push_back(text);

      if (img) {
        cv::rectangle(*img, rect, {0, 255, 0}, 2);
        std::snprintf(
          buf, sizeof(buf), "%s %.2fm (%zu pts)", label.c_str(),
          lidar_dist, bbox_pts.size());
        cv::putText(
          *img, buf, {static_cast<int>(x0), static_cast<int>(y0) - 5},
          cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 255, 0}, 2);
      }
    }

    for (size_t ti = 0; ti < tracks_.size(); ++ti) {
      if (!matched[ti]) {
        ++tracks_[ti].miss_count;
      }
    }
    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [this](const Track & t) {
          return t.miss_count > max_miss_;
        }),
      tracks_.end());

    return out;
  }

  void fuseEuclideanClusters(
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & det_msg,
    const rclcpp::Time & stamp, int img_w, int img_h, cv::Mat * img)
  {
    visualization_msgs::msg::MarkerArray::ConstSharedPtr euc_msg;
    {
      std::lock_guard<std::mutex> lock(euclidean_mutex_);
      euc_msg = latest_euclidean_;
    }
    if (!euc_msg || euc_msg->markers.empty()) {
      return;
    }

    struct ClusterInfo
    {
      float raw_x, raw_y, raw_z, dist;
      cv::Point2f uv;
      bool in_image;
    };
    std::vector<ClusterInfo> cls;
    cls.reserve(euc_msg->markers.size());

    for (const auto & mk : euc_msg->markers) {
      if (mk.action != visualization_msgs::msg::Marker::ADD) {
        continue;
      }
      if (mk.type != visualization_msgs::msg::Marker::CUBE) {
        continue;
      }

      const float raw_x = static_cast<float>(mk.pose.position.x);
      const float raw_y = static_cast<float>(mk.pose.position.y);
      const float raw_z = static_cast<float>(mk.pose.position.z);
      // Apply same CCW-90° rotation to convert cluster centroid to effective
      // frame
      const float lx = -raw_y, ly = raw_x, lz = raw_z;

      ClusterInfo ci;
      ci.raw_x = raw_x;
      ci.raw_y = raw_y;
      ci.raw_z = raw_z;
      ci.dist = std::sqrt(raw_x * raw_x + raw_y * raw_y + raw_z * raw_z);
      ci.in_image = false;

      if (ly >= lidar_min_forward_m_) { // HJ_260529
        std::vector<cv::Point3f> obj_pt = {{lx, ly, lz}};
        std::vector<cv::Point2f> img_pt;
        cv::projectPoints(obj_pt, rvec_, tvec_, P_, D_zero_, img_pt);
        ci.uv = img_pt[0];
        ci.in_image = (ci.uv.x >= 0 && ci.uv.x < img_w && ci.uv.y >= 0 &&
          ci.uv.y < img_h);
      }
      cls.push_back(ci);
    }

    std::vector<std::string> labels(cls.size(), "unknown");

    if (det_msg) {
      std::vector<bool> used(cls.size(), false);

      for (const auto & d2 : det_msg->detections) {
        const float cx = static_cast<float>(d2.bbox.center.position.x);
        const float cy = static_cast<float>(d2.bbox.center.position.y);
        const float hw = static_cast<float>(d2.bbox.size_x) * 0.5f;
        const float hh = static_cast<float>(d2.bbox.size_y) * 0.5f;
        const std::string lbl = !d2.id.empty() ? d2.id :
          (d2.results.empty() ? "?" : d2.results[0].hypothesis.class_id);
        int best = -1;
        float best_d = std::numeric_limits<float>::max();

        for (size_t i = 0; i < cls.size(); ++i) {
          if (!cls[i].in_image || used[i]) {
            continue;
          }
          const float mx = hw * 0.1f, my = hh * 0.1f;
          if (cls[i].uv.x<cx - hw - mx || cls[i].uv.x> cx + hw + mx) {
            continue;
          }
          if (cls[i].uv.y<cy - hh - my || cls[i].uv.y> cy + hh + my) {
            continue;
          }
          const float dx = cls[i].uv.x - cx, dy = cls[i].uv.y - cy;
          const float d = dx * dx + dy * dy;
          if (d < best_d) {
            best_d = d;
            best = static_cast<int>(i);
          }
        }

        if (best >= 0) {
          labels[best] = lbl;
          used[best] = true;
        }
      }
    }

    visualization_msgs::msg::MarkerArray out;
    visualization_msgs::msg::Marker del_all;
    del_all.action = visualization_msgs::msg::Marker::DELETEALL;
    out.markers.push_back(del_all);

    next_euclidean_id_ = 0;

    for (size_t i = 0; i < cls.size(); ++i) {
      const auto & ci = cls[i];
      const bool is_matched = (labels[i] != "unknown");
      const int base_id = (next_euclidean_id_++) * 2;

      visualization_msgs::msg::Marker sphere;
      sphere.header.frame_id = cloud_msg_frame_;
      sphere.header.stamp = stamp;
      sphere.ns = "euclidean_fusion";
      sphere.id = base_id;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position.x = ci.raw_x;
      sphere.pose.position.y = ci.raw_y;
      sphere.pose.position.z = ci.raw_z;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.4;
      sphere.color.r = is_matched ? 0.0f : 0.5f;
      sphere.color.g = is_matched ? 1.0f : 0.5f;
      sphere.color.b = is_matched ? 0.0f : 0.5f;
      sphere.color.a = is_matched ? 0.8f : 0.6f;
      sphere.lifetime = rclcpp::Duration::from_seconds(0.2);
      out.markers.push_back(sphere);

      char buf[64];
      std::snprintf(buf, sizeof(buf), "%s\n%.2f m", labels[i].c_str(), ci.dist);
      visualization_msgs::msg::Marker text;
      text.header.frame_id = cloud_msg_frame_;
      text.header.stamp = stamp;
      text.ns = "euclidean_fusion";
      text.id = base_id + 1;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = ci.raw_x;
      text.pose.position.y = ci.raw_y;
      text.pose.position.z = ci.raw_z + 0.5f;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.3;
      text.color.r = 1.0f;
      text.color.g = 1.0f;
      text.color.b = 1.0f;
      text.color.a = 1.0f;
      text.text = buf;
      text.lifetime = rclcpp::Duration::from_seconds(0.2);
      out.markers.push_back(text);

      if (img && ci.in_image) {
        const cv::Point pt(static_cast<int>(ci.uv.x),
          static_cast<int>(ci.uv.y));
        const cv::Scalar color =
          is_matched ? cv::Scalar(0, 255, 255) : cv::Scalar(128, 128, 128);
        cv::circle(*img, pt, 8, color, 2);
        cv::drawMarker(*img, pt, color, cv::MARKER_CROSS, 14, 2);
        char lbuf[64];
        std::snprintf(
          lbuf, sizeof(lbuf), "%s %.1fm", labels[i].c_str(),
          ci.dist);
        cv::putText(
          *img, lbuf, {pt.x + 10, pt.y - 5}, cv::FONT_HERSHEY_SIMPLEX,
          0.5, color, 2);
      }
    }

    pub_euclidean_->publish(out);
  }

  // --- parameters ---
  int n_closest_;
  int min_pts_;
  double ema_alpha_;
  double assoc_dist_;
  int max_miss_;
  float lidar_min_forward_m_; // HJ_260529
  double extrinsic_x_, extrinsic_y_, extrinsic_z_;
  int sync_queue_size_{8};
  double debug_image_publish_rate_hz_{2.0};
  int debug_draw_stride_{4};
  bool publish_debug_image_without_subscribers_{false};
  double detection_max_age_s_{0.50};
  std::set<std::string> unknown_class_labels_{"", "?", "unknown"};
  int image_width_{1920};
  int image_height_{1080};
  rclcpp::Time last_debug_image_pub_{0, 0, RCL_ROS_TIME};

  std::string input_cloud_topic_, detection_topic_, camera_info_topic_;
  std::string image_topic_, bbox_topic_, output_topic_;
  std::string out_image_topic_, out_det3d_topic_, out_markers_topic_,
    out_euclidean_topic_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // Cached frame_id from the most recent LiDAR message (used for marker
  // headers).
  std::string cloud_msg_frame_{"lidar_link"};

  // --- EMA tracks ---
  std::vector<Track> tracks_;
  int next_track_id_ = 0;
  int next_euclidean_id_ = 0;

  // --- calibration ---
  cv::Mat rvec_, tvec_;
  cv::Mat P_;
  cv::Mat D_zero_;
  std::atomic<bool> cam_ready_;

  std::array<cv::Scalar, 121> color_lut_;

  // --- scratch buffers (reused per frame) ---
  std::vector<cv::Point3f> scratch_obj_;
  std::vector<cv::Point2f> scratch_uv_;

  // --- detection cache ---
  vision_msgs::msg::Detection2DArray::ConstSharedPtr latest_det_;
  rclcpp::Time latest_det_receive_time_{0, 0, RCL_ROS_TIME};
  std::mutex det_mutex_;

  // --- Euclidean cluster cache ---
  visualization_msgs::msg::MarkerArray::ConstSharedPtr latest_euclidean_;
  std::mutex euclidean_mutex_;

  // --- subscribers ---
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
    det_sub_cache_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
    euclidean_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage>
  image_sub_;     // HJ_260529
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // --- publishers ---
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacles_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_det3d_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pub_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pub_euclidean_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraLidarFusionNode>());
  rclcpp::shutdown();
  return 0;
}
