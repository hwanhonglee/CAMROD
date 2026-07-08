#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <avg_msgs/conversions.hpp>
#include <avg_msgs/msg/avg_sensing_lidar.hpp>
#include <avg_msgs/msg/header.hpp>
#include <avg_msgs/msg/marker.hpp>
#include <avg_msgs/msg/marker_array.hpp>
#include <avg_msgs/msg/occupancy_grid.hpp>
#include <avg_msgs/msg/point_cloud2.hpp>
#include <avg_msgs/msg/point_stamped.hpp>
#include <avg_msgs/msg/transform_stamped.hpp>
#include <avg_msgs/point_cloud2_iterator.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace camrod::sensing {

class LidarCostGridNode : public rclcpp::Node {
public:
  using AvgSensingLidar = avg_msgs::msg::AvgSensingLidar;

  // Implements `LidarCostGridNode` behavior.
  LidarCostGridNode() : Node("lidar_cost_grid") {
    // HH_260421: Use filtered lidar points as default cost-grid input.
    input_topic_ = declare_parameter<std::string>(
        "input_topic", "/sensing/lidar/points_filtered");
    // HH_260702 - Merge perception obstacle clouds into the LiDAR cost grid so
    // camera/LiDAR-detected vehicles can affect planning before raw cloud-only
    // projection would mark the same cells.
    input_topics_ = declare_parameter<std::vector<std::string>>(
        "input_topics", std::vector<std::string>{});
    extra_input_topics_ = declare_parameter<std::vector<std::string>>(
        "extra_input_topics", std::vector<std::string>{});
    perception_marker_topics_ = declare_parameter<std::vector<std::string>>(
        "perception_marker_topics", std::vector<std::string>{});
    output_topic_ = declare_parameter<std::string>("output_topic",
                                                   "/sensing/cost_grid/lidar");
    base_frame_id_ =
        declare_parameter<std::string>("base_frame_id", "robot_base_link");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "map");
    // HH_260702 - Defaults match the field config: keep obstacles visible
    // far enough ahead for replanning before cmd_vel hard-stop range.
    resolution_ = declare_parameter<double>("resolution", 0.10);
    width_ = declare_parameter<int>("width", 180);
    height_ = declare_parameter<int>("height", 180);
    origin_x_ = declare_parameter<double>("origin_x", -9.0);
    origin_y_ = declare_parameter<double>("origin_y", -9.0);
    free_value_ = declare_parameter<int>("free_value", 0);
    unknown_value_ = declare_parameter<int>("unknown_value", -1);
    min_cost_ = declare_parameter<int>("min_cost", 65);
    max_cost_ = declare_parameter<int>("max_cost", 100);
    cost_range_min_m_ = declare_parameter<double>("cost_range_min_m", 0.4);
    cost_range_max_m_ = declare_parameter<double>("cost_range_max_m", 9.0);
    obstacle_radius_m_ = declare_parameter<double>("obstacle_radius_m", 0.20);
    cloud_min_z_m_ = declare_parameter<double>(
        "cloud_min_z_m", -std::numeric_limits<double>::infinity());
    cloud_max_z_m_ = declare_parameter<double>(
        "cloud_max_z_m", std::numeric_limits<double>::infinity());
    perception_marker_cost_ =
        declare_parameter<int>("perception_marker_cost", 90);
    perception_marker_min_radius_m_ =
        declare_parameter<double>("perception_marker_min_radius_m", 0.35);
    perception_marker_max_radius_m_ =
        declare_parameter<double>("perception_marker_max_radius_m", 0.75);
    perception_marker_radius_scale_ =
        declare_parameter<double>("perception_marker_radius_scale", 0.35);
    ego_clear_radius_m_ = declare_parameter<double>("ego_clear_radius_m", 0.90);
    max_message_age_s_ = declare_parameter<double>("max_message_age_s", 0.50);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    // HH_260707: Preserve output freshness while avoiding full grid rebuilds
    // when the latest LiDAR/perception inputs and rolling-grid origin are
    // stable.
    rebuild_min_pose_delta_m_ =
        declare_parameter<double>("rebuild_min_pose_delta_m", 0.05);
    lidar_status_topic_ = declare_parameter<std::string>(
        "lidar_status_topic", "/sensing/lidar/status");
    publish_lidar_status_ =
        declare_parameter<bool>("publish_lidar_status", false);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_grid_ = create_publisher<avg_msgs::msg::OccupancyGrid>(
        output_topic_, rclcpp::QoS(1).transient_local().reliable());
    avg_lidar_pub_ =
        create_publisher<AvgSensingLidar>(lidar_status_topic_, rclcpp::QoS(10));

    configureInputs();

    if (publish_rate_hz_ <= 0.0) {
      publish_rate_hz_ = 10.0;
    }
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&LidarCostGridNode::publishGrid, this));
  }

private:
  struct CloudInput {
    std::string topic;
    avg_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    rclcpp::Time rx_time{0, 0, RCL_ROS_TIME};
  };

  struct MarkerInput {
    std::string topic;
    avg_msgs::msg::MarkerArray::ConstSharedPtr markers;
    rclcpp::Time rx_time{0, 0, RCL_ROS_TIME};
  };

  // Implements `appendUniqueTopic` behavior.
  static void appendUniqueTopic(std::vector<std::string> &topics,
                                const std::string &topic) {
    if (topic.empty()) {
      return;
    }
    if (std::find(topics.begin(), topics.end(), topic) == topics.end()) {
      topics.push_back(topic);
    }
  }

  // Implements `configureInputs` behavior.
  void configureInputs() {
    if (input_topics_.empty()) {
      appendUniqueTopic(input_topics_, input_topic_);
      for (const auto &topic : extra_input_topics_) {
        appendUniqueTopic(input_topics_, topic);
      }
    }

    cloud_inputs_.resize(input_topics_.size());
    for (std::size_t i = 0; i < input_topics_.size(); ++i) {
      cloud_inputs_[i].topic = input_topics_[i];
      cloud_inputs_[i].rx_time =
          rclcpp::Time(0, 0, get_clock()->get_clock_type());
      sub_clouds_.push_back(create_subscription<avg_msgs::msg::PointCloud2>(
          input_topics_[i], rclcpp::SensorDataQoS(),
          [this, i](const avg_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            onCloud(i, msg);
          }));
    }

    marker_inputs_.resize(perception_marker_topics_.size());
    for (std::size_t i = 0; i < perception_marker_topics_.size(); ++i) {
      marker_inputs_[i].topic = perception_marker_topics_[i];
      marker_inputs_[i].rx_time =
          rclcpp::Time(0, 0, get_clock()->get_clock_type());
      sub_markers_.push_back(create_subscription<avg_msgs::msg::MarkerArray>(
          perception_marker_topics_[i], rclcpp::SensorDataQoS(),
          [this, i](const avg_msgs::msg::MarkerArray::ConstSharedPtr msg) {
            onMarkers(i, msg);
          }));
    }

    RCLCPP_INFO(
        get_logger(), "lidar_cost_grid: clouds=%zu markers=%zu output=%s",
        cloud_inputs_.size(), marker_inputs_.size(), output_topic_.c_str());
  }

  // Handles the `onCloud` callback.
  void onCloud(const std::size_t idx,
               const avg_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (!msg || idx >= cloud_inputs_.size()) {
      return;
    }
    cloud_inputs_[idx].cloud = msg;
    cloud_inputs_[idx].rx_time = now();
    ++input_sequence_;
    if (idx == 0) {
      latest_primary_cloud_ = msg;
    }
  }

  // Handles the `onMarkers` callback.
  void onMarkers(const std::size_t idx,
                 const avg_msgs::msg::MarkerArray::ConstSharedPtr msg) {
    if (!msg || idx >= marker_inputs_.size()) {
      return;
    }
    marker_inputs_[idx].markers = msg;
    marker_inputs_[idx].rx_time = now();
    ++input_sequence_;
  }

  // Implements `squaredDistance2d` behavior.
  static double squaredDistance2d(const double x1, const double y1,
                                  const double x2, const double y2) {
    const double dx = x1 - x2;
    const double dy = y1 - y2;
    return dx * dx + dy * dy;
  }

  // Implements `mapDistanceToCost` behavior.
  int mapDistanceToCost(const double distance_m) const {
    const double max_cost_range =
        std::max(cost_range_min_m_ + 1e-3, cost_range_max_m_);
    if (distance_m <= cost_range_min_m_) {
      return max_cost_;
    }
    if (distance_m >= max_cost_range) {
      return min_cost_;
    }
    const double norm =
        (distance_m - cost_range_min_m_) / (max_cost_range - cost_range_min_m_);
    const double inv = 1.0 - std::clamp(norm, 0.0, 1.0);
    return static_cast<int>(
        std::round(static_cast<double>(min_cost_) +
                   inv * static_cast<double>(max_cost_ - min_cost_)));
  }

  // Implements `markDisk` behavior.
  void markDiskWithRadius(avg_msgs::msg::OccupancyGrid &grid,
                          const double grid_origin_x,
                          const double grid_origin_y, const double x,
                          const double y, const double radius_m,
                          const int value) {
    const int cx =
        static_cast<int>(std::floor((x - grid_origin_x) / resolution_));
    const int cy =
        static_cast<int>(std::floor((y - grid_origin_y) / resolution_));
    const int radius_cells =
        static_cast<int>(std::ceil(std::max(0.0, radius_m) / resolution_));

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (dx * dx + dy * dy > radius_cells * radius_cells) {
          continue;
        }
        const int gx = cx + dx;
        const int gy = cy + dy;
        if (gx < 0 || gy < 0 || gx >= width_ || gy >= height_) {
          continue;
        }
        const std::size_t idx = static_cast<std::size_t>(gy * width_ + gx);
        if (grid.data[idx] < 0) {
          grid.data[idx] = static_cast<int8_t>(value);
        } else {
          grid.data[idx] =
              static_cast<int8_t>(std::max<int>(grid.data[idx], value));
        }
      }
    }
  }

  // Implements `markDisk` behavior.
  void markDisk(avg_msgs::msg::OccupancyGrid &grid, const double grid_origin_x,
                const double grid_origin_y, const double x, const double y,
                const int value) {
    markDiskWithRadius(grid, grid_origin_x, grid_origin_y, x, y,
                       obstacle_radius_m_, value);
  }

  // Implements `clearDisk` behavior.
  void clearDisk(avg_msgs::msg::OccupancyGrid &grid, const double grid_origin_x,
                 const double grid_origin_y, const double x, const double y) {
    const int cx =
        static_cast<int>(std::floor((x - grid_origin_x) / resolution_));
    const int cy =
        static_cast<int>(std::floor((y - grid_origin_y) / resolution_));
    const int radius_cells = static_cast<int>(
        std::ceil(std::max(0.0, ego_clear_radius_m_) / resolution_));

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (dx * dx + dy * dy > radius_cells * radius_cells) {
          continue;
        }
        const int gx = cx + dx;
        const int gy = cy + dy;
        if (gx < 0 || gy < 0 || gx >= width_ || gy >= height_) {
          continue;
        }
        const std::size_t idx = static_cast<std::size_t>(gy * width_ + gx);
        grid.data[idx] = static_cast<int8_t>(free_value_);
      }
    }
  }

  // Implements `getBasePoseInOutput` behavior.
  bool getBasePoseInOutput(avg_msgs::msg::PointStamped &base_in_output) {
    avg_msgs::msg::PointStamped base_origin;
    // HH_260315-00:00 Use latest TF for rolling grid anchoring.
    // Requesting "now()" can intermittently fail with small future
    // extrapolation during startup/high-load, which causes marker/grid flicker.
    base_origin.header.stamp =
        rclcpp::Time(0, 0, get_clock()->get_clock_type());
    base_origin.header.frame_id = base_frame_id_;
    base_origin.point.x = 0.0;
    base_origin.point.y = 0.0;
    base_origin.point.z = 0.0;

    try {
      base_in_output = tf_buffer_->transform(base_origin, output_frame_id_,
                                             tf2::durationFromSec(0.05));
      return true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "lidar_cost_grid failed to locate %s in %s: %s",
                           base_frame_id_.c_str(), output_frame_id_.c_str(),
                           ex.what());
      return false;
    }
  }

  // Implements `getCloudTransform` behavior.
  bool getCloudTransform(const avg_msgs::msg::Header &cloud_header,
                         avg_msgs::msg::TransformStamped &tf_out) {
    if (cloud_header.frame_id.empty()) {
      return false;
    }

    auto stamp = cloud_header.stamp;
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      stamp = now();
    }

    try {
      tf_out =
          tf_buffer_->lookupTransform(output_frame_id_, cloud_header.frame_id,
                                      stamp, tf2::durationFromSec(0.05));
      return true;
    } catch (const tf2::TransformException &) {
      // HH_260315-00:00 Fallback to latest TF when exact pointcloud stamp is
      // slightly ahead of TF buffer time.
      try {
        tf_out = tf_buffer_->lookupTransform(
            output_frame_id_, cloud_header.frame_id,
            rclcpp::Time(0, 0, get_clock()->get_clock_type()),
            tf2::durationFromSec(0.05));
        return true;
      } catch (const tf2::TransformException &ex_latest) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "lidar_cost_grid TF lookup failed (%s -> %s): %s",
                             cloud_header.frame_id.c_str(),
                             output_frame_id_.c_str(), ex_latest.what());
        return false;
      }
    }
  }

  // Implements `transformMarkerPoint` behavior.
  bool transformMarkerPoint(const avg_msgs::msg::Marker &marker,
                            avg_msgs::msg::PointStamped &point_out) {
    if (marker.header.frame_id.empty()) {
      return false;
    }

    avg_msgs::msg::PointStamped point;
    point.header = marker.header;
    if (point.header.stamp.sec == 0 && point.header.stamp.nanosec == 0) {
      point.header.stamp = now();
    }
    point.point.x = marker.pose.position.x;
    point.point.y = marker.pose.position.y;
    point.point.z = marker.pose.position.z;

    try {
      point_out = tf_buffer_->transform(point, output_frame_id_,
                                        tf2::durationFromSec(0.05));
      return true;
    } catch (const tf2::TransformException &) {
      point.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      try {
        point_out = tf_buffer_->transform(point, output_frame_id_,
                                          tf2::durationFromSec(0.05));
        return true;
      } catch (const tf2::TransformException &ex_latest) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "lidar_cost_grid marker TF failed (%s -> %s): %s",
                             marker.header.frame_id.c_str(),
                             output_frame_id_.c_str(), ex_latest.what());
        return false;
      }
    }
  }

  // Implements `markCloudInput` behavior.
  void markCloudInput(const CloudInput &input,
                      avg_msgs::msg::OccupancyGrid &grid,
                      const avg_msgs::msg::PointStamped &base_in_output,
                      const double grid_origin_x, const double grid_origin_y) {
    if (!input.cloud) {
      return;
    }

    avg_msgs::msg::TransformStamped cloud_tf_msg;
    if (!getCloudTransform(input.cloud->header, cloud_tf_msg)) {
      return;
    }

    tf2::Transform cloud_tf;
    tf2::fromMsg(cloud_tf_msg.transform, cloud_tf);

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*input.cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*input.cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*input.cloud, "z");
      // HH_260707: Deduplicate dense PointCloud2 hits by output-grid cell
      // before applying obstacle-radius disks. This preserves the max-cost
      // result while avoiding repeated markDisk() calls for many points in the
      // same cell.
      const int radius_cells = static_cast<int>(
          std::ceil(std::max(0.0, obstacle_radius_m_) / resolution_));
      const int dedup_width = width_ + radius_cells * 2;
      const int dedup_height = height_ + radius_cells * 2;
      std::vector<int> cell_costs(
          static_cast<std::size_t>(std::max(1, dedup_width) *
                                   std::max(1, dedup_height)),
          -1);

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const double x = static_cast<double>(*iter_x);
        const double y = static_cast<double>(*iter_y);
        const double z = static_cast<double>(*iter_z);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        // HH_260707 - Allow pre-ground-segmentation fallback clouds without
        // turning flat road returns into obstacle cost.
        if (z < cloud_min_z_m_ || z > cloud_max_z_m_) {
          continue;
        }

        const tf2::Vector3 p_sensor(x, y, z);
        const tf2::Vector3 p_output = cloud_tf * p_sensor;
        const double distance = std::sqrt(
            squaredDistance2d(p_output.x(), p_output.y(),
                              base_in_output.point.x, base_in_output.point.y));
        const int value = mapDistanceToCost(distance);
        const int cx = static_cast<int>(std::floor(
                           (p_output.x() - grid_origin_x) / resolution_)) +
                       radius_cells;
        const int cy = static_cast<int>(std::floor(
                           (p_output.y() - grid_origin_y) / resolution_)) +
                       radius_cells;
        if (cx < 0 || cy < 0 || cx >= dedup_width || cy >= dedup_height) {
          continue;
        }
        const std::size_t cell_idx =
            static_cast<std::size_t>(cy * dedup_width + cx);
        cell_costs[cell_idx] = std::max(cell_costs[cell_idx], value);
      }

      for (int cy = 0; cy < dedup_height; ++cy) {
        for (int cx = 0; cx < dedup_width; ++cx) {
          const int value =
              cell_costs[static_cast<std::size_t>(cy * dedup_width + cx)];
          if (value < 0) {
            continue;
          }
          const int gx = cx - radius_cells;
          const int gy = cy - radius_cells;
          const double wx =
              grid_origin_x + (static_cast<double>(gx) + 0.5) * resolution_;
          const double wy =
              grid_origin_y + (static_cast<double>(gy) + 0.5) * resolution_;
          markDisk(grid, grid_origin_x, grid_origin_y, wx, wy, value);
        }
      }
    } catch (const std::runtime_error &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "lidar_cost_grid failed to iterate %s: %s",
                           input.topic.c_str(), ex.what());
    }
  }

  // Implements `markMarkerInput` behavior.
  void markMarkerInput(const MarkerInput &input,
                       avg_msgs::msg::OccupancyGrid &grid,
                       const avg_msgs::msg::PointStamped &base_in_output,
                       const double grid_origin_x, const double grid_origin_y) {
    if (!input.markers) {
      return;
    }

    for (const auto &marker : input.markers->markers) {
      if (marker.action != avg_msgs::msg::Marker::ADD) {
        continue;
      }
      if (marker.type != avg_msgs::msg::Marker::CUBE &&
          marker.type != avg_msgs::msg::Marker::SPHERE &&
          marker.type != avg_msgs::msg::Marker::CYLINDER) {
        continue;
      }

      avg_msgs::msg::PointStamped point_out;
      if (!transformMarkerPoint(marker, point_out)) {
        continue;
      }

      const double distance = std::sqrt(
          squaredDistance2d(point_out.point.x, point_out.point.y,
                            base_in_output.point.x, base_in_output.point.y));
      const double marker_min_radius =
          std::max(obstacle_radius_m_, perception_marker_min_radius_m_);
      const double marker_max_radius =
          std::max(marker_min_radius, perception_marker_max_radius_m_);
      const double marker_radius = std::clamp(
          // HH_260707 - Keep perception detections large enough to influence
          // planning, but capped so object markers do not become lane-wide
          // stops.
          perception_marker_radius_scale_ *
              std::hypot(marker.scale.x, marker.scale.y),
          marker_min_radius, marker_max_radius);
      const int value = std::clamp(
          std::max(perception_marker_cost_, mapDistanceToCost(distance)),
          min_cost_, max_cost_);
      markDiskWithRadius(grid, grid_origin_x, grid_origin_y, point_out.point.x,
                         point_out.point.y, marker_radius, value);
    }
  }

  // Publishes `Grid` output.
  void publishGrid() {
    avg_msgs::msg::PointStamped base_in_output;
    if (!getBasePoseInOutput(base_in_output)) {
      return;
    }

    const double grid_origin_x = base_in_output.point.x + origin_x_;
    const double grid_origin_y = base_in_output.point.y + origin_y_;
    const auto now_time = now();
    const auto fresh_mask = freshInputMask(now_time);

    if (canReuseCachedGrid(base_in_output, fresh_mask)) {
      cached_grid_.header.stamp = now_time;
      cached_grid_.info.map_load_time = now_time;
      pub_grid_->publish(cached_grid_);
      publishAvgLidar(cached_grid_);
      return;
    }

    avg_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now_time;
    grid.header.frame_id = output_frame_id_;
    grid.info.map_load_time = grid.header.stamp;
    grid.info.resolution = static_cast<float>(resolution_);
    grid.info.width = static_cast<uint32_t>(width_);
    grid.info.height = static_cast<uint32_t>(height_);
    grid.info.origin.position.x = grid_origin_x;
    grid.info.origin.position.y = grid_origin_y;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    const int initial_value = (unknown_value_ >= -1 && unknown_value_ <= 100)
                                  ? unknown_value_
                                  : free_value_;
    grid.data.assign(static_cast<std::size_t>(width_ * height_),
                     static_cast<int8_t>(initial_value));

    bool has_fresh_input = false;
    for (const auto &input : cloud_inputs_) {
      if (input.cloud &&
          (now_time - input.rx_time).seconds() <= max_message_age_s_) {
        has_fresh_input = true;
        markCloudInput(input, grid, base_in_output, grid_origin_x,
                       grid_origin_y);
      }
    }
    for (const auto &input : marker_inputs_) {
      if (input.markers &&
          (now_time - input.rx_time).seconds() <= max_message_age_s_) {
        has_fresh_input = true;
        markMarkerInput(input, grid, base_in_output, grid_origin_x,
                        grid_origin_y);
      }
    }

    if (!has_fresh_input) {
      cacheBuiltGrid(grid, base_in_output, fresh_mask);
      pub_grid_->publish(grid);
      return;
    }

    if (ego_clear_radius_m_ > 0.0) {
      clearDisk(grid, grid_origin_x, grid_origin_y, base_in_output.point.x,
                base_in_output.point.y);
    }

    cacheBuiltGrid(grid, base_in_output, fresh_mask);
    pub_grid_->publish(cached_grid_);
    publishAvgLidar(cached_grid_);
  }

  std::uint64_t freshInputMask(const rclcpp::Time &now_time) const {
    std::uint64_t mask = 0;
    std::size_t bit = 0;
    for (const auto &input : cloud_inputs_) {
      if (bit >= 64) {
        break;
      }
      if (input.cloud &&
          (now_time - input.rx_time).seconds() <= max_message_age_s_) {
        mask |= (std::uint64_t{1} << bit);
      }
      ++bit;
    }
    for (const auto &input : marker_inputs_) {
      if (bit >= 64) {
        break;
      }
      if (input.markers &&
          (now_time - input.rx_time).seconds() <= max_message_age_s_) {
        mask |= (std::uint64_t{1} << bit);
      }
      ++bit;
    }
    return mask;
  }

  bool canReuseCachedGrid(const avg_msgs::msg::PointStamped &base_in_output,
                          std::uint64_t fresh_mask) const {
    if (!cached_grid_valid_) {
      return false;
    }
    if (input_sequence_ != last_built_input_sequence_ ||
        fresh_mask != last_built_fresh_mask_) {
      return false;
    }
    const double dx = base_in_output.point.x - last_base_x_;
    const double dy = base_in_output.point.y - last_base_y_;
    return (dx * dx + dy * dy) <
           (rebuild_min_pose_delta_m_ * rebuild_min_pose_delta_m_);
  }

  void cacheBuiltGrid(const avg_msgs::msg::OccupancyGrid &grid,
                      const avg_msgs::msg::PointStamped &base_in_output,
                      std::uint64_t fresh_mask) {
    cached_grid_ = grid;
    cached_grid_valid_ = true;
    last_built_input_sequence_ = input_sequence_;
    last_built_fresh_mask_ = fresh_mask;
    last_base_x_ = base_in_output.point.x;
    last_base_y_ = base_in_output.point.y;
  }

  // Publishes `AvgLidar` output.
  void publishAvgLidar(const avg_msgs::msg::OccupancyGrid &grid) {
    if (!publish_lidar_status_ || !avg_lidar_pub_) {
      return;
    }
    AvgSensingLidar avg_msg;
    avg_msg.near_cost_grid = avg_msgs::conversions::fromRos(grid);
    if (latest_primary_cloud_) {
      avg_msg.points_filtered =
          avg_msgs::conversions::fromRos(*latest_primary_cloud_);
    }
    avg_lidar_pub_->publish(avg_msg);
  }

  std::string input_topic_;
  std::vector<std::string> input_topics_;
  std::vector<std::string> extra_input_topics_;
  std::vector<std::string> perception_marker_topics_;
  std::string output_topic_;
  std::string lidar_status_topic_;
  std::string base_frame_id_;
  std::string output_frame_id_;
  double resolution_{0.10};
  int width_{180};
  int height_{180};
  double origin_x_{-9.0};
  double origin_y_{-9.0};
  int free_value_{0};
  int unknown_value_{-1};
  int min_cost_{65};
  int max_cost_{100};
  double cost_range_min_m_{0.4};
  double cost_range_max_m_{9.0};
  double obstacle_radius_m_{0.20};
  double cloud_min_z_m_{-std::numeric_limits<double>::infinity()};
  double cloud_max_z_m_{std::numeric_limits<double>::infinity()};
  int perception_marker_cost_{90};
  double perception_marker_min_radius_m_{0.35};
  double perception_marker_max_radius_m_{0.75};
  double perception_marker_radius_scale_{0.35};
  double ego_clear_radius_m_{0.90};
  double max_message_age_s_{0.50};
  double publish_rate_hz_{10.0};
  double rebuild_min_pose_delta_m_{0.05};
  bool publish_lidar_status_{false};
  std::uint64_t input_sequence_{0};
  std::uint64_t last_built_input_sequence_{~std::uint64_t{0}};
  std::uint64_t last_built_fresh_mask_{0};
  bool cached_grid_valid_{false};
  avg_msgs::msg::OccupancyGrid cached_grid_;
  double last_base_x_{0.0};
  double last_base_y_{0.0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<avg_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_;
  rclcpp::Publisher<AvgSensingLidar>::SharedPtr avg_lidar_pub_;
  std::vector<rclcpp::Subscription<avg_msgs::msg::PointCloud2>::SharedPtr>
      sub_clouds_;
  std::vector<rclcpp::Subscription<avg_msgs::msg::MarkerArray>::SharedPtr>
      sub_markers_;
  std::vector<CloudInput> cloud_inputs_;
  std::vector<MarkerInput> marker_inputs_;
  avg_msgs::msg::PointCloud2::ConstSharedPtr latest_primary_cloud_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace camrod::sensing

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<camrod::sensing::LidarCostGridNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
