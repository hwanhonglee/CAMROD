/*********************************************************************************************************************
Copyright (c) 2023 Vanjee
All rights reserved
...
*********************************************************************************************************************/

#pragma once

#include "source/source.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <unordered_set>
#include <vector>

#ifdef ROS_FOUND
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace vanjee {
namespace lidar {

// ------------------------------
// ROS1 helpers
// ------------------------------
namespace detail_ros1 {

inline bool getXYZOffsets(const sensor_msgs::PointCloud2 &cloud, int &off_x, int &off_y, int &off_z) {
  off_x = off_y = off_z = -1;
  for (const auto &f : cloud.fields) {
    if (f.name == "x" && f.datatype == sensor_msgs::PointField::FLOAT32) off_x = (int)f.offset;
    else if (f.name == "y" && f.datatype == sensor_msgs::PointField::FLOAT32) off_y = (int)f.offset;
    else if (f.name == "z" && f.datatype == sensor_msgs::PointField::FLOAT32) off_z = (int)f.offset;
  }
  return (off_x >= 0 && off_y >= 0 && off_z >= 0);
}

inline void filterPointCloud2_ROI_RangeZ(sensor_msgs::PointCloud2 &cloud,
                                        bool enable_roi,
                                        float x_min, float x_max,
                                        float y_min, float y_max,
                                        float z_roi_min, float z_roi_max,
                                        bool enable_rangez,
                                        float min_r, float max_r,
                                        float z_min, float z_max) {
  if (cloud.data.empty() || cloud.point_step == 0) return;

  int off_x, off_y, off_z;
  if (!getXYZOffsets(cloud, off_x, off_y, off_z)) return;

  const size_t ps = (size_t)cloud.point_step;
  const size_t n  = (size_t)cloud.width * (size_t)cloud.height;

  std::vector<uint8_t> out;
  out.reserve(cloud.data.size());

  const uint8_t *src = cloud.data.data();
  size_t keep = 0;

  for (size_t i = 0; i < n; ++i) {
    const uint8_t *pt = src + i * ps;
    const float x = *reinterpret_cast<const float*>(pt + off_x);
    const float y = *reinterpret_cast<const float*>(pt + off_y);
    const float z = *reinterpret_cast<const float*>(pt + off_z);

    if (enable_roi) {
      if (x < x_min || x > x_max) continue;
      if (y < y_min || y > y_max) continue;
      if (z < z_roi_min || z > z_roi_max) continue;
    }
    if (enable_rangez) {
      if (z < z_min || z > z_max) continue;
      const float r = std::sqrt(x*x + y*y + z*z);
      if (r < min_r || r > max_r) continue;
    }

    out.insert(out.end(), pt, pt + ps);
    ++keep;
  }

  cloud.height = 1;
  cloud.width = (uint32_t)keep;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.swap(out);
  cloud.is_dense = false;
}

inline void voxelDownsample_PointCloud2(sensor_msgs::PointCloud2 &cloud, float voxel) {
  if (voxel <= 0.0f) return;
  if (cloud.data.empty() || cloud.point_step == 0) return;

  int off_x, off_y, off_z;
  if (!getXYZOffsets(cloud, off_x, off_y, off_z)) return;

  auto hash3 = [](int xi, int yi, int zi) -> uint64_t {
    // 21bit씩 packing
    return ( (uint64_t)(xi & 0x1FFFFF) << 42 ) |
           ( (uint64_t)(yi & 0x1FFFFF) << 21 ) |
           ( (uint64_t)(zi & 0x1FFFFF) );
  };

  const size_t ps = (size_t)cloud.point_step;
  const size_t n  = (size_t)cloud.width * (size_t)cloud.height;

  std::unordered_set<uint64_t> seen;
  seen.reserve(n);

  std::vector<uint8_t> out;
  out.reserve(cloud.data.size());

  const uint8_t *src = cloud.data.data();
  size_t keep = 0;

  for (size_t i = 0; i < n; ++i) {
    const uint8_t *pt = src + i * ps;
    const float x = *reinterpret_cast<const float*>(pt + off_x);
    const float y = *reinterpret_cast<const float*>(pt + off_y);
    const float z = *reinterpret_cast<const float*>(pt + off_z);

    const int xi = (int)std::floor(x / voxel);
    const int yi = (int)std::floor(y / voxel);
    const int zi = (int)std::floor(z / voxel);

    const uint64_t key = hash3(xi, yi, zi);
    if (seen.find(key) != seen.end()) continue;
    seen.insert(key);

    out.insert(out.end(), pt, pt + ps);
    ++keep;
  }

  cloud.height = 1;
  cloud.width = (uint32_t)keep;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.swap(out);
  cloud.is_dense = false;
}

} // namespace detail_ros1

// ------------------------------
// ROS1 conversion (ring filter 가능하도록 파라미터 추가)
// ------------------------------
inline sensor_msgs::PointCloud2 toRosMsgLight(const LidarPointCloudMsg &vanjee_msg,
                                             const std::string &frame_id,
                                             bool enable_ring_filter,
                                             uint16_t keep_ring_min,
                                             uint16_t keep_ring_max) {
  sensor_msgs::PointCloud2 ros_msg;
  uint16_t offset = 0;

  ros_msg.width  = vanjee_msg.height;
  ros_msg.height = vanjee_msg.width;

  sensor_msgs::PointField x_field, y_field, z_field;
  x_field.name = "x"; x_field.offset = offset; x_field.datatype = sensor_msgs::PointField::FLOAT32; x_field.count = 1;
  offset += x_field.count * sizeOfPointField(sensor_msgs::PointField::FLOAT32);
  ros_msg.fields.push_back(x_field);

  y_field.name = "y"; y_field.offset = offset; y_field.datatype = sensor_msgs::PointField::FLOAT32; y_field.count = 1;
  offset += y_field.count * sizeOfPointField(sensor_msgs::PointField::FLOAT32);
  ros_msg.fields.push_back(y_field);

  z_field.name = "z"; z_field.offset = offset; z_field.datatype = sensor_msgs::PointField::FLOAT32; z_field.count = 1;
  offset += z_field.count * sizeOfPointField(sensor_msgs::PointField::FLOAT32);
  ros_msg.fields.push_back(z_field);

#ifdef POINT_TYPE_XYZI
  sensor_msgs::PointField intensity_field;
  intensity_field.name = "intensity"; intensity_field.offset = offset; intensity_field.datatype = sensor_msgs::PointField::FLOAT32; intensity_field.count = 1;
  offset += intensity_field.count * sizeOfPointField(sensor_msgs::PointField::FLOAT32);
  ros_msg.fields.push_back(intensity_field);
#endif

#ifdef POINT_TYPE_XYZIRT
  sensor_msgs::PointField intensity_field, ring_field, timestamp_field;

  intensity_field.name = "intensity"; intensity_field.offset = offset; intensity_field.datatype = sensor_msgs::PointField::FLOAT32; intensity_field.count = 1;
  offset += intensity_field.count * sizeOfPointField(sensor_msgs::PointField::FLOAT32);
  ros_msg.fields.push_back(intensity_field);

  ring_field.name = "ring"; ring_field.offset = offset; ring_field.datatype = sensor_msgs::PointField::UINT16; ring_field.count = 1;
  offset += ring_field.count * sizeOfPointField(sensor_msgs::PointField::UINT16);
  ros_msg.fields.push_back(ring_field);

  timestamp_field.name = "timestamp"; timestamp_field.offset = offset; timestamp_field.datatype = sensor_msgs::PointField::FLOAT64; timestamp_field.count = 1;
  offset += timestamp_field.count * sizeOfPointField(sensor_msgs::PointField::FLOAT64);
  ros_msg.fields.push_back(timestamp_field);
#endif

  ros_msg.is_dense = vanjee_msg.is_dense;
  ros_msg.is_bigendian = false;
  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.point_step * ros_msg.width;

  ros_msg.header.seq = vanjee_msg.seq;
  ros_msg.header.frame_id = frame_id;
  ros_msg.header.stamp = ros_msg.header.stamp.fromSec(vanjee_msg.timestamp);

  uint32_t size = ros_msg.row_step * ros_msg.height;
  ros_msg.data.resize(size);

#ifdef POINT_TYPE_XYZIRT
  if (enable_ring_filter) {
    size_t keep_cnt = 0;
    for (size_t i = 0; i < vanjee_msg.height; i++) {
      for (size_t j = 0; j < vanjee_msg.width; j++) {
        const auto &p = vanjee_msg.points[i + j * vanjee_msg.height];
        if (p.ring >= keep_ring_min && p.ring <= keep_ring_max) ++keep_cnt;
      }
    }
    ros_msg.height = 1;
    ros_msg.width = (uint32_t)keep_cnt;
    ros_msg.row_step = ros_msg.point_step * ros_msg.width;

    const uint32_t new_size = ros_msg.row_step * ros_msg.height;
    ros_msg.data.resize(new_size);

    uint8_t *dst = ros_msg.data.data();
    for (size_t i = 0; i < vanjee_msg.height; i++) {
      for (size_t j = 0; j < vanjee_msg.width; j++) {
        const auto &p = vanjee_msg.points[i + j * vanjee_msg.height];
        if (p.ring < keep_ring_min || p.ring > keep_ring_max) continue;
        std::memcpy(dst, &p, ros_msg.point_step);
        dst += ros_msg.point_step;
      }
    }
    return ros_msg;
  }
#endif

  std::memcpy(ros_msg.data.data(), vanjee_msg.points.data(), size);
  return ros_msg;
}

inline sensor_msgs::PointCloud2 toRosMsg(const LidarPointCloudMsg &vanjee_msg,
                                        const std::string &frame_id,
                                        bool send_by_rows,
                                        bool enable_ring_filter,
                                        uint16_t keep_ring_min,
                                        uint16_t keep_ring_max) {
  if (!send_by_rows) {
    return toRosMsgLight(vanjee_msg, frame_id, enable_ring_filter, keep_ring_min, keep_ring_max);
  }

  sensor_msgs::PointCloud2 ros_msg;

  int fields = 4;
#ifdef POINT_TYPE_XYZIRT
  fields = 6;
#endif
  ros_msg.fields.clear();
  ros_msg.fields.reserve(fields);

#ifdef POINT_TYPE_XYZIRT
  if (enable_ring_filter) {
    size_t keep_cnt = 0;
    for (size_t i = 0; i < vanjee_msg.height; i++) {
      for (size_t j = 0; j < vanjee_msg.width; j++) {
        const auto &p = vanjee_msg.points[i + j * vanjee_msg.height];
        if (p.ring >= keep_ring_min && p.ring <= keep_ring_max) ++keep_cnt;
      }
    }
    ros_msg.height = 1;
    ros_msg.width = (uint32_t)keep_cnt;
  } else {
    ros_msg.width = vanjee_msg.width;
    ros_msg.height = vanjee_msg.height;
  }
#else
  ros_msg.width = vanjee_msg.width;
  ros_msg.height = vanjee_msg.height;
#endif

  int offset = 0;
  offset = addPointField(ros_msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
#ifdef POINT_TYPE_XYZI
  offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset);
#endif
#ifdef POINT_TYPE_XYZIRT
  offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "ring", 1, sensor_msgs::PointField::UINT16, offset);
  offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::PointField::FLOAT64, offset);
#endif

  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.width * ros_msg.point_step;
  ros_msg.is_dense = vanjee_msg.is_dense;
  ros_msg.data.resize((size_t)ros_msg.point_step * (size_t)ros_msg.width * (size_t)ros_msg.height);

  sensor_msgs::PointCloud2Iterator<float> iter_x(ros_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(ros_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(ros_msg, "z");
#ifdef POINT_TYPE_XYZI
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(ros_msg, "intensity");
#endif
#ifdef POINT_TYPE_XYZIRT
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(ros_msg, "intensity");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring(ros_msg, "ring");
  sensor_msgs::PointCloud2Iterator<double> iter_timestamp(ros_msg, "timestamp");
#endif

  for (size_t i = 0; i < vanjee_msg.height; i++) {
    for (size_t j = 0; j < vanjee_msg.width; j++) {
      const auto &p = vanjee_msg.points[i + j * vanjee_msg.height];

#ifdef POINT_TYPE_XYZIRT
      if (enable_ring_filter) {
        const uint16_t r = p.ring;
        if (r < keep_ring_min || r > keep_ring_max) continue;
      }
#endif

      *iter_x = p.x; *iter_y = p.y; *iter_z = p.z;
      ++iter_x; ++iter_y; ++iter_z;

#ifdef POINT_TYPE_XYZI
      *iter_intensity = p.intensity;
      ++iter_intensity;
#endif
#ifdef POINT_TYPE_XYZIRT
      *iter_intensity = p.intensity;
      *iter_ring = p.ring;
      *iter_timestamp = p.timestamp;
      ++iter_intensity; ++iter_ring; ++iter_timestamp;
#endif
    }
  }

  ros_msg.header.seq = vanjee_msg.seq;
  ros_msg.header.stamp = ros_msg.header.stamp.fromSec(vanjee_msg.timestamp);
  ros_msg.header.frame_id = frame_id;

  return ros_msg;
}

// ------------------------------
// ROS1 Destination (옵션 필터들 적용)
// ------------------------------
class DestinationPointCloudRos : public DestinationPointCloud {
 private:
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Publisher pub_;
  std::string frame_id_;
  bool send_by_rows_{false};

  bool enable_ring_filter_{false};
  uint16_t keep_ring_min_{1};
  uint16_t keep_ring_max_{65535};

  bool enable_roi_filter_{false};
  float roi_x_min_{-50.0f}, roi_x_max_{50.0f};
  float roi_y_min_{-50.0f}, roi_y_max_{50.0f};
  float roi_z_min_{-5.0f},  roi_z_max_{5.0f};

  bool enable_range_z_filter_{false};
  float min_range_m_{0.0f};
  float max_range_m_{200.0f};
  float z_min_m_{-999.0f};
  float z_max_m_{ 999.0f};

  bool enable_voxel_noise_{false};
  float voxel_size_m_{0.20f};

 public:
  void init(const YAML::Node &config) override;
  void sendPointCloud(const LidarPointCloudMsg &msg) override;
  ~DestinationPointCloudRos() override = default;
};

inline void DestinationPointCloudRos::init(const YAML::Node &config) {
  yamlRead<bool>(config["ros"], "ros_send_by_rows", send_by_rows_, false);
  yamlRead<std::string>(config["ros"], "ros_frame_id", frame_id_, "vanjee_lidar");
  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"], "ros_send_point_cloud_topic", ros_send_topic, "vanjee_lidar_points");

  yamlRead<bool>(config["driver"], "enable_ring_filter", enable_ring_filter_, false);
  int kmin = 1, kmax = 65535;
  yamlRead<int>(config["driver"], "keep_ring_min", kmin, 1);
  yamlRead<int>(config["driver"], "keep_ring_max", kmax, 65535);
  keep_ring_min_ = (uint16_t)kmin;
  keep_ring_max_ = (uint16_t)kmax;

  yamlRead<bool>(config["driver"], "enable_roi_filter", enable_roi_filter_, false);
  yamlRead<float>(config["driver"], "roi_x_min", roi_x_min_, -50.0f);
  yamlRead<float>(config["driver"], "roi_x_max", roi_x_max_,  50.0f);
  yamlRead<float>(config["driver"], "roi_y_min", roi_y_min_, -50.0f);
  yamlRead<float>(config["driver"], "roi_y_max", roi_y_max_,  50.0f);
  yamlRead<float>(config["driver"], "roi_z_min", roi_z_min_,  -5.0f);
  yamlRead<float>(config["driver"], "roi_z_max", roi_z_max_,   5.0f);

  yamlRead<bool>(config["driver"], "enable_range_z_filter", enable_range_z_filter_, false);
  yamlRead<float>(config["driver"], "min_range_m", min_range_m_, 0.0f);
  yamlRead<float>(config["driver"], "max_range_m", max_range_m_, 200.0f);
  yamlRead<float>(config["driver"], "z_min_m", z_min_m_, -999.0f);
  yamlRead<float>(config["driver"], "z_max_m", z_max_m_,  999.0f);

  yamlRead<bool>(config["driver"], "enable_voxel_noise", enable_voxel_noise_, false);
  yamlRead<float>(config["driver"], "voxel_size_m", voxel_size_m_, 0.2f);

  nh_.reset(new ros::NodeHandle());
  pub_ = nh_->advertise<sensor_msgs::PointCloud2>(ros_send_topic, 10);
}

inline void DestinationPointCloudRos::sendPointCloud(const LidarPointCloudMsg &msg) {
  sensor_msgs::PointCloud2 cloud =
      toRosMsg(msg, frame_id_, send_by_rows_, enable_ring_filter_, keep_ring_min_, keep_ring_max_);

  if (enable_roi_filter_ || enable_range_z_filter_) {
    detail_ros1::filterPointCloud2_ROI_RangeZ(cloud,
                                             enable_roi_filter_,
                                             roi_x_min_, roi_x_max_,
                                             roi_y_min_, roi_y_max_,
                                             roi_z_min_, roi_z_max_,
                                             enable_range_z_filter_,
                                             min_range_m_, max_range_m_,
                                             z_min_m_, z_max_m_);
  }
  if (enable_voxel_noise_) {
    detail_ros1::voxelDownsample_PointCloud2(cloud, voxel_size_m_);
  }

  pub_.publish(cloud);
}

}  // namespace lidar
}  // namespace vanjee

#endif  // ROS_FOUND

// =================================================================================================
// ROS2
// =================================================================================================
#ifdef ROS2_FOUND

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "source/source_ros_msg_delegate.hpp"

namespace vanjee {
namespace lidar {

// ------------------------------
// ROS2 helpers
// ------------------------------
namespace detail_ros2 {

inline bool getXYZOffsets(const sensor_msgs::msg::PointCloud2 &cloud, int &off_x, int &off_y, int &off_z) {
  off_x = off_y = off_z = -1;
  for (const auto &f : cloud.fields) {
    if (f.name == "x" && f.datatype == sensor_msgs::msg::PointField::FLOAT32) off_x = (int)f.offset;
    else if (f.name == "y" && f.datatype == sensor_msgs::msg::PointField::FLOAT32) off_y = (int)f.offset;
    else if (f.name == "z" && f.datatype == sensor_msgs::msg::PointField::FLOAT32) off_z = (int)f.offset;
  }
  return (off_x >= 0 && off_y >= 0 && off_z >= 0);
}

inline void filterPointCloud2_ROI_RangeZ(sensor_msgs::msg::PointCloud2 &cloud,
                                        bool enable_roi,
                                        float x_min, float x_max,
                                        float y_min, float y_max,
                                        float z_roi_min, float z_roi_max,
                                        bool enable_rangez,
                                        float min_r, float max_r,
                                        float z_min, float z_max) {
  if (cloud.data.empty() || cloud.point_step == 0) return;

  int off_x, off_y, off_z;
  if (!getXYZOffsets(cloud, off_x, off_y, off_z)) return;

  const size_t ps = (size_t)cloud.point_step;
  const size_t n  = (size_t)cloud.width * (size_t)cloud.height;

  std::vector<uint8_t> out;
  out.reserve(cloud.data.size());

  const uint8_t *src = cloud.data.data();
  size_t keep = 0;

  for (size_t i = 0; i < n; ++i) {
    const uint8_t *pt = src + i * ps;
    const float x = *reinterpret_cast<const float*>(pt + off_x);
    const float y = *reinterpret_cast<const float*>(pt + off_y);
    const float z = *reinterpret_cast<const float*>(pt + off_z);

    if (enable_roi) {
      if (x < x_min || x > x_max) continue;
      if (y < y_min || y > y_max) continue;
      if (z < z_roi_min || z > z_roi_max) continue;
    }
    if (enable_rangez) {
      if (z < z_min || z > z_max) continue;
      const float r = std::sqrt(x*x + y*y + z*z);
      if (r < min_r || r > max_r) continue;
    }

    out.insert(out.end(), pt, pt + ps);
    ++keep;
  }

  cloud.height = 1;
  cloud.width = (uint32_t)keep;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.swap(out);
  cloud.is_dense = false;
}

inline void voxelDownsample_PointCloud2(sensor_msgs::msg::PointCloud2 &cloud, float voxel) {
  if (voxel <= 0.0f) return;
  if (cloud.data.empty() || cloud.point_step == 0) return;

  int off_x, off_y, off_z;
  if (!getXYZOffsets(cloud, off_x, off_y, off_z)) return;

  auto hash3 = [](int xi, int yi, int zi) -> uint64_t {
    return ( (uint64_t)(xi & 0x1FFFFF) << 42 ) |
           ( (uint64_t)(yi & 0x1FFFFF) << 21 ) |
           ( (uint64_t)(zi & 0x1FFFFF) );
  };

  const size_t ps = (size_t)cloud.point_step;
  const size_t n  = (size_t)cloud.width * (size_t)cloud.height;

  std::unordered_set<uint64_t> seen;
  seen.reserve(n);

  std::vector<uint8_t> out;
  out.reserve(cloud.data.size());

  const uint8_t *src = cloud.data.data();
  size_t keep = 0;

  for (size_t i = 0; i < n; ++i) {
    const uint8_t *pt = src + i * ps;
    const float x = *reinterpret_cast<const float*>(pt + off_x);
    const float y = *reinterpret_cast<const float*>(pt + off_y);
    const float z = *reinterpret_cast<const float*>(pt + off_z);

    const int xi = (int)std::floor(x / voxel);
    const int yi = (int)std::floor(y / voxel);
    const int zi = (int)std::floor(z / voxel);

    const uint64_t key = hash3(xi, yi, zi);
    if (seen.find(key) != seen.end()) continue;
    seen.insert(key);

    out.insert(out.end(), pt, pt + ps);
    ++keep;
  }

  cloud.height = 1;
  cloud.width = (uint32_t)keep;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.swap(out);
  cloud.is_dense = false;
}

} // namespace detail_ros2

// ------------------------------
// ROS2 conversion
// ------------------------------
inline sensor_msgs::msg::PointCloud2 toRosMsgLight(const LidarPointCloudMsg &vanjee_msg,
                                                  const std::string &frame_id,
                                                  bool enable_ring_filter,
                                                  uint16_t keep_ring_min,
                                                  uint16_t keep_ring_max) {
  sensor_msgs::msg::PointCloud2 ros_msg;
  uint16_t offset = 0;

  ros_msg.width  = vanjee_msg.height;
  ros_msg.height = vanjee_msg.width;

  sensor_msgs::msg::PointField x_field, y_field, z_field;
  x_field.name = "x"; x_field.offset = offset; x_field.datatype = sensor_msgs::msg::PointField::FLOAT32; x_field.count = 1;
  offset += x_field.count * sizeOfPointField(sensor_msgs::msg::PointField::FLOAT32);
  ros_msg.fields.push_back(x_field);

  y_field.name = "y"; y_field.offset = offset; y_field.datatype = sensor_msgs::msg::PointField::FLOAT32; y_field.count = 1;
  offset += y_field.count * sizeOfPointField(sensor_msgs::msg::PointField::FLOAT32);
  ros_msg.fields.push_back(y_field);

  z_field.name = "z"; z_field.offset = offset; z_field.datatype = sensor_msgs::msg::PointField::FLOAT32; z_field.count = 1;
  offset += z_field.count * sizeOfPointField(sensor_msgs::msg::PointField::FLOAT32);
  ros_msg.fields.push_back(z_field);

#ifdef POINT_TYPE_XYZI
  sensor_msgs::msg::PointField intensity_field;
  intensity_field.name = "intensity"; intensity_field.offset = offset; intensity_field.datatype = sensor_msgs::msg::PointField::FLOAT32; intensity_field.count = 1;
  offset += intensity_field.count * sizeOfPointField(sensor_msgs::msg::PointField::FLOAT32);
  ros_msg.fields.push_back(intensity_field);
#endif

#ifdef POINT_TYPE_XYZIRT
  sensor_msgs::msg::PointField intensity_field, ring_field, timestamp_field;

  intensity_field.name = "intensity"; intensity_field.offset = offset; intensity_field.datatype = sensor_msgs::msg::PointField::FLOAT32; intensity_field.count = 1;
  offset += intensity_field.count * sizeOfPointField(sensor_msgs::msg::PointField::FLOAT32);
  ros_msg.fields.push_back(intensity_field);

  ring_field.name = "ring"; ring_field.offset = offset; ring_field.datatype = sensor_msgs::msg::PointField::UINT16; ring_field.count = 1;
  offset += ring_field.count * sizeOfPointField(sensor_msgs::msg::PointField::UINT16);
  ros_msg.fields.push_back(ring_field);

  timestamp_field.name = "timestamp"; timestamp_field.offset = offset; timestamp_field.datatype = sensor_msgs::msg::PointField::FLOAT64; timestamp_field.count = 1;
  offset += timestamp_field.count * sizeOfPointField(sensor_msgs::msg::PointField::FLOAT64);
  ros_msg.fields.push_back(timestamp_field);
#endif

  ros_msg.is_dense = vanjee_msg.is_dense;
  ros_msg.is_bigendian = false;
  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.point_step * ros_msg.width;

  ros_msg.header.frame_id = frame_id;
  ros_msg.header.stamp.sec = (uint32_t)std::floor(vanjee_msg.timestamp);
  ros_msg.header.stamp.nanosec = (uint32_t)std::round((vanjee_msg.timestamp - ros_msg.header.stamp.sec) * 1e9);

  uint32_t size = ros_msg.row_step * ros_msg.height;
  ros_msg.data.resize(size);

#ifdef POINT_TYPE_XYZIRT
  if (enable_ring_filter) {
    size_t keep_cnt = 0;
    for (size_t i = 0; i < vanjee_msg.height; i++) {
      for (size_t j = 0; j < vanjee_msg.width; j++) {
        const auto &p = vanjee_msg.points[i + j * vanjee_msg.height];
        if (p.ring >= keep_ring_min && p.ring <= keep_ring_max) ++keep_cnt;
      }
    }

    ros_msg.height = 1;
    ros_msg.width = (uint32_t)keep_cnt;
    ros_msg.row_step = ros_msg.point_step * ros_msg.width;

    const uint32_t new_size = ros_msg.row_step * ros_msg.height;
    ros_msg.data.resize(new_size);

    uint8_t *dst = ros_msg.data.data();
    for (size_t i = 0; i < vanjee_msg.height; i++) {
      for (size_t j = 0; j < vanjee_msg.width; j++) {
        const auto &p = vanjee_msg.points[i + j * vanjee_msg.height];
        if (p.ring < keep_ring_min || p.ring > keep_ring_max) continue;
        std::memcpy(dst, &p, ros_msg.point_step);
        dst += ros_msg.point_step;
      }
    }
    return ros_msg;
  }
#endif

  std::memcpy(ros_msg.data.data(), vanjee_msg.points.data(), size);
  return ros_msg;
}

inline sensor_msgs::msg::PointCloud2 toRosMsg(const LidarPointCloudMsg &vanjee_msg,
                                             const std::string &frame_id,
                                             bool send_by_rows,
                                             bool enable_ring_filter,
                                             uint16_t keep_ring_min,
                                             uint16_t keep_ring_max) {
  if (!send_by_rows) {
    return toRosMsgLight(vanjee_msg, frame_id, enable_ring_filter, keep_ring_min, keep_ring_max);
  }

  sensor_msgs::msg::PointCloud2 ros_msg;

  int fields = 4;
#ifdef POINT_TYPE_XYZIRT
  fields = 6;
#endif
  ros_msg.fields.clear();
  ros_msg.fields.reserve(fields);

#ifdef POINT_TYPE_XYZIRT
  if (enable_ring_filter) {
    size_t keep_cnt = 0;
    for (size_t i = 0; i < vanjee_msg.height; i++) {
      for (size_t j = 0; j < vanjee_msg.width; j++) {
        const auto &p = vanjee_msg.points[i + j * vanjee_msg.height];
        if (p.ring >= keep_ring_min && p.ring <= keep_ring_max) ++keep_cnt;
      }
    }
    ros_msg.height = 1;
    ros_msg.width = (uint32_t)keep_cnt;
  } else {
    ros_msg.width = vanjee_msg.width;
    ros_msg.height = vanjee_msg.height;
  }
#else
  ros_msg.width = vanjee_msg.width;
  ros_msg.height = vanjee_msg.height;
#endif

  int offset = 0;
  offset = addPointField(ros_msg, "x", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "y", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "z", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
#ifdef POINT_TYPE_XYZI
  offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
#endif
#ifdef POINT_TYPE_XYZIRT
  offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "ring", 1, sensor_msgs::msg::PointField::UINT16, offset);
  offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64, offset);
#endif

  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.width * ros_msg.point_step;
  ros_msg.is_dense = vanjee_msg.is_dense;
  ros_msg.data.resize((size_t)ros_msg.point_step * (size_t)ros_msg.width * (size_t)ros_msg.height);

  sensor_msgs::PointCloud2Iterator<float> iter_x(ros_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(ros_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(ros_msg, "z");
#ifdef POINT_TYPE_XYZI
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(ros_msg, "intensity");
#endif
#ifdef POINT_TYPE_XYZIRT
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(ros_msg, "intensity");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring(ros_msg, "ring");
  sensor_msgs::PointCloud2Iterator<double> iter_timestamp(ros_msg, "timestamp");
#endif

  for (size_t i = 0; i < vanjee_msg.height; i++) {
    for (size_t j = 0; j < vanjee_msg.width; j++) {
      const auto &p = vanjee_msg.points[i + j * vanjee_msg.height];

#ifdef POINT_TYPE_XYZIRT
      if (enable_ring_filter) {
        const uint16_t r = p.ring;
        if (r < keep_ring_min || r > keep_ring_max) continue;
      }
#endif

      *iter_x = p.x; *iter_y = p.y; *iter_z = p.z;
      ++iter_x; ++iter_y; ++iter_z;

#ifdef POINT_TYPE_XYZI
      *iter_intensity = p.intensity;
      ++iter_intensity;
#endif
#ifdef POINT_TYPE_XYZIRT
      *iter_intensity = p.intensity;
      *iter_ring = p.ring;
      *iter_timestamp = p.timestamp;
      ++iter_intensity; ++iter_ring; ++iter_timestamp;
#endif
    }
  }

  ros_msg.header.stamp.sec = (uint32_t)std::floor(vanjee_msg.timestamp);
  ros_msg.header.stamp.nanosec = (uint32_t)std::round((vanjee_msg.timestamp - ros_msg.header.stamp.sec) * 1e9);
  ros_msg.header.frame_id = frame_id;

  return ros_msg;
}

// ------------------------------
// ROS2 Destination
// ------------------------------
class DestinationPointCloudRos : virtual public DestinationPointCloud {
 private:
  using PointCloud2MsgPubPtr = std::shared_ptr<VanjeeLidarSdkPublishRosMsg<sensor_msgs::msg::PointCloud2>>;
  PointCloud2MsgPubPtr pointcloud2_msg_pub_ptr_;

  std::string frame_id_;
  bool send_by_rows_{false};

  bool enable_ring_filter_{false};
  uint16_t keep_ring_min_{1};
  uint16_t keep_ring_max_{65535};

  bool enable_roi_filter_{false};
  float roi_x_min_{-50.0f}, roi_x_max_{50.0f};
  float roi_y_min_{-50.0f}, roi_y_max_{50.0f};
  float roi_z_min_{-5.0f},  roi_z_max_{5.0f};

  bool enable_range_z_filter_{false};
  float min_range_m_{0.0f};
  float max_range_m_{200.0f};
  float z_min_m_{-999.0f};
  float z_max_m_{ 999.0f};

  bool enable_voxel_noise_{false};
  float voxel_size_m_{0.20f};

 public:
  void init(const YAML::Node &config) override;
  void sendPointCloud(const LidarPointCloudMsg &msg) override;
  ~DestinationPointCloudRos() override = default;
};

inline void DestinationPointCloudRos::init(const YAML::Node &config) {
  yamlRead<bool>(config["ros"], "ros_send_by_rows", send_by_rows_, false);
  yamlRead<std::string>(config["ros"], "ros_frame_id", frame_id_, "vanjee_lidar");

  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"], "ros_send_point_cloud_topic", ros_send_topic, "vanjee_lidar_points");

  yamlRead<bool>(config["driver"], "enable_ring_filter", enable_ring_filter_, false);
  int kmin = 1, kmax = 65535;
  yamlRead<int>(config["driver"], "keep_ring_min", kmin, 1);
  yamlRead<int>(config["driver"], "keep_ring_max", kmax, 65535);
  keep_ring_min_ = (uint16_t)kmin;
  keep_ring_max_ = (uint16_t)kmax;

  yamlRead<bool>(config["driver"], "enable_roi_filter", enable_roi_filter_, false);
  yamlRead<float>(config["driver"], "roi_x_min", roi_x_min_, -50.0f);
  yamlRead<float>(config["driver"], "roi_x_max", roi_x_max_,  50.0f);
  yamlRead<float>(config["driver"], "roi_y_min", roi_y_min_, -50.0f);
  yamlRead<float>(config["driver"], "roi_y_max", roi_y_max_,  50.0f);
  yamlRead<float>(config["driver"], "roi_z_min", roi_z_min_, -50.0f);
  yamlRead<float>(config["driver"], "roi_z_max", roi_z_max_,  50.0f);

  yamlRead<bool>(config["driver"], "enable_range_z_filter", enable_range_z_filter_, false);
  yamlRead<float>(config["driver"], "min_range_m", min_range_m_, 0.0f);
  yamlRead<float>(config["driver"], "max_range_m", max_range_m_, 200.0f);
  yamlRead<float>(config["driver"], "z_min_m", z_min_m_, -999.0f);
  yamlRead<float>(config["driver"], "z_max_m", z_max_m_,  999.0f);

  yamlRead<bool>(config["driver"], "enable_voxel_noise", enable_voxel_noise_, false);
  yamlRead<float>(config["driver"], "voxel_size_m", voxel_size_m_, 0.2f);

  pointcloud2_msg_pub_ptr_ =
      VanjeeLidarSdkNode::CreateInstance()->GetPointCloud2MsgPublisher(ros_send_topic);
}

inline void DestinationPointCloudRos::sendPointCloud(const LidarPointCloudMsg &msg) {
  sensor_msgs::msg::PointCloud2 cloud =
      toRosMsg(msg, frame_id_, send_by_rows_, enable_ring_filter_, keep_ring_min_, keep_ring_max_);

  if (enable_roi_filter_ || enable_range_z_filter_) {
    detail_ros2::filterPointCloud2_ROI_RangeZ(cloud,
                                             enable_roi_filter_,
                                             roi_x_min_, roi_x_max_,
                                             roi_y_min_, roi_y_max_,
                                             roi_z_min_, roi_z_max_,
                                             enable_range_z_filter_,
                                             min_range_m_, max_range_m_,
                                             z_min_m_, z_max_m_);
  }
  if (enable_voxel_noise_) {
    detail_ros2::voxelDownsample_PointCloud2(cloud, voxel_size_m_);
  }

  // ✅ header timestamp를 ROS Time(now)로 통일 (RViz/TF latency 방지)
  cloud.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  pointcloud2_msg_pub_ptr_->PublishMsg(cloud);
}

}  // namespace lidar
}  // namespace vanjee

#endif  // ROS2_FOUND
