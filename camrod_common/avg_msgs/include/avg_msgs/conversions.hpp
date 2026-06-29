#pragma once

#include <algorithm>
#include <cstddef>

#include <action_msgs/msg/goal_info.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <vision_msgs/msg/pose2_d.hpp>

#include <avg_msgs/msg/avg_bool.hpp>
#include <avg_msgs/msg/avg_bounding_box2_d.hpp>
#include <avg_msgs/msg/avg_camera_info.hpp>
#include <avg_msgs/msg/avg_color_rgba.hpp>
#include <avg_msgs/msg/avg_detection2_d.hpp>
#include <avg_msgs/msg/avg_detection2_d_array.hpp>
#include <avg_msgs/msg/avg_float32.hpp>
#include <avg_msgs/msg/avg_goal_info.hpp>
#include <avg_msgs/msg/avg_goal_status.hpp>
#include <avg_msgs/msg/avg_goal_status_array.hpp>
#include <avg_msgs/msg/avg_header.hpp>
#include <avg_msgs/msg/avg_image.hpp>
#include <avg_msgs/msg/avg_imu.hpp>
#include <avg_msgs/msg/avg_laser_scan.hpp>
#include <avg_msgs/msg/avg_map_meta_data.hpp>
#include <avg_msgs/msg/avg_marker.hpp>
#include <avg_msgs/msg/avg_marker_array.hpp>
#include <avg_msgs/msg/avg_nav_sat_fix.hpp>
#include <avg_msgs/msg/avg_nav_sat_status.hpp>
#include <avg_msgs/msg/avg_object_hypothesis.hpp>
#include <avg_msgs/msg/avg_object_hypothesis_with_pose.hpp>
#include <avg_msgs/msg/avg_occupancy_grid.hpp>
#include <avg_msgs/msg/avg_odometry.hpp>
#include <avg_msgs/msg/avg_path.hpp>
#include <avg_msgs/msg/avg_point.hpp>
#include <avg_msgs/msg/avg_point32.hpp>
#include <avg_msgs/msg/avg_point_cloud2.hpp>
#include <avg_msgs/msg/avg_point_field.hpp>
#include <avg_msgs/msg/avg_polygon.hpp>
#include <avg_msgs/msg/avg_polygon_stamped.hpp>
#include <avg_msgs/msg/avg_pose.hpp>
#include <avg_msgs/msg/avg_pose2_d.hpp>
#include <avg_msgs/msg/avg_pose_stamped.hpp>
#include <avg_msgs/msg/avg_pose_with_covariance.hpp>
#include <avg_msgs/msg/avg_pose_with_covariance_stamped.hpp>
#include <avg_msgs/msg/avg_quaternion.hpp>
#include <avg_msgs/msg/avg_range.hpp>
#include <avg_msgs/msg/avg_region_of_interest.hpp>
#include <avg_msgs/msg/avg_string.hpp>
#include <avg_msgs/msg/avg_transform.hpp>
#include <avg_msgs/msg/avg_transform_stamped.hpp>
#include <avg_msgs/msg/avg_twist.hpp>
#include <avg_msgs/msg/avg_twist_stamped.hpp>
#include <avg_msgs/msg/avg_twist_with_covariance.hpp>
#include <avg_msgs/msg/avg_twist_with_covariance_stamped.hpp>
#include <avg_msgs/msg/avg_vector3.hpp>

namespace avg_msgs::conversions
{

inline avg_msgs::msg::AvgHeader fromRos(const std_msgs::msg::Header & in)
{
  avg_msgs::msg::AvgHeader out;
  out.stamp = in.stamp;
  out.frame_id = in.frame_id;
  return out;
}

inline std_msgs::msg::Header toRos(const avg_msgs::msg::AvgHeader & in)
{
  std_msgs::msg::Header out;
  out.stamp = in.stamp;
  out.frame_id = in.frame_id;
  return out;
}

inline avg_msgs::msg::AvgBool fromRos(const std_msgs::msg::Bool & in)
{
  avg_msgs::msg::AvgBool out;
  out.data = in.data;
  return out;
}

inline std_msgs::msg::Bool toRos(const avg_msgs::msg::AvgBool & in)
{
  std_msgs::msg::Bool out;
  out.data = in.data;
  return out;
}

inline avg_msgs::msg::AvgString fromRos(const std_msgs::msg::String & in)
{
  avg_msgs::msg::AvgString out;
  out.data = in.data;
  return out;
}

inline std_msgs::msg::String toRos(const avg_msgs::msg::AvgString & in)
{
  std_msgs::msg::String out;
  out.data = in.data;
  return out;
}

inline avg_msgs::msg::AvgFloat32 fromRos(const std_msgs::msg::Float32 & in)
{
  avg_msgs::msg::AvgFloat32 out;
  out.data = in.data;
  return out;
}

inline std_msgs::msg::Float32 toRos(const avg_msgs::msg::AvgFloat32 & in)
{
  std_msgs::msg::Float32 out;
  out.data = in.data;
  return out;
}

inline avg_msgs::msg::AvgColorRGBA fromRos(const std_msgs::msg::ColorRGBA & in)
{
  avg_msgs::msg::AvgColorRGBA out;
  out.r = in.r;
  out.g = in.g;
  out.b = in.b;
  out.a = in.a;
  return out;
}

inline std_msgs::msg::ColorRGBA toRos(const avg_msgs::msg::AvgColorRGBA & in)
{
  std_msgs::msg::ColorRGBA out;
  out.r = in.r;
  out.g = in.g;
  out.b = in.b;
  out.a = in.a;
  return out;
}

inline avg_msgs::msg::AvgPoint fromRos(const geometry_msgs::msg::Point & in)
{
  avg_msgs::msg::AvgPoint out;
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
  return out;
}

inline geometry_msgs::msg::Point toRos(const avg_msgs::msg::AvgPoint & in)
{
  geometry_msgs::msg::Point out;
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
  return out;
}

inline avg_msgs::msg::AvgPoint32 fromRos(const geometry_msgs::msg::Point32 & in)
{
  avg_msgs::msg::AvgPoint32 out;
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
  return out;
}

inline geometry_msgs::msg::Point32 toRos(const avg_msgs::msg::AvgPoint32 & in)
{
  geometry_msgs::msg::Point32 out;
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
  return out;
}

inline avg_msgs::msg::AvgQuaternion fromRos(const geometry_msgs::msg::Quaternion & in)
{
  avg_msgs::msg::AvgQuaternion out;
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
  out.w = in.w;
  return out;
}

inline geometry_msgs::msg::Quaternion toRos(const avg_msgs::msg::AvgQuaternion & in)
{
  geometry_msgs::msg::Quaternion out;
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
  out.w = in.w;
  return out;
}

inline avg_msgs::msg::AvgVector3 fromRos(const geometry_msgs::msg::Vector3 & in)
{
  avg_msgs::msg::AvgVector3 out;
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
  return out;
}

inline geometry_msgs::msg::Vector3 toRos(const avg_msgs::msg::AvgVector3 & in)
{
  geometry_msgs::msg::Vector3 out;
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
  return out;
}

inline avg_msgs::msg::AvgPose fromRos(const geometry_msgs::msg::Pose & in)
{
  avg_msgs::msg::AvgPose out;
  out.position = fromRos(in.position);
  out.orientation = fromRos(in.orientation);
  return out;
}

inline geometry_msgs::msg::Pose toRos(const avg_msgs::msg::AvgPose & in)
{
  geometry_msgs::msg::Pose out;
  out.position = toRos(in.position);
  out.orientation = toRos(in.orientation);
  return out;
}

inline avg_msgs::msg::AvgPose2D fromRos(const geometry_msgs::msg::Pose2D & in)
{
  avg_msgs::msg::AvgPose2D out;
  out.x = in.x;
  out.y = in.y;
  out.theta = in.theta;
  return out;
}

inline geometry_msgs::msg::Pose2D toRos(const avg_msgs::msg::AvgPose2D & in)
{
  geometry_msgs::msg::Pose2D out;
  out.x = in.x;
  out.y = in.y;
  out.theta = in.theta;
  return out;
}

inline avg_msgs::msg::AvgPose2D fromRos(const vision_msgs::msg::Pose2D & in)
{
  avg_msgs::msg::AvgPose2D out;
  out.x = in.position.x;
  out.y = in.position.y;
  out.theta = in.theta;
  return out;
}

inline vision_msgs::msg::Pose2D toRosVision(const avg_msgs::msg::AvgPose2D & in)
{
  vision_msgs::msg::Pose2D out;
  out.position.x = in.x;
  out.position.y = in.y;
  out.theta = in.theta;
  return out;
}

inline avg_msgs::msg::AvgPoseStamped fromRos(const geometry_msgs::msg::PoseStamped & in)
{
  avg_msgs::msg::AvgPoseStamped out;
  out.header = fromRos(in.header);
  out.pose = fromRos(in.pose);
  return out;
}

inline geometry_msgs::msg::PoseStamped toRos(const avg_msgs::msg::AvgPoseStamped & in)
{
  geometry_msgs::msg::PoseStamped out;
  out.header = toRos(in.header);
  out.pose = toRos(in.pose);
  return out;
}

inline avg_msgs::msg::AvgPoseWithCovariance fromRos(
  const geometry_msgs::msg::PoseWithCovariance & in)
{
  avg_msgs::msg::AvgPoseWithCovariance out;
  out.pose = fromRos(in.pose);
  out.covariance = in.covariance;
  return out;
}

inline geometry_msgs::msg::PoseWithCovariance toRos(
  const avg_msgs::msg::AvgPoseWithCovariance & in)
{
  geometry_msgs::msg::PoseWithCovariance out;
  out.pose = toRos(in.pose);
  out.covariance = in.covariance;
  return out;
}

inline avg_msgs::msg::AvgPoseWithCovarianceStamped fromRos(
  const geometry_msgs::msg::PoseWithCovarianceStamped & in)
{
  avg_msgs::msg::AvgPoseWithCovarianceStamped out;
  out.header = fromRos(in.header);
  out.pose = fromRos(in.pose);
  return out;
}

inline geometry_msgs::msg::PoseWithCovarianceStamped toRos(
  const avg_msgs::msg::AvgPoseWithCovarianceStamped & in)
{
  geometry_msgs::msg::PoseWithCovarianceStamped out;
  out.header = toRos(in.header);
  out.pose = toRos(in.pose);
  return out;
}

inline avg_msgs::msg::AvgTwist fromRos(const geometry_msgs::msg::Twist & in)
{
  avg_msgs::msg::AvgTwist out;
  out.linear = fromRos(in.linear);
  out.angular = fromRos(in.angular);
  return out;
}

inline geometry_msgs::msg::Twist toRos(const avg_msgs::msg::AvgTwist & in)
{
  geometry_msgs::msg::Twist out;
  out.linear = toRos(in.linear);
  out.angular = toRos(in.angular);
  return out;
}

inline avg_msgs::msg::AvgTwistStamped fromRos(const geometry_msgs::msg::TwistStamped & in)
{
  avg_msgs::msg::AvgTwistStamped out;
  out.header = fromRos(in.header);
  out.twist = fromRos(in.twist);
  return out;
}

inline geometry_msgs::msg::TwistStamped toRos(const avg_msgs::msg::AvgTwistStamped & in)
{
  geometry_msgs::msg::TwistStamped out;
  out.header = toRos(in.header);
  out.twist = toRos(in.twist);
  return out;
}

inline avg_msgs::msg::AvgTwistWithCovariance fromRos(
  const geometry_msgs::msg::TwistWithCovariance & in)
{
  avg_msgs::msg::AvgTwistWithCovariance out;
  out.twist = fromRos(in.twist);
  out.covariance = in.covariance;
  return out;
}

inline geometry_msgs::msg::TwistWithCovariance toRos(
  const avg_msgs::msg::AvgTwistWithCovariance & in)
{
  geometry_msgs::msg::TwistWithCovariance out;
  out.twist = toRos(in.twist);
  out.covariance = in.covariance;
  return out;
}

inline avg_msgs::msg::AvgTwistWithCovarianceStamped fromRos(
  const geometry_msgs::msg::TwistWithCovarianceStamped & in)
{
  avg_msgs::msg::AvgTwistWithCovarianceStamped out;
  out.header = fromRos(in.header);
  out.twist = fromRos(in.twist);
  return out;
}

inline geometry_msgs::msg::TwistWithCovarianceStamped toRos(
  const avg_msgs::msg::AvgTwistWithCovarianceStamped & in)
{
  geometry_msgs::msg::TwistWithCovarianceStamped out;
  out.header = toRos(in.header);
  out.twist = toRos(in.twist);
  return out;
}

inline avg_msgs::msg::AvgPolygon fromRos(const geometry_msgs::msg::Polygon & in)
{
  avg_msgs::msg::AvgPolygon out;
  out.points.reserve(in.points.size());
  for (const auto & point : in.points) {
    out.points.push_back(fromRos(point));
  }
  return out;
}

inline geometry_msgs::msg::Polygon toRos(const avg_msgs::msg::AvgPolygon & in)
{
  geometry_msgs::msg::Polygon out;
  out.points.reserve(in.points.size());
  for (const auto & point : in.points) {
    out.points.push_back(toRos(point));
  }
  return out;
}

inline avg_msgs::msg::AvgPolygonStamped fromRos(const geometry_msgs::msg::PolygonStamped & in)
{
  avg_msgs::msg::AvgPolygonStamped out;
  out.header = fromRos(in.header);
  out.polygon = fromRos(in.polygon);
  return out;
}

inline geometry_msgs::msg::PolygonStamped toRos(const avg_msgs::msg::AvgPolygonStamped & in)
{
  geometry_msgs::msg::PolygonStamped out;
  out.header = toRos(in.header);
  out.polygon = toRos(in.polygon);
  return out;
}

inline avg_msgs::msg::AvgTransform fromRos(const geometry_msgs::msg::Transform & in)
{
  avg_msgs::msg::AvgTransform out;
  out.translation = fromRos(in.translation);
  out.rotation = fromRos(in.rotation);
  return out;
}

inline geometry_msgs::msg::Transform toRos(const avg_msgs::msg::AvgTransform & in)
{
  geometry_msgs::msg::Transform out;
  out.translation = toRos(in.translation);
  out.rotation = toRos(in.rotation);
  return out;
}

inline avg_msgs::msg::AvgTransformStamped fromRos(
  const geometry_msgs::msg::TransformStamped & in)
{
  avg_msgs::msg::AvgTransformStamped out;
  out.header = fromRos(in.header);
  out.child_frame_id = in.child_frame_id;
  out.transform = fromRos(in.transform);
  return out;
}

inline geometry_msgs::msg::TransformStamped toRos(
  const avg_msgs::msg::AvgTransformStamped & in)
{
  geometry_msgs::msg::TransformStamped out;
  out.header = toRos(in.header);
  out.child_frame_id = in.child_frame_id;
  out.transform = toRos(in.transform);
  return out;
}

inline avg_msgs::msg::AvgMapMetaData fromRos(const nav_msgs::msg::MapMetaData & in)
{
  avg_msgs::msg::AvgMapMetaData out;
  out.map_load_time = in.map_load_time;
  out.resolution = in.resolution;
  out.width = in.width;
  out.height = in.height;
  out.origin = fromRos(in.origin);
  return out;
}

inline nav_msgs::msg::MapMetaData toRos(const avg_msgs::msg::AvgMapMetaData & in)
{
  nav_msgs::msg::MapMetaData out;
  out.map_load_time = in.map_load_time;
  out.resolution = in.resolution;
  out.width = in.width;
  out.height = in.height;
  out.origin = toRos(in.origin);
  return out;
}

inline avg_msgs::msg::AvgOccupancyGrid fromRos(const nav_msgs::msg::OccupancyGrid & in)
{
  avg_msgs::msg::AvgOccupancyGrid out;
  out.header = fromRos(in.header);
  out.info = fromRos(in.info);
  out.data = in.data;
  return out;
}

inline nav_msgs::msg::OccupancyGrid toRos(const avg_msgs::msg::AvgOccupancyGrid & in)
{
  nav_msgs::msg::OccupancyGrid out;
  out.header = toRos(in.header);
  out.info = toRos(in.info);
  out.data = in.data;
  return out;
}

inline avg_msgs::msg::AvgPath fromRos(const nav_msgs::msg::Path & in)
{
  avg_msgs::msg::AvgPath out;
  out.header = fromRos(in.header);
  out.poses.reserve(in.poses.size());
  for (const auto & pose : in.poses) {
    out.poses.push_back(fromRos(pose));
  }
  return out;
}

inline nav_msgs::msg::Path toRos(const avg_msgs::msg::AvgPath & in)
{
  nav_msgs::msg::Path out;
  out.header = toRos(in.header);
  out.poses.reserve(in.poses.size());
  for (const auto & pose : in.poses) {
    out.poses.push_back(toRos(pose));
  }
  return out;
}

inline avg_msgs::msg::AvgOdometry fromRos(const nav_msgs::msg::Odometry & in)
{
  avg_msgs::msg::AvgOdometry out;
  out.header = fromRos(in.header);
  out.child_frame_id = in.child_frame_id;
  out.pose = fromRos(in.pose);
  out.twist = fromRos(in.twist);
  return out;
}

inline nav_msgs::msg::Odometry toRos(const avg_msgs::msg::AvgOdometry & in)
{
  nav_msgs::msg::Odometry out;
  out.header = toRos(in.header);
  out.child_frame_id = in.child_frame_id;
  out.pose = toRos(in.pose);
  out.twist = toRos(in.twist);
  return out;
}

inline avg_msgs::msg::AvgNavSatStatus fromRos(const sensor_msgs::msg::NavSatStatus & in)
{
  avg_msgs::msg::AvgNavSatStatus out;
  out.status = in.status;
  out.service = in.service;
  return out;
}

inline sensor_msgs::msg::NavSatStatus toRos(const avg_msgs::msg::AvgNavSatStatus & in)
{
  sensor_msgs::msg::NavSatStatus out;
  out.status = in.status;
  out.service = in.service;
  return out;
}

inline avg_msgs::msg::AvgNavSatFix fromRos(const sensor_msgs::msg::NavSatFix & in)
{
  avg_msgs::msg::AvgNavSatFix out;
  out.header = fromRos(in.header);
  out.status = fromRos(in.status);
  out.latitude = in.latitude;
  out.longitude = in.longitude;
  out.altitude = in.altitude;
  out.position_covariance = in.position_covariance;
  out.position_covariance_type = in.position_covariance_type;
  return out;
}

inline sensor_msgs::msg::NavSatFix toRos(const avg_msgs::msg::AvgNavSatFix & in)
{
  sensor_msgs::msg::NavSatFix out;
  out.header = toRos(in.header);
  out.status = toRos(in.status);
  out.latitude = in.latitude;
  out.longitude = in.longitude;
  out.altitude = in.altitude;
  out.position_covariance = in.position_covariance;
  out.position_covariance_type = in.position_covariance_type;
  return out;
}

inline avg_msgs::msg::AvgImu fromRos(const sensor_msgs::msg::Imu & in)
{
  avg_msgs::msg::AvgImu out;
  out.header = fromRos(in.header);
  out.orientation = fromRos(in.orientation);
  out.orientation_covariance = in.orientation_covariance;
  out.angular_velocity = fromRos(in.angular_velocity);
  out.angular_velocity_covariance = in.angular_velocity_covariance;
  out.linear_acceleration = fromRos(in.linear_acceleration);
  out.linear_acceleration_covariance = in.linear_acceleration_covariance;
  return out;
}

inline sensor_msgs::msg::Imu toRos(const avg_msgs::msg::AvgImu & in)
{
  sensor_msgs::msg::Imu out;
  out.header = toRos(in.header);
  out.orientation = toRos(in.orientation);
  out.orientation_covariance = in.orientation_covariance;
  out.angular_velocity = toRos(in.angular_velocity);
  out.angular_velocity_covariance = in.angular_velocity_covariance;
  out.linear_acceleration = toRos(in.linear_acceleration);
  out.linear_acceleration_covariance = in.linear_acceleration_covariance;
  return out;
}

inline avg_msgs::msg::AvgRange fromRos(const sensor_msgs::msg::Range & in)
{
  avg_msgs::msg::AvgRange out;
  out.header = fromRos(in.header);
  out.radiation_type = in.radiation_type;
  out.field_of_view = in.field_of_view;
  out.min_range = in.min_range;
  out.max_range = in.max_range;
  out.range = in.range;
  return out;
}

inline sensor_msgs::msg::Range toRos(const avg_msgs::msg::AvgRange & in)
{
  sensor_msgs::msg::Range out;
  out.header = toRos(in.header);
  out.radiation_type = in.radiation_type;
  out.field_of_view = in.field_of_view;
  out.min_range = in.min_range;
  out.max_range = in.max_range;
  out.range = in.range;
  return out;
}

inline avg_msgs::msg::AvgPointField fromRos(const sensor_msgs::msg::PointField & in)
{
  avg_msgs::msg::AvgPointField out;
  out.name = in.name;
  out.offset = in.offset;
  out.datatype = in.datatype;
  out.count = in.count;
  return out;
}

inline sensor_msgs::msg::PointField toRos(const avg_msgs::msg::AvgPointField & in)
{
  sensor_msgs::msg::PointField out;
  out.name = in.name;
  out.offset = in.offset;
  out.datatype = in.datatype;
  out.count = in.count;
  return out;
}

inline avg_msgs::msg::AvgPointCloud2 fromRos(const sensor_msgs::msg::PointCloud2 & in)
{
  avg_msgs::msg::AvgPointCloud2 out;
  out.header = fromRos(in.header);
  out.height = in.height;
  out.width = in.width;
  out.fields.reserve(in.fields.size());
  for (const auto & field : in.fields) {
    out.fields.push_back(fromRos(field));
  }
  out.is_bigendian = in.is_bigendian;
  out.point_step = in.point_step;
  out.row_step = in.row_step;
  out.data = in.data;
  out.is_dense = in.is_dense;
  return out;
}

inline sensor_msgs::msg::PointCloud2 toRos(const avg_msgs::msg::AvgPointCloud2 & in)
{
  sensor_msgs::msg::PointCloud2 out;
  out.header = toRos(in.header);
  out.height = in.height;
  out.width = in.width;
  out.fields.reserve(in.fields.size());
  for (const auto & field : in.fields) {
    out.fields.push_back(toRos(field));
  }
  out.is_bigendian = in.is_bigendian;
  out.point_step = in.point_step;
  out.row_step = in.row_step;
  out.data = in.data;
  out.is_dense = in.is_dense;
  return out;
}

inline avg_msgs::msg::AvgLaserScan fromRos(const sensor_msgs::msg::LaserScan & in)
{
  avg_msgs::msg::AvgLaserScan out;
  out.header = fromRos(in.header);
  out.angle_min = in.angle_min;
  out.angle_max = in.angle_max;
  out.angle_increment = in.angle_increment;
  out.time_increment = in.time_increment;
  out.scan_time = in.scan_time;
  out.range_min = in.range_min;
  out.range_max = in.range_max;
  out.ranges = in.ranges;
  out.intensities = in.intensities;
  return out;
}

inline sensor_msgs::msg::LaserScan toRos(const avg_msgs::msg::AvgLaserScan & in)
{
  sensor_msgs::msg::LaserScan out;
  out.header = toRos(in.header);
  out.angle_min = in.angle_min;
  out.angle_max = in.angle_max;
  out.angle_increment = in.angle_increment;
  out.time_increment = in.time_increment;
  out.scan_time = in.scan_time;
  out.range_min = in.range_min;
  out.range_max = in.range_max;
  out.ranges = in.ranges;
  out.intensities = in.intensities;
  return out;
}

inline avg_msgs::msg::AvgRegionOfInterest fromRos(
  const sensor_msgs::msg::RegionOfInterest & in)
{
  avg_msgs::msg::AvgRegionOfInterest out;
  out.x_offset = in.x_offset;
  out.y_offset = in.y_offset;
  out.height = in.height;
  out.width = in.width;
  out.do_rectify = in.do_rectify;
  return out;
}

inline sensor_msgs::msg::RegionOfInterest toRos(
  const avg_msgs::msg::AvgRegionOfInterest & in)
{
  sensor_msgs::msg::RegionOfInterest out;
  out.x_offset = in.x_offset;
  out.y_offset = in.y_offset;
  out.height = in.height;
  out.width = in.width;
  out.do_rectify = in.do_rectify;
  return out;
}

inline avg_msgs::msg::AvgImage fromRos(const sensor_msgs::msg::Image & in)
{
  avg_msgs::msg::AvgImage out;
  out.header = fromRos(in.header);
  out.height = in.height;
  out.width = in.width;
  out.encoding = in.encoding;
  out.is_bigendian = in.is_bigendian;
  out.step = in.step;
  out.data = in.data;
  return out;
}

inline sensor_msgs::msg::Image toRos(const avg_msgs::msg::AvgImage & in)
{
  sensor_msgs::msg::Image out;
  out.header = toRos(in.header);
  out.height = in.height;
  out.width = in.width;
  out.encoding = in.encoding;
  out.is_bigendian = in.is_bigendian;
  out.step = in.step;
  out.data = in.data;
  return out;
}

inline avg_msgs::msg::AvgCameraInfo fromRos(const sensor_msgs::msg::CameraInfo & in)
{
  avg_msgs::msg::AvgCameraInfo out;
  out.header = fromRos(in.header);
  out.height = in.height;
  out.width = in.width;
  out.distortion_model = in.distortion_model;
  out.d = in.d;
  out.k = in.k;
  out.r = in.r;
  out.p = in.p;
  out.binning_x = in.binning_x;
  out.binning_y = in.binning_y;
  out.roi = fromRos(in.roi);
  return out;
}

inline sensor_msgs::msg::CameraInfo toRos(const avg_msgs::msg::AvgCameraInfo & in)
{
  sensor_msgs::msg::CameraInfo out;
  out.header = toRos(in.header);
  out.height = in.height;
  out.width = in.width;
  out.distortion_model = in.distortion_model;
  out.d = in.d;
  out.k = in.k;
  out.r = in.r;
  out.p = in.p;
  out.binning_x = in.binning_x;
  out.binning_y = in.binning_y;
  out.roi = toRos(in.roi);
  return out;
}

inline avg_msgs::msg::AvgGoalInfo fromRos(const action_msgs::msg::GoalInfo & in)
{
  avg_msgs::msg::AvgGoalInfo out;
  out.goal_id = in.goal_id;
  out.stamp = in.stamp;
  return out;
}

inline action_msgs::msg::GoalInfo toRos(const avg_msgs::msg::AvgGoalInfo & in)
{
  action_msgs::msg::GoalInfo out;
  out.goal_id = in.goal_id;
  out.stamp = in.stamp;
  return out;
}

inline avg_msgs::msg::AvgGoalStatus fromRos(const action_msgs::msg::GoalStatus & in)
{
  avg_msgs::msg::AvgGoalStatus out;
  out.goal_info = fromRos(in.goal_info);
  out.status = in.status;
  return out;
}

inline action_msgs::msg::GoalStatus toRos(const avg_msgs::msg::AvgGoalStatus & in)
{
  action_msgs::msg::GoalStatus out;
  out.goal_info = toRos(in.goal_info);
  out.status = in.status;
  return out;
}

inline avg_msgs::msg::AvgGoalStatusArray fromRos(
  const action_msgs::msg::GoalStatusArray & in)
{
  avg_msgs::msg::AvgGoalStatusArray out;
  out.status_list.reserve(in.status_list.size());
  for (const auto & status : in.status_list) {
    out.status_list.push_back(fromRos(status));
  }
  return out;
}

inline action_msgs::msg::GoalStatusArray toRos(
  const avg_msgs::msg::AvgGoalStatusArray & in)
{
  action_msgs::msg::GoalStatusArray out;
  out.status_list.reserve(in.status_list.size());
  for (const auto & status : in.status_list) {
    out.status_list.push_back(toRos(status));
  }
  return out;
}

inline avg_msgs::msg::AvgBoundingBox2D fromRos(const vision_msgs::msg::BoundingBox2D & in)
{
  avg_msgs::msg::AvgBoundingBox2D out;
  out.center = fromRos(in.center);
  out.size_x = in.size_x;
  out.size_y = in.size_y;
  return out;
}

inline vision_msgs::msg::BoundingBox2D toRos(const avg_msgs::msg::AvgBoundingBox2D & in)
{
  vision_msgs::msg::BoundingBox2D out;
  out.center = toRosVision(in.center);
  out.size_x = in.size_x;
  out.size_y = in.size_y;
  return out;
}

inline avg_msgs::msg::AvgObjectHypothesis fromRos(
  const vision_msgs::msg::ObjectHypothesis & in)
{
  avg_msgs::msg::AvgObjectHypothesis out;
  out.class_id = in.class_id;
  out.score = in.score;
  return out;
}

inline vision_msgs::msg::ObjectHypothesis toRos(
  const avg_msgs::msg::AvgObjectHypothesis & in)
{
  vision_msgs::msg::ObjectHypothesis out;
  out.class_id = in.class_id;
  out.score = in.score;
  return out;
}

inline avg_msgs::msg::AvgObjectHypothesisWithPose fromRos(
  const vision_msgs::msg::ObjectHypothesisWithPose & in)
{
  avg_msgs::msg::AvgObjectHypothesisWithPose out;
  out.hypothesis = fromRos(in.hypothesis);
  out.pose = fromRos(in.pose);
  return out;
}

inline vision_msgs::msg::ObjectHypothesisWithPose toRos(
  const avg_msgs::msg::AvgObjectHypothesisWithPose & in)
{
  vision_msgs::msg::ObjectHypothesisWithPose out;
  out.hypothesis = toRos(in.hypothesis);
  out.pose = toRos(in.pose);
  return out;
}

inline avg_msgs::msg::AvgDetection2D fromRos(const vision_msgs::msg::Detection2D & in)
{
  avg_msgs::msg::AvgDetection2D out;
  out.header = fromRos(in.header);
  out.results.reserve(in.results.size());
  for (const auto & result : in.results) {
    out.results.push_back(fromRos(result));
  }
  out.bbox = fromRos(in.bbox);
  out.id = in.id;
  return out;
}

inline vision_msgs::msg::Detection2D toRos(const avg_msgs::msg::AvgDetection2D & in)
{
  vision_msgs::msg::Detection2D out;
  out.header = toRos(in.header);
  out.results.reserve(in.results.size());
  for (const auto & result : in.results) {
    out.results.push_back(toRos(result));
  }
  out.bbox = toRos(in.bbox);
  out.id = in.id;
  return out;
}

inline avg_msgs::msg::AvgDetection2DArray fromRos(
  const vision_msgs::msg::Detection2DArray & in)
{
  avg_msgs::msg::AvgDetection2DArray out;
  out.header = fromRos(in.header);
  out.detections.reserve(in.detections.size());
  for (const auto & detection : in.detections) {
    out.detections.push_back(fromRos(detection));
  }
  return out;
}

inline vision_msgs::msg::Detection2DArray toRos(
  const avg_msgs::msg::AvgDetection2DArray & in)
{
  vision_msgs::msg::Detection2DArray out;
  out.header = toRos(in.header);
  out.detections.reserve(in.detections.size());
  for (const auto & detection : in.detections) {
    out.detections.push_back(toRos(detection));
  }
  return out;
}

inline avg_msgs::msg::AvgMarker fromRos(const visualization_msgs::msg::Marker & in)
{
  avg_msgs::msg::AvgMarker out;
  out.header = fromRos(in.header);
  out.ns = in.ns;
  out.id = in.id;
  out.type = in.type;
  out.action = in.action;
  out.pose = fromRos(in.pose);
  out.scale = fromRos(in.scale);
  out.color = fromRos(in.color);
  out.lifetime = in.lifetime;
  out.frame_locked = in.frame_locked;
  out.points.reserve(in.points.size());
  for (const auto & point : in.points) {
    out.points.push_back(fromRos(point));
  }
  out.colors.reserve(in.colors.size());
  for (const auto & color : in.colors) {
    out.colors.push_back(fromRos(color));
  }
  out.text = in.text;
  out.mesh_resource = in.mesh_resource;
  out.mesh_use_embedded_materials = in.mesh_use_embedded_materials;
  return out;
}

inline visualization_msgs::msg::Marker toRos(const avg_msgs::msg::AvgMarker & in)
{
  visualization_msgs::msg::Marker out;
  out.header = toRos(in.header);
  out.ns = in.ns;
  out.id = in.id;
  out.type = in.type;
  out.action = in.action;
  out.pose = toRos(in.pose);
  out.scale = toRos(in.scale);
  out.color = toRos(in.color);
  out.lifetime = in.lifetime;
  out.frame_locked = in.frame_locked;
  out.points.reserve(in.points.size());
  for (const auto & point : in.points) {
    out.points.push_back(toRos(point));
  }
  out.colors.reserve(in.colors.size());
  for (const auto & color : in.colors) {
    out.colors.push_back(toRos(color));
  }
  out.text = in.text;
  out.mesh_resource = in.mesh_resource;
  out.mesh_use_embedded_materials = in.mesh_use_embedded_materials;
  return out;
}

inline avg_msgs::msg::AvgMarkerArray fromRos(const visualization_msgs::msg::MarkerArray & in)
{
  avg_msgs::msg::AvgMarkerArray out;
  out.markers.reserve(in.markers.size());
  for (const auto & marker : in.markers) {
    out.markers.push_back(fromRos(marker));
  }
  return out;
}

inline visualization_msgs::msg::MarkerArray toRos(
  const avg_msgs::msg::AvgMarkerArray & in)
{
  visualization_msgs::msg::MarkerArray out;
  out.markers.reserve(in.markers.size());
  for (const auto & marker : in.markers) {
    out.markers.push_back(toRos(marker));
  }
  return out;
}

}  // namespace avg_msgs::conversions
