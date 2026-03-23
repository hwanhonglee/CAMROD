# CAMROD Dependency Source Audit

- Workspace root: `/home/camrod_ws`
- Modules scanned: 10
- Workspace packages detected: 51

## Summary by module

### camrod_bringup
- Local external packages used: 0
- Still binary-only from `/opt/ros`: 8
  - ament_cmake, ament_lint_auto, ament_lint_common, diagnostic_msgs, rclcpp, rclpy, ros2launch, sensor_msgs_py
- System/rosdep (non-ROS package name): 0

### camrod_common
- Local external packages used: 0
- Still binary-only from `/opt/ros`: 23
  - action_msgs, ament_cmake, ament_cmake_gtest, ament_cmake_python, ament_lint_auto, ament_lint_common, builtin_interfaces, geometry_msgs, nav_msgs, pluginlib, rclcpp, rclpy, rcutils, rosidl_default_generators, rosidl_default_runtime, rviz2, rviz_common, rviz_default_plugins, rviz_rendering, sensor_msgs, std_msgs, visualization_msgs, yaml_cpp_vendor
- System/rosdep (non-ROS package name): 1
  - python3-numpy

### camrod_localization
- Local external packages used: 0
- Still binary-only from `/opt/ros`: 26
  - ament_cmake, ament_cmake_gtest, ament_lint_auto, ament_lint_common, angles, builtin_interfaces, diagnostic_msgs, diagnostic_updater, geographic_msgs, geometry_msgs, launch_ros, launch_testing_ament_cmake, message_filters, nav_msgs, rclcpp, rclpy, rosidl_default_generators, rosidl_default_runtime, sensor_msgs, std_msgs, std_srvs, tf2, tf2_eigen, tf2_geometry_msgs, tf2_ros, yaml_cpp_vendor
- System/rosdep (non-ROS package name): 6
  - eigen, geographiclib, libboost-dev, python3, python3-dev, yaml-cpp

### camrod_map
- Local external packages used: 0
- Still binary-only from `/opt/ros`: 14
  - ament_cmake, ament_cmake_core, geometry_msgs, mrt_cmake_modules, nav_msgs, rclcpp, rclpy, ros2cli, ros_environment, std_msgs, tf2, tf2_geometry_msgs, tf2_ros, visualization_msgs
- System/rosdep (non-ROS package name): 8
  - boost, catkin, eigen, geographiclib, gtest, libboost-python-dev, pugixml-dev, rosbash

### camrod_perception
- Local external packages used: 0
- Still binary-only from `/opt/ros`: 20
  - ament_cmake, ament_cmake_gtest, ament_cmake_pytest, ament_cmake_python, ament_cmake_ros, ament_index_python, ament_lint_auto, ament_lint_common, geometry_msgs, launch, nav_msgs, python_cmake_module, rclcpp, rclpy, rcpputils, sensor_msgs, tf2, tf2_geometry_msgs, tf2_ros, visualization_msgs
- System/rosdep (non-ROS package name): 6
  - libboost-dev, libboost-python, libboost-python-dev, libopencv-dev, python3-numpy, python3-opencv

### camrod_planning
- Local external packages used: 0
- Still binary-only from `/opt/ros`: 48
  - action_msgs, ament_cmake, ament_cmake_core, ament_cmake_gtest, ament_cmake_pytest, ament_cmake_python, ament_lint_auto, ament_lint_common, angles, behaviortree_cpp_v3, bond, bondcpp, builtin_interfaces, diagnostic_msgs, diagnostic_updater, eigen3_cmake_module, geometry_msgs, launch, launch_ros, launch_testing, launch_testing_ament_cmake, launch_testing_ros, lifecycle_msgs, map_msgs, message_filters, nav2_map_server, nav_2d_msgs, nav_2d_utils, nav_msgs, ompl, osrf_pycommon, pluginlib, rcl_interfaces, rclcpp, rclcpp_action, rclcpp_lifecycle, rclpy, rosidl_default_generators, sensor_msgs, sensor_msgs_py, std_msgs, std_srvs, test_msgs, tf2, tf2_geometry_msgs, tf2_ros, tf2_sensor_msgs, visualization_msgs
- System/rosdep (non-ROS package name): 6
  - eigen, libboost-program-options, libboost-program-options-dev, nlohmann-json-dev, python3-numpy, python3-yaml

### camrod_platform
- Local external packages used: 0
- Still binary-only from `/opt/ros`: 12
  - ament_cmake, ament_lint_auto, ament_lint_common, geometry_msgs, nav_msgs, rclcpp, rclpy, std_msgs, tf2, tf2_geometry_msgs, tf2_ros, visualization_msgs
- System/rosdep (non-ROS package name): 0

### camrod_sensing
- Local external packages used: 0
- Still binary-only from `/opt/ros`: 25
  - ament_cmake, ament_cmake_gtest, ament_cmake_ros, ament_lint_auto, ament_lint_common, builtin_interfaces, diagnostic_msgs, diagnostic_updater, geometry_msgs, message_filters, nav_msgs, nmea_msgs, pcl_msgs, rcl_interfaces, rclcpp, rclcpp_components, rclpy, rosidl_default_generators, rosidl_default_runtime, rtcm_msgs, sensor_msgs, std_msgs, tf2, tf2_geometry_msgs, tf2_ros
- System/rosdep (non-ROS package name): 9
  - asio, eigen, libpcl-all-dev, libpcl-common, libpcl-features, libpcl-filters, libpcl-io, libpcl-segmentation, libpcl-surface

### camrod_sensor_kit
- Local external packages used: 0
- Still binary-only from `/opt/ros`: 13
  - ament_cmake, ament_lint_auto, ament_lint_common, diagnostic_msgs, geometry_msgs, nav_msgs, rclcpp, rclpy, tf2, tf2_geometry_msgs, tf2_msgs, tf2_ros, visualization_msgs
- System/rosdep (non-ROS package name): 0

### camrod_system
- Local external packages used: 0
- Still binary-only from `/opt/ros`: 5
  - ament_cmake, ament_lint_auto, ament_lint_common, diagnostic_msgs, rclpy
- System/rosdep (non-ROS package name): 0

## Detailed per package

### camrod_bringup
- `camrod_bringup`
  - internal deps: avg_msgs, camrod_localization, camrod_map, camrod_planning, camrod_platform, camrod_sensing, camrod_sensor_kit, robot_localization
  - binary /opt/ros deps: ament_cmake, ament_lint_auto, ament_lint_common, diagnostic_msgs, rclcpp, rclpy, ros2launch, sensor_msgs_py

### camrod_common
- `avg_msgs`
  - internal deps: nav2_msgs, vision_msgs
  - binary /opt/ros deps: action_msgs, ament_cmake, builtin_interfaces, geometry_msgs, nav_msgs, rcutils, rosidl_default_generators, rosidl_default_runtime, sensor_msgs, std_msgs, visualization_msgs
- `vision_msgs`
  - binary /opt/ros deps: ament_cmake, ament_cmake_gtest, ament_lint_auto, ament_lint_common, geometry_msgs, rosidl_default_generators, rosidl_default_runtime, std_msgs
- `vision_msgs_rviz_plugins`
  - internal deps: vision_msgs
  - binary /opt/ros deps: ament_cmake, ament_cmake_python, ament_lint_auto, ament_lint_common, pluginlib, rclcpp, rclpy, rviz2, rviz_common, rviz_default_plugins, rviz_rendering, yaml_cpp_vendor
  - system/rosdep deps: python3-numpy

### camrod_localization
- `robot_localization`
  - binary /opt/ros deps: ament_cmake, ament_cmake_gtest, ament_lint_auto, ament_lint_common, angles, builtin_interfaces, diagnostic_msgs, diagnostic_updater, geographic_msgs, geometry_msgs, launch_ros, launch_testing_ament_cmake, message_filters, nav_msgs, rclcpp, rosidl_default_generators, rosidl_default_runtime, sensor_msgs, std_msgs, std_srvs, tf2, tf2_eigen, tf2_geometry_msgs, tf2_ros, yaml_cpp_vendor
  - system/rosdep deps: eigen, geographiclib, libboost-dev
- `camrod_localization`
  - internal deps: avg_msgs, lanelet2_core, lanelet2_io, lanelet2_projection
  - binary /opt/ros deps: ament_cmake, ament_lint_auto, ament_lint_common, geometry_msgs, nav_msgs, rclcpp, rclpy, sensor_msgs, std_msgs, tf2, tf2_geometry_msgs, tf2_ros
  - system/rosdep deps: eigen, python3, python3-dev, yaml-cpp

### camrod_map
- `lanelet2`
  - internal deps: lanelet2_core, lanelet2_examples, lanelet2_io, lanelet2_maps, lanelet2_matching, lanelet2_projection, lanelet2_python, lanelet2_routing, lanelet2_traffic_rules, lanelet2_validation
  - binary /opt/ros deps: ament_cmake_core, ros_environment
  - system/rosdep deps: catkin
- `lanelet2_core`
  - binary /opt/ros deps: ament_cmake_core, mrt_cmake_modules
  - system/rosdep deps: boost, catkin, eigen, gtest
- `lanelet2_examples`
  - internal deps: lanelet2_core, lanelet2_io, lanelet2_matching, lanelet2_projection, lanelet2_python, lanelet2_routing, lanelet2_traffic_rules
  - binary /opt/ros deps: ament_cmake_core, mrt_cmake_modules, ros2cli
  - system/rosdep deps: catkin, gtest, rosbash
- `lanelet2_io`
  - internal deps: lanelet2_core
  - binary /opt/ros deps: ament_cmake_core, mrt_cmake_modules
  - system/rosdep deps: boost, catkin, gtest, pugixml-dev
- `lanelet2_maps`
  - internal deps: lanelet2_core
  - binary /opt/ros deps: ament_cmake_core, mrt_cmake_modules
  - system/rosdep deps: catkin
- `lanelet2_matching`
  - internal deps: lanelet2_core, lanelet2_io, lanelet2_maps, lanelet2_projection, lanelet2_traffic_rules
  - binary /opt/ros deps: ament_cmake_core, mrt_cmake_modules
  - system/rosdep deps: catkin, gtest
- `lanelet2_projection`
  - internal deps: lanelet2_io
  - binary /opt/ros deps: ament_cmake_core, mrt_cmake_modules
  - system/rosdep deps: catkin, geographiclib, gtest
- `lanelet2_python`
  - internal deps: lanelet2_core, lanelet2_io, lanelet2_matching, lanelet2_projection, lanelet2_routing, lanelet2_traffic_rules
  - binary /opt/ros deps: ament_cmake_core, mrt_cmake_modules
  - system/rosdep deps: catkin, gtest, libboost-python-dev
- `lanelet2_routing`
  - internal deps: lanelet2_core, lanelet2_traffic_rules
  - binary /opt/ros deps: ament_cmake_core, mrt_cmake_modules
  - system/rosdep deps: boost, catkin, gtest
- `lanelet2_traffic_rules`
  - internal deps: lanelet2_core
  - binary /opt/ros deps: ament_cmake_core, mrt_cmake_modules
  - system/rosdep deps: catkin, gtest
- `lanelet2_validation`
  - internal deps: lanelet2_core, lanelet2_io, lanelet2_maps, lanelet2_projection, lanelet2_routing, lanelet2_traffic_rules
  - binary /opt/ros deps: ament_cmake_core, mrt_cmake_modules
  - system/rosdep deps: catkin, gtest
- `camrod_map`
  - internal deps: avg_msgs, lanelet2_core, lanelet2_io, lanelet2_projection
  - binary /opt/ros deps: ament_cmake, geometry_msgs, nav_msgs, rclcpp, rclpy, std_msgs, tf2, tf2_geometry_msgs, tf2_ros, visualization_msgs

### camrod_perception
- `cv_bridge`
  - binary /opt/ros deps: ament_cmake_gtest, ament_cmake_pytest, ament_cmake_ros, ament_index_python, ament_lint_auto, ament_lint_common, python_cmake_module, rcpputils, sensor_msgs
  - system/rosdep deps: libboost-dev, libboost-python, libboost-python-dev, libopencv-dev, python3-numpy, python3-opencv
- `image_geometry`
  - binary /opt/ros deps: ament_cmake_gtest, ament_cmake_pytest, ament_cmake_python, ament_cmake_ros, sensor_msgs
  - system/rosdep deps: libopencv-dev
- `opencv_tests`
  - internal deps: cv_bridge
  - binary /opt/ros deps: launch, rclpy, sensor_msgs
- `vision_opencv`
  - internal deps: cv_bridge, image_geometry
  - binary /opt/ros deps: ament_cmake
- `camrod_perception`
  - internal deps: avg_msgs, image_geometry, vision_msgs
  - binary /opt/ros deps: ament_cmake, ament_lint_auto, ament_lint_common, geometry_msgs, nav_msgs, rclcpp, rclpy, sensor_msgs, tf2, tf2_geometry_msgs, tf2_ros, visualization_msgs

### camrod_planning
- `laser_geometry`
  - binary /opt/ros deps: ament_cmake, ament_cmake_gtest, ament_cmake_pytest, ament_cmake_python, ament_lint_auto, ament_lint_common, eigen3_cmake_module, rclcpp, rclpy, sensor_msgs, sensor_msgs_py, tf2
  - system/rosdep deps: eigen, python3-numpy
- `nav2_behavior_tree`
  - internal deps: nav2_common, nav2_msgs, nav2_util
  - binary /opt/ros deps: ament_cmake, ament_cmake_gtest, ament_lint_auto, ament_lint_common, behaviortree_cpp_v3, builtin_interfaces, geometry_msgs, lifecycle_msgs, nav_msgs, rclcpp, rclcpp_action, rclcpp_lifecycle, sensor_msgs, std_msgs, std_srvs, test_msgs, tf2, tf2_geometry_msgs, tf2_ros
- `nav2_bt_navigator`
  - internal deps: nav2_behavior_tree, nav2_common, nav2_core, nav2_msgs, nav2_util
  - binary /opt/ros deps: ament_cmake, ament_lint_auto, ament_lint_common, behaviortree_cpp_v3, geometry_msgs, nav_msgs, rclcpp, rclcpp_action, rclcpp_lifecycle, std_msgs, std_srvs, tf2_ros
- `nav2_common`
  - binary /opt/ros deps: ament_cmake_core, ament_cmake_python, launch, launch_ros, osrf_pycommon, rclpy
  - system/rosdep deps: python3-yaml
- `nav2_controller`
  - internal deps: nav2_common, nav2_core, nav2_msgs, nav2_util
  - binary /opt/ros deps: ament_cmake, ament_cmake_gtest, ament_cmake_pytest, ament_lint_auto, ament_lint_common, angles, nav_2d_msgs, nav_2d_utils, pluginlib, rclcpp, rclcpp_action, std_msgs
- `nav2_core`
  - internal deps: nav2_common, nav2_costmap_2d, nav2_util
  - binary /opt/ros deps: ament_cmake, ament_cmake_gtest, ament_cmake_pytest, ament_lint_auto, ament_lint_common, geometry_msgs, launch, launch_testing, nav_msgs, pluginlib, rclcpp, rclcpp_lifecycle, std_msgs, tf2_ros
- `nav2_costmap_2d`
  - internal deps: laser_geometry, nav2_common, nav2_lifecycle_manager, nav2_msgs, nav2_util, nav2_voxel_grid
  - binary /opt/ros deps: ament_cmake, ament_cmake_gtest, ament_lint_auto, ament_lint_common, angles, geometry_msgs, launch, launch_testing, map_msgs, message_filters, nav2_map_server, nav_msgs, pluginlib, rclcpp, rclcpp_lifecycle, sensor_msgs, std_msgs, std_srvs, tf2, tf2_geometry_msgs, tf2_ros, tf2_sensor_msgs, visualization_msgs
- `nav2_lifecycle_manager`
  - internal deps: nav2_common, nav2_msgs, nav2_util
  - binary /opt/ros deps: ament_cmake, ament_cmake_gtest, ament_cmake_pytest, ament_lint_auto, ament_lint_common, bondcpp, diagnostic_updater, geometry_msgs, lifecycle_msgs, rclcpp_action, rclcpp_lifecycle, std_msgs, std_srvs, tf2_geometry_msgs
- `nav2_msgs`
  - internal deps: nav2_common
  - binary /opt/ros deps: action_msgs, ament_cmake, builtin_interfaces, geometry_msgs, nav_msgs, rclcpp, rosidl_default_generators, std_msgs
- `nav2_planner`
  - internal deps: nav2_common, nav2_core, nav2_costmap_2d, nav2_msgs, nav2_util
  - binary /opt/ros deps: ament_cmake, ament_lint_auto, ament_lint_common, builtin_interfaces, geometry_msgs, nav_msgs, pluginlib, rclcpp, rclcpp_action, rclcpp_lifecycle, tf2_ros, visualization_msgs
- `nav2_regulated_pure_pursuit_controller`
  - internal deps: nav2_common, nav2_core, nav2_costmap_2d, nav2_msgs, nav2_util
  - binary /opt/ros deps: ament_cmake, ament_cmake_gtest, ament_lint_auto, ament_lint_common, geometry_msgs, pluginlib, rclcpp, tf2, tf2_geometry_msgs
- `nav2_smac_planner`
  - internal deps: nav2_common, nav2_core, nav2_costmap_2d, nav2_msgs, nav2_util
  - binary /opt/ros deps: ament_cmake, ament_cmake_gtest, ament_cmake_pytest, ament_lint_auto, ament_lint_common, angles, builtin_interfaces, eigen3_cmake_module, geometry_msgs, nav_msgs, ompl, pluginlib, rclcpp, rclcpp_action, rclcpp_lifecycle, tf2_ros, visualization_msgs
  - system/rosdep deps: eigen, nlohmann-json-dev
- `nav2_util`
  - internal deps: nav2_common, nav2_msgs
  - binary /opt/ros deps: action_msgs, ament_cmake, ament_cmake_gtest, ament_cmake_pytest, ament_lint_auto, ament_lint_common, bond, bondcpp, geometry_msgs, launch, launch_testing_ament_cmake, launch_testing_ros, lifecycle_msgs, nav_msgs, rcl_interfaces, rclcpp, rclcpp_action, rclcpp_lifecycle, std_srvs, test_msgs, tf2, tf2_geometry_msgs, tf2_ros
  - system/rosdep deps: libboost-program-options, libboost-program-options-dev
- `nav2_voxel_grid`
  - internal deps: nav2_common
  - binary /opt/ros deps: ament_cmake, ament_cmake_gtest, ament_lint_auto, ament_lint_common, rclcpp
- `camrod_planning`
  - internal deps: avg_msgs, camrod_map, lanelet2_core, lanelet2_io, lanelet2_projection, nav2_bt_navigator, nav2_controller, nav2_costmap_2d, nav2_lifecycle_manager, nav2_msgs, nav2_planner
  - binary /opt/ros deps: action_msgs, ament_cmake, ament_lint_auto, ament_lint_common, diagnostic_msgs, geometry_msgs, nav_msgs, rclcpp, rclcpp_action, rclpy, std_msgs

### camrod_platform
- `camrod_platform`
  - internal deps: avg_msgs, camrod_sensor_kit
  - binary /opt/ros deps: ament_cmake, ament_lint_auto, ament_lint_common, geometry_msgs, nav_msgs, rclcpp, rclpy, std_msgs, tf2, tf2_geometry_msgs, tf2_ros, visualization_msgs

### camrod_sensing
- `pcl_conversions`
  - binary /opt/ros deps: ament_cmake, ament_cmake_gtest, message_filters, pcl_msgs, rclcpp, sensor_msgs, std_msgs
  - system/rosdep deps: eigen, libpcl-all-dev, libpcl-common, libpcl-io
- `pcl_ros`
  - internal deps: pcl_conversions
  - binary /opt/ros deps: ament_cmake, ament_cmake_gtest, ament_lint_auto, ament_lint_common, geometry_msgs, rclcpp, rclcpp_components, sensor_msgs, tf2, tf2_geometry_msgs, tf2_ros
  - system/rosdep deps: eigen, libpcl-all-dev, libpcl-common, libpcl-features, libpcl-filters, libpcl-io, libpcl-segmentation, libpcl-surface
- `perception_pcl`
  - internal deps: pcl_conversions, pcl_ros
  - binary /opt/ros deps: ament_cmake, pcl_msgs
- `ublox`
  - internal deps: ublox_gps, ublox_msgs, ublox_serialization
  - binary /opt/ros deps: ament_cmake
- `ublox_gps`
  - internal deps: ublox_msgs, ublox_serialization
  - binary /opt/ros deps: ament_cmake_ros, diagnostic_msgs, diagnostic_updater, geometry_msgs, nmea_msgs, rcl_interfaces, rclcpp, rclcpp_components, rtcm_msgs, sensor_msgs, std_msgs, tf2
  - system/rosdep deps: asio
- `ublox_msgs`
  - internal deps: ublox_serialization
  - binary /opt/ros deps: ament_cmake_ros, rosidl_default_generators, sensor_msgs, std_msgs
- `ublox_serialization`
  - binary /opt/ros deps: ament_cmake
- `vanjee_lidar_msg`
  - binary /opt/ros deps: ament_cmake, ament_lint_auto, ament_lint_common, builtin_interfaces, rclcpp, rosidl_default_generators, rosidl_default_runtime, std_msgs
- `vanjee_lidar_sdk`
  - internal deps: vanjee_lidar_msg
  - binary /opt/ros deps: ament_cmake, rclcpp, sensor_msgs, std_msgs
- `camrod_sensing`
  - internal deps: avg_msgs, pcl_conversions
  - binary /opt/ros deps: ament_cmake, ament_lint_auto, ament_lint_common, geometry_msgs, nav_msgs, rclcpp, rclpy, sensor_msgs, tf2, tf2_geometry_msgs, tf2_ros
  - system/rosdep deps: libpcl-all-dev

### camrod_sensor_kit
- `camrod_sensor_kit`
  - internal deps: avg_msgs
  - binary /opt/ros deps: ament_cmake, ament_lint_auto, ament_lint_common, diagnostic_msgs, geometry_msgs, nav_msgs, rclcpp, rclpy, tf2, tf2_geometry_msgs, tf2_msgs, tf2_ros, visualization_msgs

### camrod_system
- `camrod_system`
  - internal deps: avg_msgs
  - binary /opt/ros deps: ament_cmake, ament_lint_auto, ament_lint_common, diagnostic_msgs, rclpy
