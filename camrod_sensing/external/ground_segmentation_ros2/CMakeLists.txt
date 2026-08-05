cmake_minimum_required(VERSION 3.9)
project(ground_segmentation_ros2)

# HH_260604: Build with C++17 because this external node uses structured bindings and if-init statements.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS
  common
  io
  filters
  kdtree
  segmentation
)
find_package(nanoflann REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
  message_filters
)

add_library(ground_segmentation_ros2_component SHARED src/ground_segmentation_ros2_node.cpp)

# HH_260721 - Keep the header-only ground-segmentation core inside this ROS 2
# package so the retired standalone package cannot re-enter the build graph.
target_include_directories(ground_segmentation_ros2_component PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${pcl_conversions_INCLUDE_DIRS}
)

ament_target_dependencies(ground_segmentation_ros2_component
  ${dependencies}
)

target_link_libraries(ground_segmentation_ros2_component
  ${PCL_LIBRARIES}
  nanoflann::nanoflann
)

rclcpp_components_register_node(
  ground_segmentation_ros2_component
  PLUGIN "GroundSegmentationNode"
  EXECUTABLE ground_segmentation_ros2_node
)

install(
  TARGETS ground_segmentation_ros2_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # HH_260721 - Preserve the audited upstream algorithm formatting; CAMROD
  # validates this vendored package through compilation and runtime smoke tests.
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
