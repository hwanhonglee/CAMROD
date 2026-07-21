cmake_minimum_required(VERSION 3.8)
project(camrod_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# HH_260720 - Establish one OpenCV target set before cv_bridge/image_geometry/PCL are imported.
find_package(opencv4_vendor REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d imgproc imgcodecs)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(avg_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree search filters segmentation)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(apriltag REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(obstacle_fusion_node src/obstacle_fusion_node.cpp)
ament_target_dependencies(obstacle_fusion_node
  avg_msgs
  rclcpp
  sensor_msgs
  geometry_msgs
  vision_msgs
  visualization_msgs
  cv_bridge
  message_filters
)
target_include_directories(obstacle_fusion_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(obstacle_fusion_node ${OpenCV_LIBS})

add_executable(obstacle_lidar_node src/obstacle_lidar_node.cpp)
ament_target_dependencies(obstacle_lidar_node
  avg_msgs
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  pcl_conversions
)

target_link_libraries(obstacle_lidar_node
  pcl_common
  pcl_kdtree
  pcl_search
  pcl_filters
  pcl_segmentation
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# HH_260720 - Build the rear-camera AprilTag detector used only by AprilTag parking.
add_executable(apriltag_parking_detector_node src/apriltag_parking_detector_node.cpp)
ament_target_dependencies(apriltag_parking_detector_node
  # HH_260720 - Link the generated AprilTag parking pose and conversion helpers.
  avg_msgs
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  cv_bridge
)
target_include_directories(apriltag_parking_detector_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(apriltag_parking_detector_node
  apriltag::apriltag
  ${OpenCV_LIBS}
)

install(TARGETS
  obstacle_fusion_node
  obstacle_lidar_node
  apriltag_parking_detector_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # HH_260721 - Keep vendored YOLO sources outside CAMROD package lint ownership.
  set(AMENT_LINT_AUTO_FILE_EXCLUDE external)
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cppcheck
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_flake8
    ament_cmake_lint_cmake
    ament_cmake_pep257
    ament_cmake_uncrustify
    ament_cmake_xmllint
  )
  ament_lint_auto_find_test_dependencies()

  # HH_260721 - Run maintained-file lint with the same explicit scope as sibling packages.
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(src LANGUAGE c++)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake(CMakeLists.txt)
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(
    launch
    CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/test/ament_flake8.ini"
  )
  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257(launch --add-ignore D213)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify(src)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint(package.xml config/cyclonedds.xml)
endif()

ament_package()
