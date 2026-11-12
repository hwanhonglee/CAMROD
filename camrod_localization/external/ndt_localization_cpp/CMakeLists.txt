cmake_minimum_required(VERSION 3.14)
project(ndt_localization_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ublox_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(autoware_internal_debug_msgs REQUIRED)

add_library(${PROJECT_NAME}_geo SHARED src/geo.cpp)
target_include_directories(${PROJECT_NAME}_geo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_geo yaml-cpp)

add_library(points_preprocessor_component SHARED src/points_preprocessor.cpp)
ament_target_dependencies(points_preprocessor_component rclcpp rclcpp_components sensor_msgs geometry_msgs tf2_ros)
rclcpp_components_register_node(points_preprocessor_component
  PLUGIN "ndt_localization_cpp::PointsPreprocessor" EXECUTABLE points_preprocessor)

add_library(initial_pose_bridge_component SHARED src/initial_pose_bridge.cpp)
target_link_libraries(initial_pose_bridge_component ${PROJECT_NAME}_geo yaml-cpp)
ament_target_dependencies(initial_pose_bridge_component rclcpp rclcpp_components sensor_msgs geometry_msgs
  nav_msgs std_msgs std_srvs ublox_msgs)
rclcpp_components_register_node(initial_pose_bridge_component
  PLUGIN "ndt_localization_cpp::InitialPoseBridge" EXECUTABLE initial_pose_bridge)

add_library(ndt_ekf_gate_component SHARED src/ndt_ekf_gate.cpp)
ament_target_dependencies(ndt_ekf_gate_component rclcpp rclcpp_components geometry_msgs autoware_internal_debug_msgs)
rclcpp_components_register_node(ndt_ekf_gate_component
  PLUGIN "ndt_localization_cpp::NdtEkfGate" EXECUTABLE ndt_ekf_gate)

install(TARGETS ${PROJECT_NAME}_geo points_preprocessor_component initial_pose_bridge_component ndt_ekf_gate_component
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/ndt_viz_flatten.py DESTINATION lib/${PROJECT_NAME})

ament_package()
