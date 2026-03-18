cmake_minimum_required(VERSION 3.8)
project(camrod_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(avg_msgs REQUIRED)

install(
  PROGRAMS
    scripts/system_checker_node.py
    scripts/module_checker_node.py
    scripts/system_diagnostic_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
