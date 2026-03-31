cmake_minimum_required(VERSION 3.8)
project(robot_diagnostics_agg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
  find_package(yaml REQUIRED)
  set(YAML_LIB yaml)
else()
  set(YAML_LIB yaml-cpp)
endif()

set(DEPS rclcpp diagnostic_msgs)

# ── aggregator_node ────────────────────────────────────────────────────────
add_executable(aggregator_node src/aggregator_node.cpp)
ament_target_dependencies(aggregator_node ${DEPS})
target_link_libraries(aggregator_node ${YAML_LIB})

install(TARGETS aggregator_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
