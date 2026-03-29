cmake_minimum_required(VERSION 3.8)
project(robot_hw_gpu_checker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(robot_diagnostics_base REQUIRED)

set(DEPS rclcpp diagnostic_msgs diagnostic_updater robot_diagnostics_base)

# ── hw_checker_node ────────────────────────────────────────────────────────
add_executable(hw_checker_node src/hw_checker_node.cpp)
ament_target_dependencies(hw_checker_node ${DEPS})
# filesystem 링크 (GCC < 9 호환)
target_link_libraries(hw_checker_node stdc++fs)

# ── gpu_checker_node ───────────────────────────────────────────────────────
add_executable(gpu_checker_node src/gpu_checker_node.cpp)
ament_target_dependencies(gpu_checker_node ${DEPS})

install(TARGETS hw_checker_node gpu_checker_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
