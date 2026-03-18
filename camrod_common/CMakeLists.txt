cmake_minimum_required(VERSION 3.8)
project(camrod_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(avg_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_python_install_package(${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(avg_msgs)
ament_export_dependencies(nav2_msgs)

ament_package()
