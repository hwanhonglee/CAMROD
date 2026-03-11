cmake_minimum_required(VERSION 3.8)
project(vanjee_ground_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- ROS2 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# ---- PCL ----
find_package(PCL REQUIRED COMPONENTS common filters segmentation)

# PCL이 내부적으로 요구하는 imported targets 보정
# (환경에 따라 libusb::libusb, QHULL::QHULL 등이 없어서 터지는 경우 방어)
find_package(PkgConfig QUIET)

# ---- libusb::libusb 보정 ----
# 어떤 시스템은 'libusb-1.0'을 링크 문자열로만 제공하고 타겟이 없음
if(NOT TARGET libusb::libusb)
  if(PkgConfig_FOUND)
    pkg_check_modules(LIBUSB QUIET libusb-1.0)
    if(LIBUSB_FOUND)
      add_library(libusb::libusb INTERFACE IMPORTED)
      target_include_directories(libusb::libusb INTERFACE ${LIBUSB_INCLUDE_DIRS})
      target_link_libraries(libusb::libusb INTERFACE ${LIBUSB_LIBRARIES})
    endif()
  endif()
endif()

# ---- QHULL::QHULL 보정 ----
# ⚠️ 중요: "새로 만들지 말고", 존재하는 qhull 타겟이 있으면 alias만 만든다.
# 이미 QHULL::QHULL이 있는 환경도 많아 add_library(IMPORTED)로 만들면 충돌남.
if(NOT TARGET QHULL::QHULL)
  # Ubuntu/PCL 조합에 따라 Qhull 패키지의 타겟명이 다를 수 있음
  find_package(Qhull QUIET)

  # 1) Qhull::qhull / Qhull::qhull_r 가 있으면 alias 제공
  if(TARGET Qhull::qhull)
    add_library(QHULL::QHULL ALIAS Qhull::qhull)
  elseif(TARGET Qhull::qhull_r)
    add_library(QHULL::QHULL ALIAS Qhull::qhull_r)
  else()
    # 2) 그래도 없으면: 여기서는 "타겟을 만들지 않고" 그냥 넘어간다.
    #    (보통 libqhull-dev 설치 시 위 타겟이 잡힘)
    message(STATUS "QHULL::QHULL target not found via CMake. If link errors occur, install: sudo apt install libqhull-dev")
  endif()
endif()

# ---- ROS-PCL bridge ----
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# ---- Node ----
add_executable(ground_filter_node src/ground_filter_node.cpp)

target_include_directories(ground_filter_node PUBLIC
  ${PCL_INCLUDE_DIRS}
)

# PCL 컴파일 정의(가끔 필요)
add_definitions(${PCL_DEFINITIONS})

# 링크
target_link_libraries(ground_filter_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(ground_filter_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)

install(TARGETS ground_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

# (YAML 안 쓰면 config 디렉토리는 없어도 됨. 있어도 문제는 없음.)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
