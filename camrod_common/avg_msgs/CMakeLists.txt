cmake_minimum_required(VERSION 3.8)
project(avg_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

set(msg_files
  "msg/ModuleHealth.msg"
  "msg/SystemDiagnostic.msg"
  "msg/AvgLocalizationMode.msg"
  "msg/AvgLocalizationDiagnostics.msg"
  "msg/AvgLocalizationStatus.msg"
  "msg/AvgSensorPose.msg"
  "msg/AvgRobotSpecifications.msg"
  "msg/AvgRobotInfo.msg"
  "msg/AvgSensingLidar.msg"
  "msg/AvgSensingCamera.msg"
  "msg/AvgSensingImu.msg"
  "msg/AvgSensingGnss.msg"
  "msg/AvgSensingRadar.msg"
  "msg/AvgSensingMsgs.msg"
  "msg/AvgLocalizationMsgs.msg"
  "msg/AvgPlanningMsgs.msg"
  "msg/AvgMapMsgs.msg"
  "msg/AvgPlatformMsgs.msg"
  "msg/AvgPerceptionMsgs.msg"
  "msg/AvgBringupMsgs.msg"
  "msg/AvgSensorKitMsgs.msg"
  "msg/AvgSystemMsgs.msg"
)

set(${PROJECT_NAME}_MEMBER_OF_GROUPS "rosidl_interface_packages")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
    builtin_interfaces
    action_msgs
    nav2_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs
    vision_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(nav2_msgs)
ament_package()
