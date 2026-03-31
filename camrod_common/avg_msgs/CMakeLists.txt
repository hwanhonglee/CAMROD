cmake_minimum_required(VERSION 3.8)
project(avg_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# NOTE:
# On some arm64 Docker builds, rcutils package discovery can resolve through an
# unexpected prefix and fail library probing. Resolve rcutils explicitly first
# so downstream rosidl_* packages inherit a stable rcutils_DIR.
find_package(rcutils REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
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

# NOTE:
# rosidl_generator_py still relies on legacy `FindPythonLibs` variables on
# some environments. Bridge modern Python3 discovery to legacy names so
# multi-arch Docker builds (especially arm64 under qemu) resolve consistently.
set(PYTHON_EXECUTABLE "${Python3_EXECUTABLE}" CACHE FILEPATH "Python executable" FORCE)
set(PYTHON_INCLUDE_DIR "${Python3_INCLUDE_DIRS}" CACHE PATH "Python include dir" FORCE)
set(PYTHON_INCLUDE_DIRS "${Python3_INCLUDE_DIRS}" CACHE STRING "Python include dirs" FORCE)
if(Python3_LIBRARIES)
  list(GET Python3_LIBRARIES 0 _python3_primary_lib)
  set(PYTHON_LIBRARY "${_python3_primary_lib}" CACHE FILEPATH "Python library" FORCE)
  set(PYTHON_LIBRARIES "${Python3_LIBRARIES}" CACHE STRING "Python libraries" FORCE)
endif()

set(msg_files
  "msg/ModuleState.msg"
  "msg/SystemStatus.msg"
  "msg/AvgLocalizationMode.msg"
  "msg/AvgLocalizationStatusStream.msg"
  "msg/AvgLocalizationStatus.msg"
  "msg/AvgSensorPose.msg"
  "msg/AvgRobotSpecifications.msg"
  "msg/AvgRobotInfo.msg"
  "msg/AvgSensingLidar.msg"
  "msg/AvgSensingCamera.msg"
  "msg/AvgSensingImu.msg"
  "msg/AvgSensingGnss.msg"
  "msg/AvgSensingRadar.msg"
  "msg/AvgGnssPose.msg" # HH_260326: custom GNSS pose message type
  "msg/AvgTrackingError.msg" # HH_260330: local/global path tracking error stream
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

set(srv_files
  "srv/RequestGoalByKey.srv"
)

set(${PROJECT_NAME}_MEMBER_OF_GROUPS "rosidl_interface_packages")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
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
