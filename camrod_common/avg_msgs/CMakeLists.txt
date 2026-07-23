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
find_package(unique_identifier_msgs REQUIRED)
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
  # HH_260720 - Generate concrete avg-owned contracts; standard ROS types are boundary-only.
  "msg/AvgHeader.msg"
  "msg/AvgBool.msg"
  "msg/AvgString.msg"
  "msg/AvgFloat32.msg"
  "msg/AvgColorRGBA.msg"
  "msg/AvgPoint.msg"
  "msg/AvgPoint32.msg"
  "msg/AvgQuaternion.msg"
  "msg/AvgVector3.msg"
  "msg/AvgPose.msg"
  "msg/AvgPoseStamped.msg"
  "msg/AvgPoseWithCovariance.msg"
  "msg/AvgPoseWithCovarianceStamped.msg"
  "msg/AvgTwist.msg"
  "msg/AvgTwistStamped.msg"
  "msg/AvgTwistWithCovariance.msg"
  "msg/AvgTwistWithCovarianceStamped.msg"
  "msg/AvgPolygon.msg"
  "msg/AvgPolygonStamped.msg"
  "msg/AvgTransform.msg"
  "msg/AvgTransformStamped.msg"
  "msg/AvgMapMetaData.msg"
  "msg/AvgOccupancyGrid.msg"
  "msg/AvgPath.msg"
  "msg/AvgOdometry.msg"
  "msg/AvgNavSatStatus.msg"
  "msg/AvgNavSatFix.msg"
  "msg/AvgImu.msg"
  "msg/AvgRange.msg"
  "msg/AvgPointField.msg"
  "msg/AvgPointCloud2.msg"
  "msg/AvgLaserScan.msg"
  "msg/AvgRegionOfInterest.msg"
  "msg/AvgImage.msg"
  "msg/AvgCameraInfo.msg"
  "msg/AvgGoalInfo.msg"
  "msg/AvgGoalStatus.msg"
  "msg/AvgGoalStatusArray.msg"
  "msg/AvgPose2D.msg"
  "msg/AvgBoundingBox2D.msg"
  "msg/AvgObjectHypothesis.msg"
  "msg/AvgObjectHypothesisWithPose.msg"
  "msg/AvgDetection2D.msg"
  "msg/AvgDetection2DArray.msg"
  "msg/AvgMarker.msg"
  "msg/AvgMarkerArray.msg"
  # HH_260721 - Use the platform-neutral service lifecycle contract.
  "msg/AvgServiceState.msg"
  # HH_260720 - Parking exposes one controller-ready tag pose without detection aliases.
  "msg/AvgAprilTagPose.msg" 
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
  "msg/AvgPlatformStatus.msg" # HH_260409: platform CAN/status bridge payload
  "msg/AvgLightCommand.msg" # 260708: headlight/turn-indicator command for light MCU bridge
  "msg/PlanningMissionKey.msg" # HH_260617: planning semantic mission key command/status
  "msg/PlanningRecallRequest.msg" # HH_260617: camping-site recall request
  "msg/PlanningScenario.msg" # HH_260617: planning scenario command/status
  "msg/PlanningState.msg" # HH_260617: planning state-machine status
  "msg/CampsiteOccupancy.msg" # HH_260723 - Perception-owned occupied campsite state
  # HH_260720 - Add typed route metadata used by planning, map, and platform control.
  "msg/RouteLaneletIds.msg"
  "msg/RouteTurnSegment.msg"
  "msg/RouteTurnSegmentArray.msg"
  "msg/UiDestinationCommand.msg" # HH_260617: UI destination command
  "msg/MotionOperation.msg" # HH_260720 - Typed start/cancel/return command for control nodes.
  "msg/AvgSensingMsgs.msg"
  "msg/AvgLocalizationMsgs.msg"
  "msg/AvgPlanningMsgs.msg"
  "msg/AvgMapMsgs.msg"
  "msg/AvgPlatformMsgs.msg"
  "msg/AvgPerceptionMsgs.msg"
  "msg/AvgBringupMsgs.msg"
  "msg/AvgSensorKitMsgs.msg"
  "msg/AvgSystemMsgs.msg"
  "msg/AudioRequest.msg"     # HH_260615: voice announcer — key-based WAV playback request
  "msg/VoiceState.msg"       # HH_260615: voice announcer — current playback state
)

set(srv_files
  "srv/RequestGoalByKey.srv"
  "srv/RequestMotionOperation.srv" # HH_260720 - Typed control-operation request service.
)

set(${PROJECT_NAME}_MEMBER_OF_GROUPS "rosidl_interface_packages")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  # HH_260720 - Parking control no longer requires a generated ROS action contract.
  DEPENDENCIES
    builtin_interfaces
    action_msgs
    nav2_msgs
    unique_identifier_msgs
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
# HH_260720 - Install the real utility headers at ROSIDL's exported include root;
# no forwarding or message-alias headers are kept in the source tree.
install(
  FILES
    include/avg_msgs/conversions.hpp
    include/avg_msgs/point_cloud2_iterator.hpp
  DESTINATION include/${PROJECT_NAME}/avg_msgs
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(nav2_msgs)
ament_export_dependencies(unique_identifier_msgs)
ament_package()
