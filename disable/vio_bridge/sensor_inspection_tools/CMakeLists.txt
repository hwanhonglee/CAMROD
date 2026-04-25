cmake_minimum_required(VERSION 3.16)
project(sensor_inspection_tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(SENSOR_TOOLS_USE_ZED "Build ZED sensor inspector" ON)
option(SENSOR_TOOLS_USE_ORBBEC "Build Orbbec sensor inspector" ON)

if(SENSOR_TOOLS_USE_ZED)
  set(ZED_SDK_ROOT "/usr/local/zed" CACHE PATH "ZED SDK root path")
  find_path(ZED_INCLUDE_DIR sl/Camera.hpp
    PATHS
      ${ZED_SDK_ROOT}/include
      /usr/local/zed/include
  )
  find_library(ZED_LIBRARY sl_zed
    PATHS
      ${ZED_SDK_ROOT}/lib
      /usr/local/zed/lib
  )
  find_library(ZED_AI_LIBRARY sl_ai
    PATHS
      ${ZED_SDK_ROOT}/lib
      /usr/local/zed/lib
  )

  find_path(CUDA_HEADER_DIR cuda.h
    PATHS
      /usr/include
      /usr/local/include
      /usr/local/cuda/include
      $ENV{CUDA_HOME}/include
      $ENV{CUDA_PATH}/include
  )

  if(ZED_INCLUDE_DIR AND ZED_LIBRARY AND CUDA_HEADER_DIR)
    add_executable(zed_sensor_inspector zed_sensor_inspector.cpp)
    target_include_directories(zed_sensor_inspector PRIVATE ${ZED_INCLUDE_DIR} ${CUDA_HEADER_DIR})
    if(ZED_AI_LIBRARY)
      target_link_libraries(zed_sensor_inspector PRIVATE ${ZED_LIBRARY} ${ZED_AI_LIBRARY})
    else()
      target_link_libraries(zed_sensor_inspector PRIVATE ${ZED_LIBRARY})
    endif()
    if(UNIX AND NOT APPLE)
      target_link_options(zed_sensor_inspector PRIVATE -Wl,--allow-shlib-undefined)
    endif()
    target_compile_definitions(zed_sensor_inspector PRIVATE SENSOR_TOOLS_USE_ZED=1)
  else()
    message(WARNING "ZED SDK or CUDA headers not found. zed_sensor_inspector will not be built.")
  endif()
endif()

if(SENSOR_TOOLS_USE_ORBBEC)
  set(ORBBEC_SDK_ROOT "/usr/local/OrbbecSDK" CACHE PATH "Orbbec SDK root path")

  find_package(OrbbecSDK QUIET CONFIG
    PATHS
      ${ORBBEC_SDK_ROOT}/lib
      ${ORBBEC_SDK_ROOT}
      /usr/local/lib
  )

  if(TARGET ob::OrbbecSDK)
    add_executable(orbbec_sensor_inspector orbbec_sensor_inspector.cpp)
    target_link_libraries(orbbec_sensor_inspector PRIVATE ob::OrbbecSDK)
    target_compile_definitions(orbbec_sensor_inspector PRIVATE SENSOR_TOOLS_USE_ORBBEC=1)
    add_executable(orbbec_force_ip_tool orbbec_force_ip_tool.cpp)
    target_link_libraries(orbbec_force_ip_tool PRIVATE ob::OrbbecSDK)
    target_compile_definitions(orbbec_force_ip_tool PRIVATE SENSOR_TOOLS_USE_ORBBEC=1)
  else()
    find_path(ORBBEC_INCLUDE_DIR libobsensor/ObSensor.hpp
      PATHS
        ${ORBBEC_SDK_ROOT}/include
        /usr/include
        /usr/local/include
    )

    find_library(ORBBEC_LIBRARY
      NAMES OrbbecSDK obsensor
      PATHS
        ${ORBBEC_SDK_ROOT}/lib
        ${ORBBEC_SDK_ROOT}/lib64
        /usr/lib
        /usr/lib/x86_64-linux-gnu
        /usr/local/lib
    )

    if(ORBBEC_INCLUDE_DIR AND ORBBEC_LIBRARY)
      add_executable(orbbec_sensor_inspector orbbec_sensor_inspector.cpp)
      target_include_directories(orbbec_sensor_inspector PRIVATE ${ORBBEC_INCLUDE_DIR})
      target_link_libraries(orbbec_sensor_inspector PRIVATE ${ORBBEC_LIBRARY})
      target_compile_definitions(orbbec_sensor_inspector PRIVATE SENSOR_TOOLS_USE_ORBBEC=1)
      add_executable(orbbec_force_ip_tool orbbec_force_ip_tool.cpp)
      target_include_directories(orbbec_force_ip_tool PRIVATE ${ORBBEC_INCLUDE_DIR})
      target_link_libraries(orbbec_force_ip_tool PRIVATE ${ORBBEC_LIBRARY})
      target_compile_definitions(orbbec_force_ip_tool PRIVATE SENSOR_TOOLS_USE_ORBBEC=1)
    else()
      message(WARNING "Orbbec SDK not found. orbbec_sensor_inspector will not be built.")
    endif()
  endif()
endif()
