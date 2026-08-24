cmake_minimum_required(VERSION 3.16)
project(camrod_voice)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── ROS 2 dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(avg_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ── SDL2 / SDL2_mixer ─────────────────────────────────────────────────────────
find_package(SDL2 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2_MIXER REQUIRED IMPORTED_TARGET SDL2_mixer)

# ── Shared component library ──────────────────────────────────────────────────
add_library(${PROJECT_NAME}_component SHARED
  src/voice_announcer_node.cpp
  src/priority_queue.cpp
  src/audio_player.cpp
)

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_component
  SDL2::SDL2
  PkgConfig::SDL2_MIXER
)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  avg_msgs
  ament_index_cpp
)

rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "voice_announcer::VoiceAnnouncerNode"
  EXECUTABLE voice_announcer_node
)

# ── Standalone executable ─────────────────────────────────────────────────────
add_executable(voice_announcer_node_standalone src/main.cpp)
target_link_libraries(voice_announcer_node_standalone ${PROJECT_NAME}_component)

# ── Install C++ targets ───────────────────────────────────────────────────────
install(
  TARGETS ${PROJECT_NAME}_component voice_announcer_node_standalone
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

# ── Install Python scripts ────────────────────────────────────────────────────
install(PROGRAMS
  src/voice_event_adapter_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME voice_event_adapter_node
)
install(FILES
  src/voice_event_policy.py
  DESTINATION lib/${PROJECT_NAME}
)

# HH_260728 - Install the audited Bluetooth amplifier provisioning helper as
# an explicit administrator tool; it is never run automatically by bringup.
install(PROGRAMS
  setup_bt_audio.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

# ── Install launch / config / resource ───────────────────────────────────────
install(DIRECTORY launch/   DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/   DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY resource/ DESTINATION share/${PROJECT_NAME}/resource)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Isolate tests from user-site pytest plugins that may target a newer pytest.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  ament_add_pytest_test(
    test_voice_event_policy
    test/test_voice_event_policy.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  ament_add_pytest_test(
    test_audio_output_contract
    test/test_audio_output_contract.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
endif()

ament_package()
