cmake_minimum_required(VERSION 3.8)
project(camrod_runtime)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

function(camrod_add_scoped_container target)
  add_executable(${target} src/scoped_component_container.cpp)
  target_compile_features(${target} PUBLIC cxx_std_17)
  ament_target_dependencies(${target} rclcpp rclcpp_components)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

camrod_add_scoped_container(scoped_component_container)
camrod_add_scoped_container(scoped_component_container_mt)
target_compile_definitions(
  scoped_component_container_mt
  PRIVATE CAMROD_RUNTIME_MULTITHREADED_EXECUTOR)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_scoped_component_container_contract
    test/test_scoped_component_container_contract.py
  )
endif()

ament_package()
