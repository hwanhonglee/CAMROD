cmake_minimum_required(VERSION 3.16)
project(yaml_cpp_vendor)

find_package(ament_cmake REQUIRED)
include(ExternalProject)

set(YAML_CPP_VERSION "0.8.0")
# Install directly to the final workspace prefix so cmake config paths are correct
set(YAML_CPP_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/opt/yaml_cpp_vendor")

ExternalProject_Add(yaml_cpp_ep
  PREFIX "${CMAKE_CURRENT_BINARY_DIR}/yaml_cpp_ep"
  URL "https://github.com/jbeder/yaml-cpp/archive/refs/tags/${YAML_CPP_VERSION}.tar.gz"
  CMAKE_ARGS
    "-DCMAKE_BUILD_TYPE=Release"
    "-DCMAKE_INSTALL_PREFIX=${YAML_CPP_INSTALL_DIR}"
    "-DYAML_CPP_BUILD_TESTS=OFF"
    "-DYAML_CPP_BUILD_TOOLS=OFF"
    "-DBUILD_SHARED_LIBS=ON"
    "-DCMAKE_POSITION_INDEPENDENT_CODE=ON"
    "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}"
  BUILD_BYPRODUCTS
    "${YAML_CPP_INSTALL_DIR}/lib/libyaml-cpp.so"
)

configure_file(
  "cmake/yaml_cpp_vendor-extras.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/yaml_cpp_vendor-extras.cmake"
  @ONLY
)

ament_environment_hooks("env-hooks/yaml_cpp_vendor.sh.in")

ament_package(
  CONFIG_EXTRAS "${CMAKE_CURRENT_BINARY_DIR}/cmake/yaml_cpp_vendor-extras.cmake"
)
