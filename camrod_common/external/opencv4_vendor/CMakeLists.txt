cmake_minimum_required(VERSION 3.16)
project(opencv4_vendor)

find_package(ament_cmake REQUIRED)
include(ExternalProject)

# HH_260720 - Keep the OpenCV ABI used by cv_bridge independent from parking packages.
set(OPENCV_VERSION "4.5.4")
set(OPENCV_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/opt/opencv4_vendor")

ExternalProject_Add(opencv_ep
  PREFIX "${CMAKE_CURRENT_BINARY_DIR}/opencv_ep"
  URL "https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_VERSION}.tar.gz"
  CMAKE_ARGS
    "-DCMAKE_BUILD_TYPE=Release"
    "-DCMAKE_INSTALL_PREFIX=${OPENCV_INSTALL_DIR}"
    "-DBUILD_SHARED_LIBS=ON"
    "-DBUILD_LIST=core,imgproc,imgcodecs,videoio,calib3d,highgui"
    "-DBUILD_TESTS=OFF"
    "-DBUILD_PERF_TESTS=OFF"
    "-DBUILD_EXAMPLES=OFF"
    "-DBUILD_DOCS=OFF"
    "-DBUILD_opencv_python3=OFF"
    "-DBUILD_opencv_python2=OFF"
    "-DBUILD_opencv_java=OFF"
    "-DBUILD_opencv_js=OFF"
    "-DWITH_GTK=OFF"
    "-DWITH_QT=OFF"
    "-DWITH_V4L=ON"
    "-DWITH_FFMPEG=ON"
    "-DOPENCV_GENERATE_PKGCONFIG=OFF"
    "-DCMAKE_POSITION_INDEPENDENT_CODE=ON"
    "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}"
  BUILD_BYPRODUCTS
    "${OPENCV_INSTALL_DIR}/lib/libopencv_core.so"
    "${OPENCV_INSTALL_DIR}/lib/libopencv_imgproc.so"
    "${OPENCV_INSTALL_DIR}/lib/libopencv_imgcodecs.so"
    "${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so"
    "${OPENCV_INSTALL_DIR}/lib/libopencv_calib3d.so"
)

configure_file(
  "cmake/opencv4_vendor-extras.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/opencv4_vendor-extras.cmake"
  @ONLY
)

ament_environment_hooks("env-hooks/opencv4_vendor.sh.in")

ament_package(
  CONFIG_EXTRAS "${CMAKE_CURRENT_BINARY_DIR}/cmake/opencv4_vendor-extras.cmake"
)
