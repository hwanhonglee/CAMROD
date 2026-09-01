cmake_minimum_required(VERSION 3.8)
project(yolov9mit)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

option(YOLOV9_MIT_USE_TENSORRT  "Use TensorRT"  ON)

# Optional TensorRT installation prefix.  When it is not set, prefer the
# distribution-provided headers and libraries before a TensorRT copy bundled
# with the CUDA toolkit.  The preference matters on development machines which
# have both (for example TensorRT 8 from apt and TensorRT 10 under
# /usr/local/cuda): compiling against one major and loading the other is an ABI
# mismatch which can crash in deserializeCudaEngine().
set(TENSORRT_ROOT "$ENV{TENSORRT_ROOT}" CACHE PATH
  "TensorRT installation prefix (must contain a coherent include/lib set)")

set(ENABLE_TENSORRT OFF)

if(YOLOV9_MIT_USE_TENSORRT)
  find_package(CUDA REQUIRED)

  set(_tensorrt_include_hints)
  if(TENSORRT_ROOT)
    list(APPEND _tensorrt_include_hints
      "${TENSORRT_ROOT}/include/${CMAKE_LIBRARY_ARCHITECTURE}"
      "${TENSORRT_ROOT}/include"
      "${TENSORRT_ROOT}/targets/${CMAKE_SYSTEM_PROCESSOR}-linux/include")
  else()
    # Debian/Ubuntu multi-arch packages are preferred.  CUDA-bundled TensorRT
    # is still a supported fallback when no distribution package is installed.
    list(APPEND _tensorrt_include_hints
      "/usr/include/${CMAKE_LIBRARY_ARCHITECTURE}"
      "/usr/include"
      "/usr/local/include/${CMAKE_LIBRARY_ARCHITECTURE}"
      "/usr/local/include")
    if(CUDA_TOOLKIT_ROOT_DIR)
      list(APPEND _tensorrt_include_hints
        "${CUDA_TOOLKIT_ROOT_DIR}/targets/${CMAKE_SYSTEM_PROCESSOR}-linux/include"
        "${CUDA_TOOLKIT_ROOT_DIR}/include")
    endif()
  endif()

  find_path(TENSORRT_INCLUDE_DIR
    NAMES NvInferVersion.h
    PATHS ${_tensorrt_include_hints}
    NO_DEFAULT_PATH)
  if(NOT TENSORRT_INCLUDE_DIR AND NOT TENSORRT_ROOT)
    find_path(TENSORRT_INCLUDE_DIR NAMES NvInferVersion.h)
  endif()
  if(NOT TENSORRT_INCLUDE_DIR OR
    NOT EXISTS "${TENSORRT_INCLUDE_DIR}/NvInfer.h")
    message(FATAL_ERROR
      "TensorRT headers were not found as a complete set. Set TENSORRT_ROOT "
      "to the installation prefix containing include/NvInfer.h and "
      "include/NvInferVersion.h.")
  endif()
  foreach(_required_header NvInferPlugin.h NvOnnxParser.h)
    if(NOT EXISTS "${TENSORRT_INCLUDE_DIR}/${_required_header}")
      message(FATAL_ERROR
        "TensorRT header '${_required_header}' is missing from "
        "${TENSORRT_INCLUDE_DIR}. Install the matching TensorRT plugin and "
        "ONNX parser development packages.")
    endif()
  endforeach()

  file(STRINGS "${TENSORRT_INCLUDE_DIR}/NvInferVersion.h"
    _tensorrt_major_line REGEX "^#define NV_TENSORRT_MAJOR[ \t]+[0-9]+")
  file(STRINGS "${TENSORRT_INCLUDE_DIR}/NvInferVersion.h"
    _tensorrt_minor_line REGEX "^#define NV_TENSORRT_MINOR[ \t]+[0-9]+")
  file(STRINGS "${TENSORRT_INCLUDE_DIR}/NvInferVersion.h"
    _tensorrt_patch_line REGEX "^#define NV_TENSORRT_PATCH[ \t]+[0-9]+")
  file(STRINGS "${TENSORRT_INCLUDE_DIR}/NvInferVersion.h"
    _tensorrt_build_line REGEX "^#define NV_TENSORRT_BUILD[ \t]+[0-9]+")
  foreach(_component MAJOR MINOR PATCH BUILD)
    string(TOLOWER "${_component}" _component_lower)
    string(REGEX REPLACE ".*NV_TENSORRT_${_component}[ \t]+([0-9]+).*" "\\1"
      TENSORRT_VERSION_${_component} "${_tensorrt_${_component_lower}_line}")
    if(NOT TENSORRT_VERSION_${_component} MATCHES "^[0-9]+$")
      message(FATAL_ERROR
        "Could not parse NV_TENSORRT_${_component} from "
        "${TENSORRT_INCLUDE_DIR}/NvInferVersion.h")
    endif()
  endforeach()

  set(_tensorrt_library_hints)
  if(TENSORRT_ROOT)
    list(APPEND _tensorrt_library_hints
      "${TENSORRT_ROOT}/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
      "${TENSORRT_ROOT}/lib"
      "${TENSORRT_ROOT}/lib64"
      "${TENSORRT_ROOT}/targets/${CMAKE_SYSTEM_PROCESSOR}-linux/lib")
  elseif(TENSORRT_INCLUDE_DIR MATCHES "^/usr/include")
    list(APPEND _tensorrt_library_hints
      "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
      "/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
      "/usr/lib"
      "/lib")
  elseif(TENSORRT_INCLUDE_DIR MATCHES "^/usr/local")
    list(APPEND _tensorrt_library_hints
      "/usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
      "/usr/local/lib"
      "/usr/local/lib64")
  endif()
  if(CUDA_TOOLKIT_ROOT_DIR)
    list(APPEND _tensorrt_library_hints
      "${CUDA_TOOLKIT_ROOT_DIR}/targets/${CMAKE_SYSTEM_PROCESSOR}-linux/lib"
      "${CUDA_TOOLKIT_ROOT_DIR}/lib64")
  endif()

  foreach(_library_spec
      "NVINFER;nvinfer"
      "NVINFERPLUGIN;nvinfer_plugin"
      "NVONNXPARSER;nvonnxparser")
    list(GET _library_spec 0 _variable)
    list(GET _library_spec 1 _library_name)
    find_library(TENSORRT_${_variable}_LIBRARY
      NAMES ${_library_name}
      PATHS ${_tensorrt_library_hints}
      NO_DEFAULT_PATH)
    if(NOT TENSORRT_${_variable}_LIBRARY AND NOT TENSORRT_ROOT)
      find_library(TENSORRT_${_variable}_LIBRARY NAMES ${_library_name})
    endif()
    if(NOT TENSORRT_${_variable}_LIBRARY)
      message(FATAL_ERROR
        "TensorRT library '${_library_name}' was not found for headers in "
        "${TENSORRT_INCLUDE_DIR}. Set TENSORRT_ROOT to a complete TensorRT "
        "installation prefix.")
    endif()

    get_filename_component(_library_realpath
      "${TENSORRT_${_variable}_LIBRARY}" REALPATH)
    get_filename_component(_library_filename "${_library_realpath}" NAME)
    set(_library_major "")
    if(_library_filename MATCHES
      "^lib${_library_name}\\.so\\.([0-9]+)(\\..*)?$")
      set(_library_major "${CMAKE_MATCH_1}")
    endif()
    if(NOT _library_major)
      message(FATAL_ERROR
        "Could not determine the TensorRT ABI major from '${_library_realpath}'. "
        "Expected a versioned lib${_library_name}.so.<major> shared library.")
    endif()
    if(NOT _library_major STREQUAL TENSORRT_VERSION_MAJOR)
      message(FATAL_ERROR
        "TensorRT ABI mismatch: headers are ${TENSORRT_VERSION_MAJOR}."
        "${TENSORRT_VERSION_MINOR}.${TENSORRT_VERSION_PATCH} at "
        "${TENSORRT_INCLUDE_DIR}, but ${_library_name} resolves to major "
        "${_library_major} at ${_library_realpath}. Set TENSORRT_ROOT (or the "
        "TENSORRT_* cache entries) so every TensorRT component uses one major.")
    endif()
  endforeach()

  message(STATUS
    "TensorRT ${TENSORRT_VERSION_MAJOR}.${TENSORRT_VERSION_MINOR}."
    "${TENSORRT_VERSION_PATCH}.${TENSORRT_VERSION_BUILD} headers: "
    "${TENSORRT_INCLUDE_DIR}")
  message(STATUS "TensorRT nvinfer: ${TENSORRT_NVINFER_LIBRARY}")
  message(STATUS "TensorRT nvinfer_plugin: ${TENSORRT_NVINFERPLUGIN_LIBRARY}")
  message(STATUS "TensorRT nvonnxparser: ${TENSORRT_NVONNXPARSER_LIBRARY}")

  # GCC removes an explicitly re-added multi-arch system directory (such as
  # /usr/include/x86_64-linux-gnu) from -I/-iquote search paths.  FindCUDA then
  # places /usr/local/cuda/include first, which can silently select a different
  # NvInfer.h.  This generated forwarding header pins NvInfer.h to the exact
  # version-checked path above.  It is exported so downstream consumers such as
  # yolov9mit_ros compile the public TensorRT class declaration against the same
  # ABI as libyolov9mit.
  set(TENSORRT_ABI_INCLUDE_DIR
    "${CMAKE_CURRENT_BINARY_DIR}/tensorrt_abi_include")
  file(MAKE_DIRECTORY "${TENSORRT_ABI_INCLUDE_DIR}")
  foreach(_forward_header NvInfer.h NvInferPlugin.h NvOnnxParser.h)
    file(WRITE "${TENSORRT_ABI_INCLUDE_DIR}/${_forward_header}"
      "#pragma once\n"
      "#include \"${TENSORRT_INCLUDE_DIR}/${_forward_header}\"\n"
      "#if NV_TENSORRT_MAJOR != ${TENSORRT_VERSION_MAJOR}\n"
      "#error \"TensorRT header major changed after CMake configuration\"\n"
      "#endif\n")
  endforeach()

  set(ENABLE_TENSORRT ON)
  set(TARGET_SRC src/yolov9mit_tensorrt.cpp)
  set(TARGET_LIBS
    ${TENSORRT_NVINFER_LIBRARY}
    ${TENSORRT_NVINFERPLUGIN_LIBRARY}
    ${TENSORRT_NVONNXPARSER_LIBRARY}
    ${CUDA_LIBRARIES})
  set(TARGET_DPENDENCIES CUDA)
endif()

configure_file(
  "${PROJECT_SOURCE_DIR}/include/yolov9mit/config.h.in"
  "${PROJECT_SOURCE_DIR}/include/yolov9mit/config.h"
)

ament_auto_add_library(yolov9mit SHARED ${TARGET_SRC})
ament_target_dependencies(yolov9mit ${TARGET_DPENDENCIES})
ament_export_dependencies(${TARGET_DPENDENCIES})

if(YOLOV9_MIT_USE_TENSORRT)
  target_include_directories(yolov9mit BEFORE PUBLIC
    "$<BUILD_INTERFACE:${TENSORRT_ABI_INCLUDE_DIR}>"
    "$<INSTALL_INTERFACE:include/yolov9mit/tensorrt_abi>")
  target_link_libraries(yolov9mit ${TARGET_LIBS})
  install(FILES
    "${TENSORRT_ABI_INCLUDE_DIR}/NvInfer.h"
    "${TENSORRT_ABI_INCLUDE_DIR}/NvInferPlugin.h"
    "${TENSORRT_ABI_INCLUDE_DIR}/NvOnnxParser.h"
    DESTINATION include/yolov9mit/tensorrt_abi)
  ament_export_include_directories(include/yolov9mit/tensorrt_abi)

  ament_auto_add_executable(yolov9mit_build_engine
    src/yolov9mit_build_engine.cpp)
  ament_target_dependencies(yolov9mit_build_engine ${TARGET_DPENDENCIES})
  target_include_directories(yolov9mit_build_engine BEFORE PRIVATE
    "${TENSORRT_ABI_INCLUDE_DIR}")
  target_link_libraries(yolov9mit_build_engine ${TARGET_LIBS})
endif()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
