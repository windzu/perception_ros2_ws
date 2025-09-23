cmake_minimum_required(VERSION 3.17)
project(autoware_tensorrt_common)

# 自定义 FindCUDNN.cmake FindTENSORRT.cmake 模块路径
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(CUDA)
find_package(CUDAToolkit)
find_package(CUDNN REQUIRED)
find_package(TENSORRT REQUIRED)

# debug info
message(STATUS "== CMake probe ==")
message(STATUS "CUDAToolkit_FOUND = ${CUDAToolkit_FOUND}")
message(STATUS "CUDNN_FOUND       = ${CUDNN_FOUND}")
message(STATUS "CUDNN_LIBRARY     = ${CUDNN_LIBRARY}")
message(STATUS "TENSORRT_FOUND    = ${TENSORRT_FOUND}")
message(STATUS "TENSORRT_VERSION_STRING  = ${TENSORRT_VERSION_STRING}")
message(STATUS "TENSORRT_INCLUDES = ${TENSORRT_INCLUDE_DIRS}")
message(STATUS "TENSORRT_LIBS     = ${TENSORRT_LIBRARIES}")

if(NOT (CUDAToolkit_FOUND AND CUDNN_FOUND AND TENSORRT_FOUND))
  message(WARNING "cuda, cudnn, tensorrt libraries are not found")
  return()
endif()

if(TENSORRT_VERSION_STRING VERSION_LESS 8.5)
  message(WARNING "Unsupported version TensorRT ${TENSORRT_VERSION} detected. This package requires TensorRT 8.5 or later.")
  return()
endif()

add_library(${PROJECT_NAME} SHARED
  src/tensorrt_common.cpp
  src/profiler.cpp
)

target_link_libraries(${PROJECT_NAME}
  # ROS 依赖
  rclcpp::rclcpp

  # NVIDIA 依赖
  CUDA::cudart
  TENSORRT::tensorrt  # <-- 使用 IMPORTED TARGET
  CUDNN::cudnn        # <-- 使用 IMPORTED TARGET

  stdc++fs
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${TENSORRT_INCLUDE_DIRS}
)

set_target_properties(${PROJECT_NAME}
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

# TODO(amadeuszsz): Remove -Wno-deprecated-declarations once removing implicit quantization
target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall -Wextra -Wpedantic -Werror -Wno-deprecated-declarations
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(
  "CUDA"
  "CUDAToolkit"
  "CUDNN"
  "rclcpp"
  "TENSORRT"
)

ament_package()
