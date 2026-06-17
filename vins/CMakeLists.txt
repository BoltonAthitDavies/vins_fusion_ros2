cmake_minimum_required(VERSION 3.5)
project(vins)
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wextra -Wpedantic)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Glog)
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
find_package(Eigen3)

# CPU-only feature tracking, matching the upstream/github build. The cv::cuda
# (GPU) paths in the feature tracker are guarded by #ifdef GPU_MODE; with
# GPU_MODE undefined they are excluded and VINS runs cv::calcOpticalFlowPyrLK on
# the CPU. Set VINS_HAS_CUDA TRUE (and rebuild) to re-enable GPU.
set(VINS_HAS_CUDA FALSE)
message(STATUS "vins: GPU_MODE OFF -> CPU-only feature tracking")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIR}
    ${GLOG_INCLUDE_DIRS}

)

file(GLOB_RECURSE SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")
file(GLOB_RECURSE HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")

add_library(vins_lib ${SRC_FILES} ${HEADER_FILES})
target_link_libraries(vins_lib  ${OpenCV_LIBS} ${CERES_LIBRARIES} ${GLOG_LIBRARIES})

# PUBLIC so the definition also reaches the node target that links vins_lib -- the
# FeatureTracker header changes under GPU_MODE, so both must compile it consistently.
if(VINS_HAS_CUDA)
  target_compile_definitions(vins_lib PUBLIC GPU_MODE)
endif()

if(OpenMP_CXX_FOUND)
   target_link_libraries(vins_lib OpenMP::OpenMP_CXX)
endif()

target_include_directories(vins_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CERES_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${CERES_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${OpenCV_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${GLOG_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>
)

# Install nodes
install(
  TARGETS vins_lib
  DESTINATION lib/${PROJECT_NAME}
)
