cmake_minimum_required(VERSION 3.5)
project(vins_fusion_ros2)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wextra -Wpedantic)

set(CMAKE_BUILD_TYPE Release)

# Build against the non-CUDA OpenCV 4.10 (CPU twin in ~/local_ocv), the same
# OpenCV rtabmap is built against. This keeps the whole stack on one consistent
# non-CUDA OpenCV (no cv::cuda modules linked anywhere). GPU is also off at the
# code level (vins/CMakeLists sets VINS_HAS_CUDA FALSE -> no GPU_MODE).
set(OpenCV_DIR $ENV{HOME}/local_ocv/lib/cmake/opencv4 CACHE PATH "" FORCE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(Ceres REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(Threads REQUIRED)

# RTAB-Map mode (carla_rtab_direct) is NOT built in this package. apt rtabmap is
# built against system OpenCV 4.5.4, while this workspace's GPU path uses local
# CUDA OpenCV 4.10 (~/local), and one package configure has a single OpenCV_DIR.
# Instead it is built standalone in rtab_tool/ against a CPU-twin OpenCV 4.10
# (~/local_ocv) + source-built rtabmap/GTSAM in ~/local. See rtab_tool/CMakeLists.txt.
option(WITH_RTABMAP "Deprecated: build carla_rtab_direct in-package (use rtab_tool/ instead)" OFF)
if(WITH_RTABMAP)
  find_package(RTABMap REQUIRED COMPONENTS core)
endif()


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_subdirectory(vins)
add_executable(vins_fusion_ros2_node src/vins_estimator.cpp src/vins_estimator_node.cpp src/visualization.cpp)
ament_target_dependencies(vins_fusion_ros2_node rclcpp rcpputils message_filters std_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge image_transport)
target_link_libraries(vins_fusion_ros2_node vins_lib)

# Offline direct-read driver (rosbag2_cpp SequentialReader -> Estimator [+ global_fusion])
set(GF ${CMAKE_CURRENT_SOURCE_DIR}/../global_fusion)
add_executable(vins_bag_reader
  src/vins_bag_reader.cpp
  ${GF}/src/globalOpt.cpp
  ${GF}/ThirdParty/GeographicLib/src/Geocentric.cpp
  ${GF}/ThirdParty/GeographicLib/src/LocalCartesian.cpp
  ${GF}/ThirdParty/GeographicLib/src/Math.cpp)
target_include_directories(vins_bag_reader PRIVATE
  ${GF}/src ${GF}/ThirdParty/GeographicLib/include ${CERES_INCLUDE_DIRS})
ament_target_dependencies(vins_bag_reader
  rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs cv_bridge rosbag2_cpp rosbag2_storage)
target_link_libraries(vins_bag_reader vins_lib ${CERES_LIBRARIES})

# CARLA bag -> VINS reader (refactored API). Loads a rosbag2 directly, feeds the
# Estimator, writes <OUTPUT_FOLDER>/vio.txt, republishes images/gnss + /vins_estimator/odometry.
add_executable(carla_bag_test src/carla_bag_test.cpp)
ament_target_dependencies(carla_bag_test
  rclcpp sensor_msgs nav_msgs geometry_msgs cv_bridge rosbag2_cpp rosbag2_storage)
target_link_libraries(carla_bag_test vins_lib)

# CARLA bag -> VINS -> RTAB-Map (direct, no ROS between them). Same VINS API as
# carla_bag_test, plus RTAB-Map core for loop closure + GPS fusion. Opt-in.
if(WITH_RTABMAP)
  add_executable(carla_rtab_direct src/carla_rtab_direct.cpp)
  ament_target_dependencies(carla_rtab_direct
    rclcpp sensor_msgs nav_msgs geometry_msgs cv_bridge rosbag2_cpp rosbag2_storage)
  target_include_directories(carla_rtab_direct SYSTEM PRIVATE ${RTABMap_INCLUDE_DIRS})
  target_link_libraries(carla_rtab_direct vins_lib rtabmap::core)
  install(TARGETS carla_rtab_direct DESTINATION lib/${PROJECT_NAME})
endif()

# Native LibCarla driver: connects live to a CARLA server (no Python bridge / bag),
# feeds VINS directly, publishes /vins_estimator/odometry + /path.
set(CARLA_DIR /home/fibo3/carla/PythonAPI/carla/dependencies)
add_executable(carla_vins_node
  src/carla_cpp/carla_vins_node.cpp
  src/carla_cpp/carla_world.cpp
  src/carla_cpp/vins_bridge.cpp
  src/carla_cpp/vins_publisher.cpp
  src/carla_cpp/keyboard_control.cpp
  src/carla_cpp/ros_output.cpp)
target_include_directories(carla_vins_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/carla_cpp)
# CARLA headers as SYSTEM includes to suppress their warnings under -Wextra/-Wpedantic.
target_include_directories(carla_vins_node SYSTEM PRIVATE
  ${CARLA_DIR}/include
  ${CARLA_DIR}/include/system)
target_compile_definitions(carla_vins_node PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
ament_target_dependencies(carla_vins_node
  rclcpp rcpputils std_msgs sensor_msgs rosgraph_msgs geometry_msgs
  visualization_msgs nav_msgs tf2 tf2_ros cv_bridge image_transport)
target_link_libraries(carla_vins_node
  vins_lib
  -Wl,--start-group
    ${CARLA_DIR}/lib/libcarla_client.a
    ${CARLA_DIR}/lib/librpc.a
    ${CARLA_DIR}/lib/libboost_filesystem.a
    ${CARLA_DIR}/lib/libboost_system.a
    ${CARLA_DIR}/lib/libboost_program_options.a
    ${CARLA_DIR}/lib/libboost_atomic.a
  -Wl,--end-group
  ${CARLA_DIR}/lib/libRecast.a
  ${CARLA_DIR}/lib/libDetour.a
  ${CARLA_DIR}/lib/libDetourCrowd.a
  ${CARLA_DIR}/lib/libDetourTileCache.a
  ${CARLA_DIR}/lib/libDebugUtils.a
  ${CARLA_DIR}/lib/libosm2odr.a
  ${CARLA_DIR}/lib/libproj.a
  ${CARLA_DIR}/lib/libxerces-c.a
  ${CARLA_DIR}/lib/libsqlite3.a
  png tiff jpeg
  Threads::Threads)

# Multi-variant live driver: 3 estimators + 2 in-process global_fusion off one
# CARLA stream -> /vins_{mono,stereo,stereo_imu,stereo_gps,stereo_imu_gps}/odometry.
add_executable(carla_vins_multi_node
  src/carla_cpp/carla_vins_multi_node.cpp
  src/carla_cpp/carla_world.cpp
  src/carla_cpp/vins_bridge.cpp
  src/carla_cpp/multi_vins.cpp
  src/carla_cpp/keyboard_control.cpp
  src/carla_cpp/ros_output.cpp
  ${GF}/src/globalOpt.cpp
  ${GF}/ThirdParty/GeographicLib/src/Geocentric.cpp
  ${GF}/ThirdParty/GeographicLib/src/LocalCartesian.cpp
  ${GF}/ThirdParty/GeographicLib/src/Math.cpp)
target_include_directories(carla_vins_multi_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/carla_cpp
  ${GF}/src ${GF}/ThirdParty/GeographicLib/include ${CERES_INCLUDE_DIRS})
target_include_directories(carla_vins_multi_node SYSTEM PRIVATE
  ${CARLA_DIR}/include ${CARLA_DIR}/include/system)
target_compile_definitions(carla_vins_multi_node PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
ament_target_dependencies(carla_vins_multi_node
  rclcpp rcpputils std_msgs sensor_msgs rosgraph_msgs geometry_msgs
  visualization_msgs nav_msgs tf2 tf2_ros cv_bridge image_transport)
target_link_libraries(carla_vins_multi_node
  vins_lib ${CERES_LIBRARIES}
  -Wl,--start-group
    ${CARLA_DIR}/lib/libcarla_client.a
    ${CARLA_DIR}/lib/librpc.a
    ${CARLA_DIR}/lib/libboost_filesystem.a
    ${CARLA_DIR}/lib/libboost_system.a
    ${CARLA_DIR}/lib/libboost_program_options.a
    ${CARLA_DIR}/lib/libboost_atomic.a
  -Wl,--end-group
  ${CARLA_DIR}/lib/libRecast.a
  ${CARLA_DIR}/lib/libDetour.a
  ${CARLA_DIR}/lib/libDetourCrowd.a
  ${CARLA_DIR}/lib/libDetourTileCache.a
  ${CARLA_DIR}/lib/libDebugUtils.a
  ${CARLA_DIR}/lib/libosm2odr.a
  ${CARLA_DIR}/lib/libproj.a
  ${CARLA_DIR}/lib/libxerces-c.a
  ${CARLA_DIR}/lib/libsqlite3.a
  png tiff jpeg
  Threads::Threads)

# Install launch files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(
  TARGETS vins_fusion_ros2_node vins_bag_reader carla_bag_test carla_vins_node carla_vins_multi_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
