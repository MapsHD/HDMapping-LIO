cmake_minimum_required(VERSION 3.16)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS ON CACHE BOOL "" FORCE)
project(hdmapping_lio)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Threads REQUIRED)
find_package(TBB REQUIRED)

set(HDMAPPING_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/HDMapping)
set(HDMAPPING_3RDPARTY ${HDMAPPING_DIR}/3rdparty)
set(HDMAPPING_EXTERNAL ${HDMAPPING_DIR}/external)

# ===========================================================================
# HDMapping bundled 3rdparty
# ===========================================================================

set(EIGEN3_INCLUDE_DIR ${HDMAPPING_3RDPARTY}/eigen)

add_subdirectory(${HDMAPPING_3RDPARTY}/LASzip)

add_subdirectory(${HDMAPPING_3RDPARTY}/Fusion/Fusion)

set(SPDLOG_BUILD_PIC ON CACHE BOOL "" FORCE)
add_subdirectory(${HDMAPPING_3RDPARTY}/spdlog)

add_subdirectory(${HDMAPPING_3RDPARTY}/unordered_dense)

add_subdirectory(${HDMAPPING_3RDPARTY}/UTL)

# ===========================================================================
# hdmapping_core_no_gui  (from HDMapping/core)
# ===========================================================================

add_library(hdmapping_core_no_gui STATIC
  ${HDMAPPING_DIR}/core/src/color_las_loader.cpp
  ${HDMAPPING_DIR}/core/src/control_points.cpp
  ${HDMAPPING_DIR}/core/src/gnss.cpp
  ${HDMAPPING_DIR}/core/src/ground_control_points.cpp
  ${HDMAPPING_DIR}/core/src/hash_utils.cpp
  ${HDMAPPING_DIR}/core/src/icp.cpp
  ${HDMAPPING_DIR}/core/src/ndt.cpp
  ${HDMAPPING_DIR}/core/src/nmea.cpp
  ${HDMAPPING_DIR}/core/src/optimization_point_to_point_source_to_target.cpp
  ${HDMAPPING_DIR}/core/src/optimize_distance_point_to_plane_source_to_target.cpp
  ${HDMAPPING_DIR}/core/src/optimize_plane_to_plane_source_to_target.cpp
  ${HDMAPPING_DIR}/core/src/optimize_point_to_plane_source_to_target.cpp
  ${HDMAPPING_DIR}/core/src/optimize_point_to_projection_onto_plane_source_to_target.cpp
  ${HDMAPPING_DIR}/core/src/pair_wise_iterative_closest_point.cpp
  ${HDMAPPING_DIR}/core/src/point_cloud.cpp
  ${HDMAPPING_DIR}/core/src/point_clouds.cpp
  ${HDMAPPING_DIR}/core/src/pose_graph_loop_closure.cpp
  ${HDMAPPING_DIR}/core/src/pose_graph_slam.cpp
  ${HDMAPPING_DIR}/core/src/registration_plane_feature.cpp
  ${HDMAPPING_DIR}/core/src/session.cpp
  ${HDMAPPING_EXTERNAL}/src/wgs84_do_puwg92.cc
  ${HDMAPPING_EXTERNAL}/src/plycpp.cpp
)

target_compile_definitions(hdmapping_core_no_gui PRIVATE
  WITH_GUI=0
  HDMAPPING_VERSION_MAJOR=0
  HDMAPPING_VERSION_MINOR=99
  HDMAPPING_VERSION_PATCH=0
)

target_include_directories(hdmapping_core_no_gui PUBLIC
  ${HDMAPPING_DIR}/core/include
  ${HDMAPPING_DIR}/shared/include
  ${EIGEN3_INCLUDE_DIR}
  ${HDMAPPING_3RDPARTY}/LASzip/include
  ${HDMAPPING_3RDPARTY}/json/include
  ${HDMAPPING_3RDPARTY}/observation_equations/codes
  ${HDMAPPING_EXTERNAL}/include
)

target_link_libraries(hdmapping_core_no_gui
  PRIVATE laszip TBB::tbb Threads::Threads
)

target_precompile_headers(hdmapping_core_no_gui
  PRIVATE ${HDMAPPING_DIR}/core/include/pch/pch.h
)

# ===========================================================================
# hdmapping_lio_algo  (from HDMapping/apps/lidar_odometry_step_1)
# ===========================================================================

add_library(hdmapping_lio_algo STATIC
  ${HDMAPPING_DIR}/apps/lidar_odometry_step_1/lidar_odometry.cpp
  ${HDMAPPING_DIR}/apps/lidar_odometry_step_1/lidar_odometry_utils.cpp
  ${HDMAPPING_DIR}/apps/lidar_odometry_step_1/lidar_odometry_utils_optimizers.cpp
  ${HDMAPPING_DIR}/apps/lidar_odometry_step_1/toml_io.cpp
)

target_compile_definitions(hdmapping_lio_algo PRIVATE
  WITH_GUI=0
  HDMAPPING_VERSION_MAJOR=0
  HDMAPPING_VERSION_MINOR=99
  HDMAPPING_VERSION_PATCH=0
  UTL_PROFILER_DISABLE
)

target_include_directories(hdmapping_lio_algo PUBLIC
  ${HDMAPPING_DIR}/apps/lidar_odometry_step_1
  ${HDMAPPING_DIR}/core/include
  ${HDMAPPING_DIR}/core_hd_mapping/include
  ${HDMAPPING_DIR}/shared/include
  ${HDMAPPING_3RDPARTY}
  ${EIGEN3_INCLUDE_DIR}
  ${HDMAPPING_3RDPARTY}/tomlplusplus/include
  ${HDMAPPING_3RDPARTY}/json/include
  ${HDMAPPING_3RDPARTY}/LASzip/include
  ${HDMAPPING_3RDPARTY}/observation_equations/codes
  ${HDMAPPING_3RDPARTY}/Fusion/Fusion
  ${HDMAPPING_EXTERNAL}/include
)

target_link_libraries(hdmapping_lio_algo PUBLIC
  hdmapping_core_no_gui
  Fusion
  unordered_dense::unordered_dense
  spdlog::spdlog
  UTL::include
  laszip
  TBB::tbb
)

# ===========================================================================
# hdmapping_lio ROS2 component library
# ===========================================================================

add_library(odometry_component SHARED
  src/odometry_node.cpp
  src/conversions.cpp
  src/pipeline.cpp
)

target_include_directories(odometry_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(odometry_component PUBLIC hdmapping_lio_algo)

ament_target_dependencies(odometry_component PUBLIC
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
)

rclcpp_components_register_node(odometry_component
  PLUGIN "hdmapping_lio::OdometryNode"
  EXECUTABLE hdmapping_lio_node
)

# ===========================================================================
# Install
# ===========================================================================

install(TARGETS odometry_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
