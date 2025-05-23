# Copyright 2016 The Cartographer Authors
# Copyright 2022 Wyca Robotics (for the ROS2 conversion)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(cartographer_ros)

find_package(ament_cmake REQUIRED)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(absl REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cartographer REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gflags REQUIRED)
find_package(glog REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_environment REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)

# glog is not linked, however we look for it to detect the glog version
# and use a different code path if glog >= 0.7.0 is detected
find_package(glog CONFIG QUIET)
if(DEFINED glog_VERSION)
  if(NOT glog_VERSION VERSION_LESS 0.7.0)
    add_definitions(-DROS_CARTOGRAPHER_GLOG_GE_070)
  endif()
endif()

include(FindPkgConfig)
pkg_search_module(CAIRO REQUIRED cairo>=1.12.16)

# Library
add_library(${PROJECT_NAME} SHARED
  src/assets_writer.cpp
  src/map_builder_bridge.cpp
  src/msg_conversion.cpp
  src/node_constants.cpp
  src/node.cpp
  src/node_options.cpp
  src/offline_node.cpp
  src/playable_bag.cpp
  src/ros_log_sink.cpp
  src/ros_map.cpp
  src/ros_map_writing_points_processor.cpp
  src/sensor_bridge.cpp
  src/submap.cpp
  src/tf_bridge.cpp
  src/time_conversion.cpp
  src/trajectory_options.cpp
  src/urdf_reader.cpp
  src/metrics/family_factory.cpp
  src/metrics/internal/family.cpp
  src/metrics/internal/histogram.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${CAIRO_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)
if ("$ENV{ROS_DISTRO}" STRLESS "kilted")
  # In Kilted, pcl_conversions was converted over to an INTERFACE target,
  # but prior to it it is only available as the INCLUDE_DIRS.  So we have this
  # compatibility hack to deal with that difference.
  target_include_directories(${PROJECT_NAME} PRIVATE ${pcl_conversions_INCLUDE_DIRS})
  # In Kilted, urdf/model.h was deprecated in favor of urdf/model.hpp.
  # Deal with the difference here so we have no warnings.
  target_compile_definitions(${PROJECT_NAME} PRIVATE "-DUSE_URDF_H_FILES")
else()
  target_link_libraries(${PROJECT_NAME} PRIVATE pcl_conversions::pcl_conversions)
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC
  absl::synchronization
  ${builtin_interfaces_TARGETS}
  cartographer
  ${cartographer_ros_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  ${sensor_msgs_TARGETS}
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  ${CAIRO_LIBRARIES}
  pcl_common
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_eigen::tf2_eigen
  ${tf2_msgs_TARGETS}
  urdf::urdf
)

# Executables
add_executable(cartographer_node src/node_main.cpp)
target_include_directories(cartographer_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(cartographer_node PRIVATE
  ${PROJECT_NAME}
  absl::memory
  cartographer
  gflags
  rclcpp::rclcpp
  tf2_ros::tf2_ros
)

add_executable(cartographer_occupancy_grid_node src/occupancy_grid_node_main.cpp)
target_include_directories(cartographer_occupancy_grid_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${CAIRO_INCLUDE_DIRS}
)
target_link_libraries(cartographer_occupancy_grid_node PRIVATE
  ${PROJECT_NAME}
  absl::synchronization
  cartographer
  ${cartographer_ros_msgs_TARGETS}
  Eigen3::Eigen
  gflags
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
)

add_executable(cartographer_offline_node src/offline_node_main.cpp)
target_include_directories(cartographer_offline_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(cartographer_offline_node PRIVATE
  ${PROJECT_NAME}
  cartographer
  gflags
  rclcpp::rclcpp
)

add_executable(cartographer_assets_writer src/assets_writer_main.cpp)
target_include_directories(cartographer_assets_writer PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(cartographer_assets_writer PRIVATE
  ${PROJECT_NAME}
  gflags
  glog::glog
  rclcpp::rclcpp
)

add_executable(cartographer_pbstream_map_publisher src/pbstream_map_publisher_main.cpp)
target_include_directories(cartographer_pbstream_map_publisher PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(cartographer_pbstream_map_publisher PRIVATE
  ${PROJECT_NAME}
  cartographer
  gflags
  glog::glog
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
)

add_executable(cartographer_pbstream_to_ros_map src/pbstream_to_ros_map_main.cpp)
target_include_directories(cartographer_pbstream_to_ros_map PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(cartographer_pbstream_to_ros_map PRIVATE
  ${PROJECT_NAME}
  cartographer
  gflags
  glog::glog
  rclcpp::rclcpp
)

add_executable(cartographer_rosbag_validate src/rosbag_validate_main.cpp)
target_include_directories(cartographer_rosbag_validate PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
if ("$ENV{ROS_DISTRO}" STRLESS "kilted")
  target_compile_definitions(cartographer_rosbag_validate PRIVATE "-DUSE_URDF_H_FILES")
endif()
target_link_libraries(cartographer_rosbag_validate
  ${PROJECT_NAME}
  absl::memory
  cartographer
  gflags
  glog::glog
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  ${sensor_msgs_TARGETS}
  tf2_eigen::tf2_eigen
  ${tf2_msgs_TARGETS}
  tf2_ros::tf2_ros
  urdf::urdf
  tf2::tf2
)

if($ENV{ROS_DISTRO} MATCHES "humble" OR $ENV{ROS_DISTRO} MATCHES "iron")
  target_compile_definitions(${PROJECT_NAME} PRIVATE PRE_JAZZY_SERIALIZED_BAG_MSG_FIELD_NAME)
  target_compile_definitions(cartographer_rosbag_validate PRIVATE PRE_JAZZY_SERIALIZED_BAG_MSG_FIELD_NAME)
endif()

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  cartographer_node
  cartographer_occupancy_grid_node
  cartographer_offline_node
  cartographer_assets_writer
  cartographer_pbstream_map_publisher
  cartographer_pbstream_to_ros_map
  cartographer_rosbag_validate
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY configuration_files urdf launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_targets(${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  absl
  builtin_interfaces
  cartographer
  cartographer_ros_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  sensor_msgs
  tf2_ros
  visualization_msgs
)
ament_package()

# Non converted bin:
#google_binary(cartographer_dev_pbstream_trajectories_to_rosbag
#  SRCS
#  dev/pbstream_trajectories_to_rosbag_main.cc
#)

#google_binary(cartographer_dev_rosbag_publisher
#  SRCS
#  dev/rosbag_publisher_main.cc
#)


#google_binary(cartographer_dev_trajectory_comparison
#  SRCS
#  dev/trajectory_comparison_main.cc
#)


## TODO(cschuet): Add support for shared library case.
#if (${BUILD_GRPC})
#  google_binary(cartographer_grpc_node
#    SRCS
#      cartographer_grpc/node_grpc_main.cc
#  )


#  google_binary(cartographer_grpc_offline_node
#    SRCS
#      cartographer_grpc/offline_node_grpc_main.cc
#  )

#  install(PROGRAMS
#    ../scripts/cartographer_grpc_server.sh
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )
#endif()
