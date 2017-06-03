# Copyright 2016 The Cartographer Authors
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

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

find_package(cartographer_ros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include(FindPkgConfig)
pkg_search_module(YAMLCPP REQUIRED yaml-cpp>=0.5.1)

find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
google_initialize_cartographer_project()
google_enable_testing()

find_package(console_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(LuaGoogle REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(urdfdom_headers REQUIRED)

if(DEFINED urdfdom_headers_VERSION)
  if(${urdfdom_headers_VERSION} GREATER 0.4.1)
    add_definitions(-DURDFDOM_HEADERS_HAS_SHARED_PTR_DEFS)
  endif()
endif()

include_directories(
  include
  "."
  ${cartographer_INCLUDE_DIRS}
  ${cartographer_ros_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
)

set(ALL_SRCS
  "cartographer_ros/assets_writer.cc"
  "cartographer_ros/map_builder_bridge.cc"
  "cartographer_ros/map_writer.cc"
  "cartographer_ros/msg_conversion.cc"
  "cartographer_ros/node.cc"
  "cartographer_ros/node_options.cc"
  "cartographer_ros/occupancy_grid.cc"
  "cartographer_ros/ros_log_sink.cc"
  "cartographer_ros/sensor_bridge.cc"
  "cartographer_ros/tf_bridge.cc"
  "cartographer_ros/time_conversion.cc"
  "cartographer_ros/trajectory_options.cc"
  )
add_library(${PROJECT_NAME} ${ALL_SRCS})

# Cartographer
target_link_libraries(${PROJECT_NAME} PUBLIC cartographer)

# Eigen
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")

# Lua
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})

# PCL
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
foreach(DEFINITION ${PCL_DEFINITIONS})
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}")
endforeach()

# YAML
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${YAMLCPP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${YAMLCPP_LIBRARIES})

# URDFDOM
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${urdfdom_headers_INCLUDE_DIRS})

add_subdirectory("cartographer_ros")

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY configuration_files
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
