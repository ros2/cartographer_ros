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

project(cartographer_rviz)
set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)

find_package(ament_cmake REQUIRED)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_AUTOMOC ON)

find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
set(BUILD_SHARED_LIBS OFF)
option(BUILD_GRPC "build features that require Cartographer gRPC support" false)
google_initialize_cartographer_project()

find_package(absl REQUIRED)
find_package(cartographer_ros REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

set(rviz_plugins_headers_to_moc
  include/cartographer_rviz/drawable_submap.h
  include/cartographer_rviz/ogre_slice.h
  include/cartographer_rviz/submaps_display.h
)

add_library(${PROJECT_NAME} SHARED
  src/drawable_submap.cpp
  src/ogre_slice.cpp
  src/submaps_display.cpp
  ${rviz_plugins_headers_to_moc}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  absl::synchronization
  cartographer
  cartographer_ros::cartographer_ros
  Eigen3::Eigen
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  tf2_ros::tf2_ros
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  pluginlib::pluginlib
)

pluginlib_export_plugin_description_file(rviz_common rviz_plugin_description.xml)
register_rviz_ogre_media_exports(DIRECTORIES "ogre_media/materials/glsl120" "ogre_media/materials/scripts")

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  absl
  cartographer
  cartographer_ros
  eigen
  rclcpp
  rviz_common
  rviz_rendering
  tf2_ros
)

ament_package()
