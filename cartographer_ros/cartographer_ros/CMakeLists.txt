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

add_executable(cartographer_node
  node_main.cc)
target_include_directories(cartographer_node SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
target_link_libraries(cartographer_node ${PROJECT_NAME})
ament_target_dependencies(cartographer_node
  "cartographer_ros_msgs"
  "console_bridge"
  "nav_msgs"
  "pcl_conversions"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_msgs"
  "tf2_ros"
)

install(TARGETS
  cartographer_node
  DESTINATION lib/${PROJECT_NAME})

# TODO(clalancette): Port these over to ROS2 once we have rosbag
# add_executable(cartographer_assets_writer
#   assets_writer_main.cc)
# target_include_directories(cartographer_assets_writer SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
# target_link_libraries(cartographer_assets_writer ${PROJECT_NAME})
# ament_target_dependencies(cartographer_assets_writer
#   "cartographer_ros_msgs"
#   "console_bridge"
#   "nav_msgs"
#   "pcl_conversions"
#   "rclcpp"
#   "sensor_msgs"
#   "tf2"
#   "tf2_msgs"
#   "tf2_ros"
# )
#
# install(TARGETS
#   cartographer_assets_writer
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)
#
# add_executable(cartographer_offline_node
#   offline_node_main.cc)
# target_include_directories(cartographer_offline_node SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
# target_link_libraries(cartographer_offline_node ${PROJECT_NAME})
# ament_target_dependencies(cartographer_offline_node
#   "cartographer_ros_msgs"
#   "console_bridge"
#   "nav_msgs"
#   "pcl_conversions"
#   "rclcpp"
#   "sensor_msgs"
#   "tf2"
#   "tf2_msgs"
#   "tf2_ros"
# )
#
# install(TARGETS
#   cartographer_offline_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)
