/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_MSG_CONVERSION_H_
#define CARTOGRAPHER_ROS_MSG_CONVERSION_H_

#include "cartographer/common/port.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace cartographer_ros {

sensor_msgs::msg::PointCloud2 ToPointCloud2Message(
    int64_t timestamp, const std::string& frame_id,
    const ::cartographer::sensor::TimedPointCloud& point_cloud);

geometry_msgs::msg::Transform ToGeometryMsgTransform(
    const ::cartographer::transform::Rigid3d& rigid3d);

geometry_msgs::msg::Pose ToGeometryMsgPose(
    const ::cartographer::transform::Rigid3d& rigid3d);

geometry_msgs::msg::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d);

::cartographer::sensor::PointCloudWithIntensities ToPointCloudWithIntensities(
    const sensor_msgs::msg::LaserScan& msg);

::cartographer::sensor::PointCloudWithIntensities ToPointCloudWithIntensities(
    const sensor_msgs::msg::MultiEchoLaserScan& msg);

::cartographer::sensor::PointCloudWithIntensities ToPointCloudWithIntensities(
    const sensor_msgs::msg::PointCloud2& message);

::cartographer::transform::Rigid3d ToRigid3d(
    const geometry_msgs::msg::TransformStamped& transform);

::cartographer::transform::Rigid3d ToRigid3d(const geometry_msgs::msg::Pose& pose);

Eigen::Vector3d ToEigen(const geometry_msgs::msg::Vector3& vector3);

Eigen::Quaterniond ToEigen(const geometry_msgs::msg::Quaternion& quaternion);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_MSG_CONVERSION_H_
