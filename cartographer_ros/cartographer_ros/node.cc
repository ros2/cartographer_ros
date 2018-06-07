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

#include "cartographer_ros/node.h"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "glog/logging.h"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/time_source.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include "visualization_msgs/msg/marker_array.hpp"

namespace cartographer_ros {

namespace {

cartographer_ros_msgs::msg::SensorTopics DefaultSensorTopics() {
  cartographer_ros_msgs::msg::SensorTopics topics;
  topics.laser_scan_topic = kLaserScanTopic;
  topics.multi_echo_laser_scan_topic = kMultiEchoLaserScanTopic;
  topics.point_cloud2_topic = kPointCloud2Topic;
  topics.imu_topic = kImuTopic;
  topics.odometry_topic = kOdometryTopic;
  return topics;
}

// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
template <typename MessageType>
::rclcpp::SubscriptionBase::SharedPtr SubscribeWithHandler(
    void (Node::*handler)(int, const std::string&,
                          const typename MessageType::ConstSharedPtr),
    const int trajectory_id, const std::string& topic,
    ::rclcpp::Node::SharedPtr node_handle, Node* const node,
    rmw_qos_profile_t custom_qos_profile) {
  return node_handle->create_subscription<MessageType>(
      topic,
      [node, handler, trajectory_id, topic](const typename MessageType::ConstSharedPtr msg) {
            (node->*handler)(trajectory_id, topic, msg);
      },
      custom_qos_profile);
}

}  // namespace

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

Node::Node(const NodeOptions& node_options, tf2_ros::Buffer* const tf_buffer)
    : node_options_(node_options),
      map_builder_bridge_(node_options_, tf_buffer) {
  carto::common::MutexLocker lock(&mutex_);
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

  custom_qos_profile.depth = 50;
  custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;

  node_handle_ = rclcpp::Node::make_shared("cartographer_node");
  submap_list_publisher_ =
      node_handle_->create_publisher<::cartographer_ros_msgs::msg::SubmapList>(
          kSubmapListTopic, custom_qos_profile);
      // node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
      //     kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  trajectory_node_list_publisher_ =
      node_handle_->create_publisher<::visualization_msgs::msg::MarkerArray>(
          kTrajectoryNodeListTopic, custom_qos_profile);
      // node_handle_.advertise<::visualization_msgs::MarkerArray>(
      //     kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
  constraint_list_publisher_ =
      node_handle_->create_publisher<::visualization_msgs::msg::MarkerArray>(
          kConstraintListTopic, custom_qos_profile);
      // node_handle_.advertise<::visualization_msgs::MarkerArray>(
      //     kConstraintListTopic, kLatestOnlyPublisherQueueSize);
  service_servers_.push_back(node_handle_->create_service<cartographer_ros_msgs::srv::SubmapQuery>(
      kSubmapQueryServiceName, std::bind(&Node::HandleSubmapQuery, this, std::placeholders::_1, std::placeholders::_2)));
  service_servers_.push_back(node_handle_->create_service<cartographer_ros_msgs::srv::StartTrajectory>(
      kStartTrajectoryServiceName, std::bind(&Node::HandleStartTrajectory, this, std::placeholders::_1, std::placeholders::_2)));
  service_servers_.push_back(node_handle_->create_service<cartographer_ros_msgs::srv::FinishTrajectory>(
      kFinishTrajectoryServiceName, std::bind(&Node::HandleFinishTrajectory, this, std::placeholders::_1, std::placeholders::_2)));
  service_servers_.push_back(node_handle_->create_service<cartographer_ros_msgs::srv::WriteState>(
      kWriteStateServiceName, std::bind(&Node::HandleWriteState, this, std::placeholders::_1, std::placeholders::_2)));

  scan_matched_point_cloud_publisher_ =
      node_handle_->create_publisher<sensor_msgs::msg::PointCloud2>(
          kScanMatchedPointCloudTopic, custom_qos_profile);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_handle_);

  wall_timers_.push_back(node_handle_->create_wall_timer(
    std::chrono::milliseconds(int(node_options_.submap_publish_period_sec * 1000)),
    std::bind(&Node::PublishSubmapList, this)));
  wall_timers_.push_back(node_handle_->create_wall_timer(
    std::chrono::milliseconds(int(node_options_.pose_publish_period_sec * 1000)),
    std::bind(&Node::PublishTrajectoryStates, this)));
  wall_timers_.push_back(node_handle_->create_wall_timer(
    std::chrono::milliseconds(int(node_options_.trajectory_publish_period_sec * 1000)),
    std::bind(&Node::PublishTrajectoryNodeList, this)));
  wall_timers_.push_back(node_handle_->create_wall_timer(
    std::chrono::milliseconds(int(kConstraintPublishPeriodSec * 1000)),
    std::bind(&Node::PublishConstraintList, this)));

  ts_ = std::make_shared<rclcpp::TimeSource>(node_handle_);
  clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ts_->attachClock(clock_);
}

Node::~Node() {}

::rclcpp::Node::SharedPtr Node::node_handle() { return node_handle_; }

void Node::HandleSubmapQuery(
    const std::shared_ptr<::cartographer_ros_msgs::srv::SubmapQuery::Request> request,
    std::shared_ptr<::cartographer_ros_msgs::srv::SubmapQuery::Response> response) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.HandleSubmapQuery(request, response);
  return;
}

void Node::PublishSubmapList() {
  carto::common::MutexLocker lock(&mutex_);
  submap_list_publisher_->publish(map_builder_bridge_.GetSubmapList(clock_));
}

void Node::AddExtrapolator(const int trajectory_id,
                           const TrajectoryOptions& options) {
  constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
  CHECK(extrapolators_.count(trajectory_id) == 0);
  const double gravity_time_constant =
      node_options_.map_builder_options.use_trajectory_builder_3d()
          ? options.trajectory_builder_options.trajectory_builder_3d_options()
                .imu_gravity_time_constant()
          : options.trajectory_builder_options.trajectory_builder_2d_options()
                .imu_gravity_time_constant();
  extrapolators_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
          gravity_time_constant));
}

void Node::AddSensorSamplers(const int trajectory_id,
                             const TrajectoryOptions& options) {
  CHECK(sensor_samplers_.count(trajectory_id) == 0);
  sensor_samplers_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(options.rangefinder_sampling_ratio,
                            options.odometry_sampling_ratio,
                            options.imu_sampling_ratio));
}

void Node::PublishTrajectoryStates() {
  carto::common::MutexLocker lock(&mutex_);
  for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
    const auto& trajectory_state = entry.second;

    auto& extrapolator = extrapolators_.at(entry.first);
    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    if (trajectory_state.local_slam_data->time !=
        extrapolator.GetLastPoseTime()) {
      // TODO(gaschler): Consider using other message without time information.
      carto::sensor::TimedPointCloud point_cloud;
      point_cloud.reserve(
          trajectory_state.local_slam_data->range_data_in_local.returns.size());
      for (const Eigen::Vector3f point :
           trajectory_state.local_slam_data->range_data_in_local.returns) {
        Eigen::Vector4f point_time;
        point_time << point, 0.f;
        point_cloud.push_back(point_time);
      }
      scan_matched_point_cloud_publisher_->publish(ToPointCloud2Message(
          carto::common::ToUniversal(trajectory_state.local_slam_data->time),
          node_options_.map_frame,
          carto::sensor::TransformTimedPointCloud(
              point_cloud, trajectory_state.local_to_map.cast<float>())));
      extrapolator.AddPose(trajectory_state.local_slam_data->time,
                           trajectory_state.local_slam_data->local_pose);
    }
    geometry_msgs::msg::TransformStamped stamped_transform;
    // If we do not publish a new point cloud, we still allow time of the
    // published poses to advance. If we already know a newer pose, we use its
    // time instead. Since tf knows how to interpolate, providing newer
    // information is better.
    const ::cartographer::common::Time now = std::max(
	FromRos(clock_->now()), extrapolator.GetLastExtrapolatedTime());
    stamped_transform.header.stamp = ToRos(now);
    const Rigid3d tracking_to_local = extrapolator.ExtrapolatePose(now);
    const Rigid3d tracking_to_map =
	trajectory_state.local_to_map * tracking_to_local;

    if (trajectory_state.published_to_tracking != nullptr) {
      if (trajectory_state.trajectory_options.provide_odom_frame) {
	std::vector<geometry_msgs::msg::TransformStamped> stamped_transforms;

	stamped_transform.header.frame_id = node_options_.map_frame;
	stamped_transform.child_frame_id =
	    trajectory_state.trajectory_options.odom_frame;
	stamped_transform.transform =
	    ToGeometryMsgTransform(trajectory_state.local_to_map);
	stamped_transforms.push_back(stamped_transform);

	stamped_transform.header.frame_id =
	    trajectory_state.trajectory_options.odom_frame;
	stamped_transform.child_frame_id =
	    trajectory_state.trajectory_options.published_frame;
	stamped_transform.transform = ToGeometryMsgTransform(
	    tracking_to_local * (*trajectory_state.published_to_tracking));
	stamped_transforms.push_back(stamped_transform);

	tf_broadcaster_->sendTransform(stamped_transforms);
      } else {
	stamped_transform.header.frame_id = node_options_.map_frame;
	stamped_transform.child_frame_id =
	    trajectory_state.trajectory_options.published_frame;
	stamped_transform.transform = ToGeometryMsgTransform(
	    tracking_to_map * (*trajectory_state.published_to_tracking));
	tf_broadcaster_->sendTransform(stamped_transform);
      }
    }
  }
}

void Node::PublishTrajectoryNodeList() {
  carto::common::MutexLocker lock(&mutex_);
  if (node_handle_->count_subscribers(kTrajectoryNodeListTopic) > 0) {
    trajectory_node_list_publisher_->publish(
        map_builder_bridge_.GetTrajectoryNodeList(clock_));
  }
}

void Node::PublishConstraintList() {
  carto::common::MutexLocker lock(&mutex_);
  if (node_handle_->count_subscribers(kConstraintListTopic) > 0) {
    constraint_list_publisher_->publish(map_builder_bridge_.GetConstraintList(clock_));
  }
}

std::unordered_set<std::string> Node::ComputeExpectedTopics(
    const TrajectoryOptions& options,
    const cartographer_ros_msgs::msg::SensorTopics& topics) {
  std::unordered_set<std::string> expected_topics;
  // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.laser_scan_topic, options.num_laser_scans)) {
    expected_topics.insert(topic);
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                 options.num_multi_echo_laser_scans)) {
    expected_topics.insert(topic);
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.point_cloud2_topic, options.num_point_clouds)) {
    expected_topics.insert(topic);
  }
  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    expected_topics.insert(topics.imu_topic);
  }
  // Odometry is optional.
  if (options.use_odometry) {
    expected_topics.insert(topics.odometry_topic);
  }
  return expected_topics;
}

int Node::AddTrajectory(const TrajectoryOptions& options,
                        const cartographer_ros_msgs::msg::SensorTopics& topics) {
  const std::unordered_set<std::string> expected_sensor_ids =
      ComputeExpectedTopics(options, topics);
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  LaunchSubscribers(options, topics, trajectory_id);
  is_active_trajectory_[trajectory_id] = true;
  subscribed_topics_.insert(expected_sensor_ids.begin(),
                            expected_sensor_ids.end());
  return trajectory_id;
}

void Node::LaunchSubscribers(const TrajectoryOptions& options,
                             const cartographer_ros_msgs::msg::SensorTopics& topics,
                             const int trajectory_id) {
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

  custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  custom_qos_profile.depth = 50;
  custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  custom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

  // TODO(mikaelarguedas) pass qos profile aroung
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.laser_scan_topic, options.num_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::LaserScan>(
             &Node::HandleLaserScanMessage, trajectory_id, topic, node_handle_,
             this, custom_qos_profile),
         topic});
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                 options.num_multi_echo_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::MultiEchoLaserScan>(
             &Node::HandleMultiEchoLaserScanMessage, trajectory_id, topic,
             node_handle_, this, custom_qos_profile),
         topic});
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.point_cloud2_topic, options.num_point_clouds)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::PointCloud2>(
             &Node::HandlePointCloud2Message, trajectory_id, topic,
             node_handle_, this, custom_qos_profile),
         topic});
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    std::string topic = topics.imu_topic;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::Imu>(&Node::HandleImuMessage,
                                                     trajectory_id, topic,
                                                     node_handle_, this, custom_qos_profile),
         topic});
  }

  if (options.use_odometry) {
    std::string topic = topics.odometry_topic;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<nav_msgs::msg::Odometry>(&Node::HandleOdometryMessage,
                                                       trajectory_id, topic,
                                                       node_handle_, this, custom_qos_profile),
         topic});
  }
}

bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_2d_options();
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_3d_options();
  }
  return false;
}

bool Node::ValidateTopicNames(
    const ::cartographer_ros_msgs::msg::SensorTopics& topics,
    const TrajectoryOptions& options) {
  for (const std::string& topic : ComputeExpectedTopics(options, topics)) {
    if (subscribed_topics_.count(topic) > 0) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

bool Node::FinishTrajectoryUnderLock(const int trajectory_id) {
  if (is_active_trajectory_.count(trajectory_id) == 0) {
    LOG(INFO) << "Trajectory_id " << trajectory_id << " is not created yet.";
    return false;
  }
  if (!is_active_trajectory_[trajectory_id]) {
    LOG(INFO) << "Trajectory_id " << trajectory_id
              << " has already been finished.";
    return false;
  }

  // Shutdown the subscribers of this trajectory.
  for (auto& entry : subscribers_[trajectory_id]) {
    subscribed_topics_.erase(entry.topic);
    LOG(INFO) << "Shutdown the subscriber of [" << entry.topic << "]";
  }
  CHECK_EQ(subscribers_.erase(trajectory_id), 1);
  CHECK(is_active_trajectory_.at(trajectory_id));
  map_builder_bridge_.FinishTrajectory(trajectory_id);
  is_active_trajectory_[trajectory_id] = false;
  return true;
}

void Node::HandleStartTrajectory(
    const std::shared_ptr<::cartographer_ros_msgs::srv::StartTrajectory::Request> request,
    std::shared_ptr<::cartographer_ros_msgs::srv::StartTrajectory::Response> response) {
  carto::common::MutexLocker lock(&mutex_);
  TrajectoryOptions options;
  if (!FromRosMessage(request->options, &options) ||
      !ValidateTrajectoryOptions(options)) {
    LOG(ERROR) << "Invalid trajectory options.";
    return;
  }
  if (!ValidateTopicNames(request->topics, options)) {
    LOG(ERROR) << "Invalid topics.";
    return;
  }
  response->trajectory_id = AddTrajectory(options, request->topics);
  return;
}

void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  CHECK(ValidateTrajectoryOptions(options));
  AddTrajectory(options, DefaultSensorTopics());
}

std::unordered_set<std::string> Node::ComputeDefaultTopics(
    const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  return ComputeExpectedTopics(options, DefaultSensorTopics());
}

int Node::AddOfflineTrajectory(
    const std::unordered_set<std::string>& expected_sensor_ids,
    const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  is_active_trajectory_[trajectory_id] = true;
  return trajectory_id;
}

void Node::HandleFinishTrajectory(
    const std::shared_ptr<::cartographer_ros_msgs::srv::FinishTrajectory::Request> request,
    std::shared_ptr<::cartographer_ros_msgs::srv::FinishTrajectory::Response> response) {
  (void)response;
  carto::common::MutexLocker lock(&mutex_);
  FinishTrajectoryUnderLock(request->trajectory_id);
  return;
}

void Node::HandleWriteState(
    const std::shared_ptr<::cartographer_ros_msgs::srv::WriteState::Request> request,
    std::shared_ptr<::cartographer_ros_msgs::srv::WriteState::Response> response) {
  (void)response;
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.SerializeState(request->filename);
  return;
}

void Node::FinishAllTrajectories() {
  carto::common::MutexLocker lock(&mutex_);
  for (auto& entry : is_active_trajectory_) {
    const int trajectory_id = entry.first;
    if (entry.second) {
      CHECK(FinishTrajectoryUnderLock(trajectory_id));
    }
  }
}

bool Node::FinishTrajectory(const int trajectory_id) {
  carto::common::MutexLocker lock(&mutex_);
  return FinishTrajectoryUnderLock(trajectory_id);
}

void Node::RunFinalOptimization() {
  {
    carto::common::MutexLocker lock(&mutex_);
    for (const auto& entry : is_active_trajectory_) {
      CHECK(!entry.second);
    }
  }
  // Assuming we are not adding new data anymore, the final optimization
  // can be performed without holding the mutex.
  map_builder_bridge_.RunFinalOptimization();
}

void Node::HandleOdometryMessage(const int trajectory_id,
                                 const std::string& sensor_id,
                                 const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);
  if (odometry_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
  }
  sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

void Node::HandleImuMessage(const int trajectory_id,
                            const std::string& sensor_id,
                            const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
  if (imu_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
  }
  sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

void Node::HandleLaserScanMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLaserScanMessage(sensor_id, msg);
}

void Node::HandleMultiEchoLaserScanMessage(
    int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}

void Node::HandlePointCloud2Message(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandlePointCloud2Message(sensor_id, msg);
}

void Node::SerializeState(const std::string& filename) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.SerializeState(filename);
}

void Node::LoadMap(const std::string& map_filename) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.LoadMap(map_filename);
}

}  // namespace cartographer_ros
