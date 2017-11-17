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
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "glog/logging.h"

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/time_source.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.h>

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

namespace {

constexpr int kInfiniteSubscriberQueueSize = 0;
constexpr int kLatestOnlyPublisherQueueSize = 1;

// Try to convert 'msg' into 'options'. Returns false on failure.
bool FromRosMessage(const cartographer_ros_msgs::msg::TrajectoryOptions& msg,
                    TrajectoryOptions* options) {
  options->tracking_frame = msg.tracking_frame;
  options->published_frame = msg.published_frame;
  options->odom_frame = msg.odom_frame;
  options->provide_odom_frame = msg.provide_odom_frame;
  options->use_odometry = msg.use_odometry;
  options->use_laser_scan = msg.use_laser_scan;
  options->use_multi_echo_laser_scan = msg.use_multi_echo_laser_scan;
  options->num_point_clouds = msg.num_point_clouds;
  if (!options->trajectory_builder_options.ParseFromString(
          msg.trajectory_builder_options_proto)) {
    LOG(ERROR) << "Failed to parse protobuf";
    return false;
  }
  return true;
}

void ShutdownSubscriber(std::unordered_map<int, ::rclcpp::subscription::SubscriptionBase::SharedPtr>& subscribers,
                        int trajectory_id) {
  if (subscribers.count(trajectory_id) == 0) {
    return;
  }
  LOG(INFO) << "Shutdown the subscriber of ["
            << subscribers[trajectory_id]->get_topic_name() << "]";
  CHECK_EQ(subscribers.erase(trajectory_id), 1);
}

bool IsTopicNameUnique(
    const string& topic,
    const std::unordered_map<int, ::rclcpp::subscription::SubscriptionBase::SharedPtr>& subscribers) {
  for (auto& entry : subscribers) {
    if (entry.second->get_topic_name() == topic) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

}  // namespace

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
  service_servers_.push_back(node_handle_->create_service<::cartographer_ros_msgs::srv::SubmapQuery>(
      kSubmapQueryServiceName, std::bind(&Node::HandleSubmapQuery, this, std::placeholders::_1, std::placeholders::_2)));
  service_servers_.push_back(node_handle_->create_service<::cartographer_ros_msgs::srv::StartTrajectory>(
      kStartTrajectoryServiceName, std::bind(&Node::HandleStartTrajectory, this, std::placeholders::_1, std::placeholders::_2)));
  service_servers_.push_back(node_handle_->create_service<::cartographer_ros_msgs::srv::FinishTrajectory>(
      kFinishTrajectoryServiceName, std::bind(&Node::HandleFinishTrajectory, this, std::placeholders::_1, std::placeholders::_2)));
  service_servers_.push_back(node_handle_->create_service<::cartographer_ros_msgs::srv::WriteAssets>(
      kWriteAssetsServiceName, std::bind(&Node::HandleWriteAssets, this, std::placeholders::_1, std::placeholders::_2)));

  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    occupancy_grid_publisher_ =
      node_handle_->create_publisher<::nav_msgs::msg::OccupancyGrid>(
          kOccupancyGridTopic, custom_qos_profile);
    occupancy_grid_thread_ =
        std::thread(&Node::SpinOccupancyGridThreadForever, this);
  }

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
}

Node::~Node() {
  {
    carto::common::MutexLocker lock(&mutex_);
    terminating_ = true;
  }
  if (occupancy_grid_thread_.joinable()) {
    occupancy_grid_thread_.join();
  }
}

::rclcpp::node::Node::SharedPtr Node::node_handle() { return node_handle_; }

MapBuilderBridge* Node::map_builder_bridge() { return &map_builder_bridge_; }

void Node::HandleSubmapQuery(
    const std::shared_ptr<::cartographer_ros_msgs::srv::SubmapQuery::Request> request,
    std::shared_ptr<::cartographer_ros_msgs::srv::SubmapQuery::Response> response) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.HandleSubmapQuery(request, response);
}

void Node::PublishSubmapList() {
  carto::common::MutexLocker lock(&mutex_);
  submap_list_publisher_->publish(map_builder_bridge_.GetSubmapList(node_handle_));
}

void Node::PublishTrajectoryStates() {
  carto::common::MutexLocker lock(&mutex_);
  for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
    const auto& trajectory_state = entry.second;

    geometry_msgs::msg::TransformStamped stamped_transform;
    stamped_transform.header.stamp = ToRos(trajectory_state.pose_estimate.time);

    const auto& tracking_to_local = trajectory_state.pose_estimate.pose;
    const Rigid3d tracking_to_map =
        trajectory_state.local_to_map * tracking_to_local;

    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    if (trajectory_state.pose_estimate.time !=
        last_scan_matched_point_cloud_time_) {
      scan_matched_point_cloud_publisher_->publish(ToPointCloud2Message(
          carto::common::ToUniversal(trajectory_state.pose_estimate.time),
          trajectory_state.trajectory_options.tracking_frame,
          carto::sensor::TransformPointCloud(
              trajectory_state.pose_estimate.point_cloud,
              tracking_to_local.inverse().cast<float>())));
      last_scan_matched_point_cloud_time_ = trajectory_state.pose_estimate.time;
    } else {
      // If we do not publish a new point cloud, we still allow time of the
      // published poses to advance.
      rclcpp::TimeSource ts(node_handle_);
      rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
      ts.attachClock(clock);

      stamped_transform.header.stamp = clock->now();
    }

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

void Node::SpinOccupancyGridThreadForever() {
  for (;;) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    {
      carto::common::MutexLocker lock(&mutex_);
      if (terminating_) {
        return;
      }
    }
    if (node_handle_->count_subscribers(kOccupancyGridTopic) == 0) {
      continue;
    }
    const auto occupancy_grid = map_builder_bridge_.BuildOccupancyGrid();
    if (occupancy_grid != nullptr) {
      occupancy_grid_publisher_->publish(*occupancy_grid);
    }
  }
}

int Node::AddTrajectory(const TrajectoryOptions& options,
                        const cartographer_ros_msgs::msg::SensorTopics& topics) {
  std::unordered_set<string> expected_sensor_ids;

  if (options.use_laser_scan) {
    expected_sensor_ids.insert(topics.laser_scan_topic);
  }
  if (options.use_multi_echo_laser_scan) {
    expected_sensor_ids.insert(topics.multi_echo_laser_scan_topic);
  }
  if (options.num_point_clouds > 0) {
    for (int i = 0; i < options.num_point_clouds; ++i) {
      string topic = topics.point_cloud2_topic;
      if (options.num_point_clouds > 1) {
        topic += "_" + std::to_string(i + 1);
      }
      expected_sensor_ids.insert(topic);
    }
  }
  if (options.trajectory_builder_options.trajectory_builder_2d_options()
          .use_imu_data()) {
    expected_sensor_ids.insert(topics.imu_topic);
  }
  if (options.use_odometry) {
    expected_sensor_ids.insert(topics.odometry_topic);
  }
  return map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
}

void Node::LaunchSubscribers(const TrajectoryOptions& options,
                             const cartographer_ros_msgs::msg::SensorTopics& topics,
                             const int trajectory_id) {
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

  custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  custom_qos_profile.depth = 50;
  custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  custom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

  if (options.use_laser_scan) {
    const string topic = topics.laser_scan_topic;
    laser_scan_subscribers_[trajectory_id] =
        node_handle_->create_subscription<sensor_msgs::msg::LaserScan>(
            topic,
            [this, trajectory_id, topic](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
              map_builder_bridge_.sensor_bridge(trajectory_id)
                  ->HandleLaserScanMessage(topic, msg);
            }, custom_qos_profile);
  }

  if (options.use_multi_echo_laser_scan) {
    const string topic = topics.multi_echo_laser_scan_topic;
    multi_echo_laser_scan_subscribers_[trajectory_id] =
        node_handle_->create_subscription<sensor_msgs::msg::MultiEchoLaserScan>(
            topic,
            [this, trajectory_id, topic](const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr msg) {
              map_builder_bridge_.sensor_bridge(trajectory_id)
                  ->HandleMultiEchoLaserScanMessage(topic, msg);
            }, custom_qos_profile);
  }

  std::vector<::rclcpp::subscription::SubscriptionBase::SharedPtr> grouped_point_cloud_subscribers;
  if (options.num_point_clouds > 0) {
    for (int i = 0; i < options.num_point_clouds; ++i) {
      string topic = topics.point_cloud2_topic;
      if (options.num_point_clouds > 1) {
        topic += "_" + std::to_string(i + 1);
      }
      grouped_point_cloud_subscribers.push_back(node_handle_->create_subscription<sensor_msgs::msg::PointCloud2>(
          topic,
          [this, trajectory_id, topic](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            map_builder_bridge_.sensor_bridge(trajectory_id)
                ->HandlePointCloud2Message(topic, msg);
          }, custom_qos_profile));
    }
    point_cloud_subscribers_[trajectory_id] = grouped_point_cloud_subscribers;
  }

  if (options.trajectory_builder_options.trajectory_builder_2d_options()
          .use_imu_data()) {
    string topic = topics.imu_topic;
    imu_subscribers_[trajectory_id] = node_handle_->create_subscription<sensor_msgs::msg::Imu>(
        topic,
        [this, trajectory_id, topic](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          map_builder_bridge_.sensor_bridge(trajectory_id)
              ->HandleImuMessage(topic, msg);
        }, custom_qos_profile);
  }

  if (options.use_odometry) {
    string topic = topics.odometry_topic;
    odom_subscribers_[trajectory_id] =
        node_handle_->create_subscription<nav_msgs::msg::Odometry>(
            topic,
            [this, trajectory_id, topic](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
              map_builder_bridge_.sensor_bridge(trajectory_id)
                  ->HandleOdometryMessage(topic, msg);
            }, custom_qos_profile);
  }

  is_active_trajectory_[trajectory_id] = true;
}

bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d() &&
      options.trajectory_builder_options.has_trajectory_builder_2d_options()) {
    // Using point clouds is only supported in 3D.
    if (options.num_point_clouds == 0) {
      return true;
    }
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d() &&
      options.trajectory_builder_options.has_trajectory_builder_3d_options()) {
    if (options.num_point_clouds != 0) {
      return true;
    }
  }
  return false;
}

bool Node::ValidateTopicName(
    const ::cartographer_ros_msgs::msg::SensorTopics& topics,
    const TrajectoryOptions& options) {
  if (!IsTopicNameUnique(topics.laser_scan_topic, laser_scan_subscribers_)) {
    return false;
  }
  if (!IsTopicNameUnique(topics.multi_echo_laser_scan_topic,
                         multi_echo_laser_scan_subscribers_)) {
    return false;
  }
  if (!IsTopicNameUnique(topics.imu_topic, imu_subscribers_)) {
    return false;
  }
  if (!IsTopicNameUnique(topics.odometry_topic, odom_subscribers_)) {
    return false;
  }
  for (auto& subscribers : point_cloud_subscribers_) {
    string topic = topics.point_cloud2_topic;
    int count = 0;
    for (auto& subscriber : subscribers.second) {
      if (options.num_point_clouds > 1) {
        topic += "_" + std::to_string(count + 1);
        ++count;
      }
      if (subscriber->get_topic_name() == topic) {
        LOG(ERROR) << "Topic name [" << topic << "] is already used";
        return false;
      }
    }
  }
  return true;
}

void Node::HandleStartTrajectory(
    const std::shared_ptr<::cartographer_ros_msgs::srv::StartTrajectory::Request> request,
    std::shared_ptr<::cartographer_ros_msgs::srv::StartTrajectory::Response> response) {
  (void)response;
  carto::common::MutexLocker lock(&mutex_);
  TrajectoryOptions options;
  if (!FromRosMessage(request->options, &options) ||
      !Node::ValidateTrajectoryOptions(options)) {
    LOG(ERROR) << "Invalid trajectory options.";
    return;
  }
  if (!Node::ValidateTopicName(request->topics, options)) {
    LOG(ERROR) << "Invalid topics.";
    return;
  }

  std::unordered_set<string> expected_sensor_ids;
  const int trajectory_id = AddTrajectory(options, request->topics);
  LaunchSubscribers(options, request->topics, trajectory_id);

  is_active_trajectory_[trajectory_id] = true;
  return;
}

void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  cartographer_ros_msgs::msg::SensorTopics topics;
  topics.laser_scan_topic = kLaserScanTopic;
  topics.multi_echo_laser_scan_topic = kMultiEchoLaserScanTopic;
  topics.point_cloud2_topic = kPointCloud2Topic;
  topics.imu_topic = kImuTopic;
  topics.odometry_topic = kOdometryTopic;

  const int trajectory_id = AddTrajectory(options, topics);
  LaunchSubscribers(options, topics, trajectory_id);
}

void Node::HandleFinishTrajectory(
    const std::shared_ptr<::cartographer_ros_msgs::srv::FinishTrajectory::Request> request,
    std::shared_ptr<::cartographer_ros_msgs::srv::FinishTrajectory::Response> response) {
  (void)response;
  carto::common::MutexLocker lock(&mutex_);
  const int trajectory_id = request->trajectory_id;
  if (is_active_trajectory_.count(trajectory_id) == 0) {
    LOG(INFO) << "Trajectory_id " << trajectory_id << " is not created yet.";
    return;
  }
  if (!is_active_trajectory_[trajectory_id]) {
    LOG(INFO) << "Trajectory_id " << trajectory_id
              << " has already been finished.";
    return;
  }

  ShutdownSubscriber(laser_scan_subscribers_, trajectory_id);
  ShutdownSubscriber(multi_echo_laser_scan_subscribers_, trajectory_id);
  ShutdownSubscriber(odom_subscribers_, trajectory_id);
  ShutdownSubscriber(imu_subscribers_, trajectory_id);

  if (point_cloud_subscribers_.count(trajectory_id) != 0) {
    for (auto& entry : point_cloud_subscribers_[trajectory_id]) {
      LOG(INFO) << "Shutdown the subscriber of [" << entry->get_topic_name() << "]";
    }
    CHECK_EQ(point_cloud_subscribers_.erase(trajectory_id), 1);
  }
  map_builder_bridge_.FinishTrajectory(trajectory_id);
  is_active_trajectory_[trajectory_id] = false;
  return;
}

void Node::HandleWriteAssets(
    const std::shared_ptr<::cartographer_ros_msgs::srv::WriteAssets::Request> request,
    std::shared_ptr<::cartographer_ros_msgs::srv::WriteAssets::Response> response) {
  (void)response;
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.WriteAssets(request->stem);
}

void Node::FinishAllTrajectories() {
  carto::common::MutexLocker lock(&mutex_);
  for (const auto& entry : is_active_trajectory_) {
    const int trajectory_id = entry.first;
    if (entry.second) {
      map_builder_bridge_.FinishTrajectory(trajectory_id);
    }
  }
}
}  // namespace cartographer_ros
