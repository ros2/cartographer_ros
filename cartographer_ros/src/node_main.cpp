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

#include <chrono>
#include <cmath>

#include "absl/memory/memory.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.hpp"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

// No trajectory or laser subscription exists while this runs. The TF listener
// has its own thread; this executor only receives candidate heading messages.
bool InitializeMapHeading(const rclcpp::Node::SharedPtr& node,
                          const tf2_ros::Buffer& tf_buffer,
                          const NodeOptions& options,
                          TrajectoryOptions* trajectory_options) {
  if (!std::isfinite(options.imu_heading_offset_rad) ||
      !std::isfinite(options.imu_heading_timeout_sec) ||
      options.imu_heading_timeout_sec <= 0. ||
      !std::isfinite(options.imu_heading_max_age_sec) ||
      options.imu_heading_max_age_sec <= 0.) {
    LOG(ERROR) << "IMU heading timeout/max age must be positive and finite; "
                  "heading offset must be finite.";
    return false;
  }
  sensor_msgs::msg::Imu::ConstSharedPtr latest;
  auto subscription = node->create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS().keep_last(1),
      [&latest](sensor_msgs::msg::Imu::ConstSharedPtr msg) { latest = msg; });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  LOG(INFO) << "Waiting for initial ENU IMU heading on "
            << subscription->get_topic_name() << "; mapping has not started.";
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(options.imu_heading_timeout_sec);
  std::string reason = "no IMU message received";
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_once(std::chrono::milliseconds(100));
    if (!latest) continue;
    if (latest->header.frame_id.empty() ||
        latest->orientation_covariance[0] == -1.) {
      reason = "IMU orientation unavailable or frame_id empty";
      continue;
    }
    const rclcpp::Time stamp(latest->header.stamp, node->get_clock()->get_clock_type());
    if (stamp.nanoseconds() == 0 ||
        std::abs((node->now() - stamp).seconds()) > options.imu_heading_max_age_sec) {
      reason = "IMU timestamp is stale or clocks disagree";
      continue;
    }
    const auto& q = latest->orientation;
    Eigen::Quaterniond world_from_imu(q.w, q.x, q.y, q.z);
    if (!world_from_imu.coeffs().allFinite() ||
        std::abs(world_from_imu.norm() - 1.) > 0.01) {
      reason = "IMU orientation quaternion is invalid";
      continue;
    }
    world_from_imu.normalize();
    try {
      const auto tracking_from_imu = ToRigid3d(tf_buffer.lookupTransform(
          trajectory_options->tracking_frame, latest->header.frame_id, stamp,
          rclcpp::Duration::from_seconds(0.)));
      const Eigen::Quaterniond world_from_tracking =
          world_from_imu * tracking_from_imu.rotation().conjugate();
      // Yaw is undefined when the tracking frame's x-axis is vertical.
      const auto forward = world_from_tracking * Eigen::Vector3d::UnitX();
      if (forward.head<2>().norm() < 1e-6) {
        reason = "tracking frame has no horizontal heading";
        continue;
      }
      const double yaw = std::atan2(forward.y(), forward.x()) +
                         options.imu_heading_offset_rad;
      const auto pose = cartographer::transform::Embed3D(
          cartographer::transform::Rigid2d::Rotation(yaw));
      *trajectory_options->trajectory_builder_options.mutable_initial_map_pose() =
          cartographer::transform::ToProto(pose);
      LOG(INFO) << "Initial map heading: " << yaw
                << " rad; starting the first trajectory at this heading.";
      return true;
    } catch (const tf2::TransformException& error) {
      reason = error.what();
    }
  }
  LOG(ERROR) << "Initial IMU heading not established; no trajectory started: "
             << reason;
  return false;
}

bool Run() {
  rclcpp::Node::SharedPtr cartographer_node = rclcpp::Node::make_shared("cartographer_node");
  constexpr double kTfBufferCacheTimeInSeconds = 10.;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer =
      std::make_shared<tf2_ros::Buffer>(
        cartographer_node->get_clock(),
        tf2::durationFromSec(kTfBufferCacheTimeInSeconds),
        cartographer_node);

  std::shared_ptr<tf2_ros::TransformListener> tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  if (node_options.initialize_map_from_imu) {
    if (!FLAGS_load_state_filename.empty() ||
        !FLAGS_start_trajectory_with_default_topics ||
        !node_options.map_builder_options.use_trajectory_builder_2d() ||
        trajectory_options.trajectory_builder_options.has_initial_trajectory_pose() ||
        trajectory_options.trajectory_builder_options.has_initial_map_pose()) {
      LOG(ERROR) << "initialize_map_from_imu requires automatic startup of a "
                    "fresh 2D map, without a saved state or another initial pose.";
      return false;
    }
    if (!InitializeMapHeading(cartographer_node, *tf_buffer, node_options,
                              &trajectory_options)) {
      return false;
    }
  }

  auto map_builder =
    cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  auto node = std::make_shared<cartographer_ros::Node>(
    node_options, std::move(map_builder), tf_buffer, cartographer_node,
    FLAGS_collect_metrics);
  if (!FLAGS_load_state_filename.empty()) {
    node->LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  if (FLAGS_start_trajectory_with_default_topics) {
    node->StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  rclcpp::spin(cartographer_node);

  node->FinishAllTrajectories();
  node->RunFinalOptimization();

  if (!FLAGS_save_state_filename.empty()) {
    node->SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
  return true;
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  // Init rclcpp first because gflags reorders command line flags in argv
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  const bool success = cartographer_ros::Run();
  ::rclcpp::shutdown();
  return success ? 0 : 1;
}
