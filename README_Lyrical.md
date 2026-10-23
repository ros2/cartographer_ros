Back to [package README](https://github.com/slgrobotics/cartographer_ros/blob/lyrical/README.rst)

-----------------------------

# ROS 2 Lyrical compatibility changes

> **Note:**
> - if *Cartographer* is not present in your `/opt/ros/${ROS_DISTRO}/share/cartographer_ros` you can build it as described below.
> - ROS2 *Lyrical* seems to distribute `/opt/ros/${ROS_DISTRO}/share/cartographer_ros_msgs` as binaries, but not the rest.
> - put COLCON_IGNORE file in directories which you don't want to be built.
> - see https://github.com/slgrobotics/articubot_one/wiki/Jazzy-to-Lyrical-migration for more.


## Cartographer (core library)

- Replaced legacy thread annotations (`GUARDED_BY`, `LOCKS_EXCLUDED`, and related macros) with their `ABSL_*` equivalents and explicitly included `absl/base/thread_annotations.h` for compatibility with the installed Abseil version.
- Raised the minimum CMake version to **3.16**.
- Switched Boost discovery to `find_package(Boost CONFIG ...)` in both the build and exported package configuration to support modern Boost/CMake packages.

## cartographer_ros (including messages and RViz)

- Applied the same Abseil annotation updates to the ROS integration and RViz plugin.
- Raised the minimum CMake version to **3.16** in the ROS, messages, RViz, and documentation packages.
- Moved `offline_node.cpp` from the shared ROS library into `cartographer_offline_node`, preventing duplicate gflags registrations (such as `collect_metrics`) when starting the live node. Added explicit URDF linkage to the offline executable and retained the header compatibility definition for older ROS distributions.

## Build and launch notes

Both repositories must be present under the workspace's `src` directory. Build and source the overlay:

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/slgrobotics/cartographer.git -b lyrical
git clone https://github.com/slgrobotics/cartographer_ros.git -b lyrical

# At this point you may want to suppress building some of the packages:
# cp cartographer_ros/docs/COLCON_IGNORE cartographer_ros/cartographer_ros_msgs/.  # because binary package is present
# cp cartographer_ros/docs/COLCON_IGNORE cartographer_ros/cartographer_rviz/.      # on a "headless" robot with ROS2 Base

cd ~/robot_ws
colcon build --allow-overriding cartographer_ros_msgs
source install/setup.bash
```
## Headless Raspberry Pi 5 build notes

Cartographer and `cartographer_ros` were built successfully on a Raspberry Pi 5 running Ubuntu Server 26.04.1 and ROS 2 Lyrical (ROS Base).

### Package selection

Keep both the `cartographer` core repository and this `cartographer_ros` repository under the workspace's `src` directory. On the headless robot, build only the core library and ROS integration:

- Skip `cartographer_rviz`: RViz runs on a separate computer, so its plugin is unnecessary on the robot.
- Skip `cartographer_ros_msgs`: use the matching binary package already installed under `/opt/ros/lyrical`.

From the workspace's `src` directory:

```bash
cd ~/robot_ws/src
cp cartographer_ros/docs/COLCON_IGNORE cartographer_ros/cartographer_ros_msgs/
cp cartographer_ros/docs/COLCON_IGNORE cartographer_ros/cartographer_rviz/
```

These marker files tell colcon to ignore the two packages.

Build from the workspace root:

```bash
cd ~/robot_ws
source /opt/ros/lyrical/setup.bash
colcon build --packages-up-to cartographer_ros
source install/setup.bash
```

There is no need for `--allow-overriding cartographer_ros_msgs` when the source messages package is ignored.

### Compiler *stderr* output

The successful Raspberry Pi builds produced substantial *stderr* output:

- **GCC ARM ABI notes:** Eigen rotation types trigger messages about parameter passing changing in GCC 10.1 when C++17 is enabled. These are informational notes, not compilation errors.

> ABI (Application Binary Interface) defines how compiled functions pass arguments and return values. GCC may report a historical ARM calling-convention correction from GCC 10.1 when compiling Eigen rotation types.
> These are informational notes, not build errors. Build Cartographer and its ROS integration with the same current compiler to avoid this particular compiler-version mismatch.

Colcon prints stderr even when a package succeeds. To optionally suppress GCC's ABI notes on the Pi:

```bash
colcon build --packages-up-to cartographer_ros --cmake-args -DCMAKE_CXX_FLAGS=-Wno-psabi
```

This sets `CMAKE_CXX_FLAGS`; preserve any other custom flags if you already use them. It does not suppress other warnings.

## Ground plane and an elevated IMU

When using IMU data, Cartographer requires its tracking frame to be colocated with the IMU:
```
tracking_frame = "imu_link",
```
To anchor the ground plane at base_footprint, independently of the EKF’s choice of odom z=0:
```
publish_frame_projected_to_2d = true,
projection_frame = "base_footprint",
```
Choose TF ownership:
- With an EKF publishing `odom → base_link`:
  - use `published_frame = "odom"` and `provide_odom_frame = false`
- With Cartographer providing odometry TF:
  - use `published_frame = "base_footprint"` and `provide_odom_frame = true`
  - disable competing odometry TF publishers.

## Initialize a fresh map from IMU heading

The live `cartographer_node` can initialize trajectory 0's local-to-map transform
before creating its first submap. Cartographer now can avoid a zero-heading startup trajectory.

Add these fields to the Lua `options` [table](https://github.com/slgrobotics/articubot_one/blob/lyrical/robots/seggy/config/cartographer_lds_2d.lua#L31):

```lua
initialize_map_from_imu = true,
imu_heading_offset_rad = 0.0,
imu_heading_timeout_sec = 30.0,
imu_heading_max_age_sec = 1.0,
```

> **Note:** This mode is opt-in and supports **fresh 2D maps only**. It rejects loaded state,
> manual trajectory startup, and conflicting initial poses. For localization in a
> saved map, leave it disabled and supply a pose in that map's coordinates through
> the existing trajectory-start service. The offline node does not use this live
> heading initialization mode.

Keep automatic trajectory startup enabled (the default). Remap `imu` to the
orientation-bearing IMU topic, for example `imu:=imu/data`. This startup
subscription is independent of `TRAJECTORY_BUILDER_2D.use_imu_data`: heading can
be used even when IMU acceleration/angular velocity are not used for SLAM.

Keep the robot stationary
until the log reports **Initial map heading**. Only then are the trajectory and
sensor subscriptions created; scans received while waiting are not inserted.
The initial map origin remains the startup tracking-frame origin.

The IMU must provide a valid normalized orientation quaternion in an ENU world
frame, where zero yaw points East and positive yaw turns toward North. A gyro-only
orientation with arbitrary startup yaw cannot provide compass alignment.

The TF from the IMU message's frame to the configured `tracking_frame` must be available
at the IMU timestamp; its mounting rotation is included in the calculation.
`imu_heading_offset_rad` rotates the IMU world heading into the desired map
heading, in radians. An offset alone does not convert a NED quaternion into ENU.

Unavailable orientations (`orientation_covariance[0] == -1`), malformed
quaternions, stale/future timestamps beyond the configured age limit, missing TF,
and a tracking x-axis with no horizontal projection do not start mapping.
If no valid heading arrives before the timeout, the process exits unsuccessfully
without creating a trajectory. It does not fall back to zero heading.

-----------------------------

Back to [package README](https://github.com/slgrobotics/cartographer_ros/blob/lyrical/README.rst)
