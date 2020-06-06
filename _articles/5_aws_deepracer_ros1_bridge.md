---
layout: text_entry
title: Running the DeepRacer with ROS2
category: Robotics
series: DeepRacer
tags: [deepracer, ros2]
---

## Interfacing with ROS2

At one point I thought that I would do all new development in the next generation of OSRF software ([ROS2](https://index.ros.org/doc/ros2/) and [Ignition](https://ignitionrobotics.org/home)).  Porting the DeepRacer to use ROS2 may be an exercise in futility, but I thought it might be fun to find out just how painful it is to run a mixed ROS/ROS2 environment.

I was not disappointed (it's super painful).

Below describes how to successfully use the DeepRacer with ROS2.

## How to interface with the DeepRacer

So what does it take to use ROS2 with the DeepRacer?

The DeepRacer currently runs [ROS Kinetic](http://wiki.ros.org/kinetic) with a stack of proprietary software developed by AWS.  So, unlike other porting efforts, the base software stack can't be changed.  However, we can build and run ROS2 applications on top of ROS 1, so long as we bridge the communication between those two APIs.

You can bridge these two networks with a package called [ros1_bridge](https://github.com/ros2/ros1_bridge). At the time of this writing, the [ros1_bridge](https://github.com/ros2/ros1_bridge) does not support dynamic binding.  We must, therefore, compile the ros1_bridge with all of the message types we need for the DeepRacer.  Since the DeepRacer has [quite a few](/articles/3_aws_deepracer_joy.html#deepracer-ros-messages) custom messages, we will need to have those messages compiled in both ROS and ROS2 for the ros1_bridge to be able to bind to them.

The first thing you'll realize when you try to build the ros1_bridge is that it requires a very specific environment.  This makes it tricky to set up correctly, especially for novices, or those just not as familiar with ROS and ROS2.  I've taken the liberty of providing this setup inside of a [docker container](https://github.com/athackst/deepracer_ws/tree/articles/deepracer_ros2_bridge) for your convenience.

The general order of operations for custom message types is:

1. Get the base (ROS) version of the messages
   * If you need custom messages, compile them in a local workspace.
   * Be sure that only the ROS environment has been sourced.
2. In a separate terminal (environment) get the new (ROS2) version of the messages.
   * If you need a custom ROS2 message, compile them in a local workspace.
   * Be sure that only the ROS2 environment has been sourced.
3. In a separate terminal (environment)
   * Source the base ROS version.
   * Source the local ROS workspace.
   * Source the base ROS2 version.
   * Source the local ROS2 workspace.
   * Build the catkin environment.

### Make the ROS messages

This is probably the easiest part of the process, but it's not without some gotchas.  First, the version of ROS that runs on the robot is on a different OS than the latest ROS2 version.  This means that we will need to compile the DeepRacer messages, which work on Kinetic with a later version of ROS, like Melodic.

Because ROS communicates over a TCP networking layer, this isn't a problem.  ROS messages can communicate over any number of operating systems, so long as the messages are exactly the same (they're check-summed at the receiver).  Lucky for us, the DeepRacer uses basic types (which rarely change between versions) and custom messages, which I've separated into [aws_deepracer_messages](https://github.com/athackst/aws_deepracer_msgs).

So let's compile!

You can check out the branch [articles/deepracer_ros2_bridge](https://github.com/athackst/deepracer_ws/tree/articles/deepracer_ros2_bridge) in my [deepracer_ws](https://github.com/athackst/deepracer_ws/) repo to use my workspace

```bash
git clone --branch articles/deepracer_ros2_bridge --recurse-submodules git@github.com:athackst/deepracer_ws.git
```

Now, build the ROS workspace

```bash
source /opt/ros/melodic/setup.bash
/opt/ros/melodic/bin/catkin_make install -C ros_ws --only-pkg-with-deps aws_deepracer_msgs -DCMAKE_INSTALL_PREFIX=ros1_bridge/melodic
```

Let's delve a little into this command:

* `-C ros_ws` : set "ros_ws" as the workspace directory for the ROS messages you want to compile.
* `--only-pkg-with-deps aws_deepracer_msgs` : Only build the "aws_deepracer_msgs" package
* `-DCMAKE_INSTALL_PREFIX` : location to place the compiled sources.  I'm setting this to be in a different location than the default since we're compiling against a different version of ROS1.

### Make the ROS2 message

Next, you're going to want to create the ROS2 version of all the messages.  I've already done this for the DeepRacer in the [eloquent branch](https://github.com/athackst/aws_deepracer_msgs/tree/eloquent) of the [aws_deepracer_msgs](https://github.com/athackst/aws_deepracer_msgs) repo.

Check out my article on [bridging ROS and ROS2](/articles/bridging_ros_ros2.html) for more information on how I ported the messages.

I found the ROS2 is much more strict when it comes to naming conventions than ROS.  This created some differences between the names of elements in the ROS message verses the names of elements in the ROS2 message.  Notably, ROS2 enforces that all elements within the message adhere to the `underscore_style`.

This necessitated the creation of rules files for all of the messages.  A `mapping_rules` file tells the `ros1_bridge` how to map from ROS messages to ROS2 messages.

> Note: You will need to use [my fork of the ros1_bridge](https://github.com/athackst/ros1_bridge) at the time of this writing because I had to modify the upstream repository to properly handle services.

Since I've already ported the [aws_deepracer_msgs](https://github.com/athackst/aws_deepracer_msgs/tree/eloquent) to [ROS2 Eloquent](https://index.ros.org/doc/ros2/Releases/Release-Eloquent-Elusor/), we just need to compile these in the ROS Eloquent environment.

If you're following along using by workspace (the [articles/deepracer_ros2_bridge](https://github.com/athackst/deepracer_ws/tree/articles/deepracer_ros2_bridge) branch in my [deepracer_ws](https://github.com/athackst/deepracer_ws/) repo) then you can build the space with:

```bash
source /opt/ros/eloquent/setup.bash
colcon build --base-paths ros2_ws --merge-install --install-base ro1_bridge/eloquent --packages-select aws_deepracer_msgs
```

Let's break this command down:

* `--base-paths` : directory to search for source packages.
* `--merge-install` : Use the "--install-base" as the install prefix for all packages instead of a package specific subdirectory in the install base.  This will make it easier to refer to this directory in the environment for inclusion in the ros1_bridge.
* `--install-base` : The location to put the compiled outputs.
* `--packages-select aws_deepracer_msgs` : The package to compile.  Note that we're using the "eloquent" branch of this package.

### Set up the ROS Bridge

Now that we have both our ROS messages and ROS2 messages compiled, we can build the `ros1_bridge`!

The first thing you'll need to do is _carefully_ (very _carefully_) set up the environment for the `ros1_bridge` to be able to find and link the two workspaces.

In essence, you want to source the ROS spaces, then the ROS2 spaces. Except for the CMAKE_PREFIX_PATH since the `ros1_bridge` uses the cmake prefix to point to ROS2, and uses the ROS_PACKAGE_PATH to find the ROS spaces.

```bash
source ros1_bridge/melodic/setup.bash
export CMAKE_PREFIX_PATH=
source ros1_bridge/eloquent/local_setup.bash
```

Your workspace environment should now resemble:

```bash
LD_LIBRARY_PATH=/workspaces/deepracer_ws/ros1_bridge_ws/eloquent/lib:/opt/ros/eloquent/lib:/workspaces/deepracer_ws/ros1_bridge_ws/melodic/lib:/opt/ros/melodic/lib
AMENT_PREFIX_PATH=/workspaces/deepracer_ws/ros1_bridge_ws/eloquent:/opt/ros/eloquent
ROS_ETC_DIR=/opt/ros/melodic/etc/ros
COLCON_PREFIX_PATH=/workspaces/deepracer_ws/ros1_bridge_ws/eloquent:/opt/ros/eloquent
ROS_ROOT=/opt/ros/melodic/share/ros
ROS_MASTER_URI=http://localhost:11311
ROS_VERSION=2
ROS_LOCALHOST_ONLY=0
ROS_PYTHON_VERSION=3
PYTHONPATH=/workspaces/deepracer_ws/ros1_bridge_ws/eloquent/lib/python3.6/site-packages:/opt/ros/eloquent/lib/python3.6/site-packages:/opt/ros/melodic/lib/python2.7/dist-packages
ROS_PACKAGE_PATH=/workspaces/deepracer_ws/ros1_bridge_ws/melodic/share:/opt/ros/melodic/share
ROSLISP_PACKAGE_DIRECTORIES=
PATH=/opt/ros/eloquent/bin:/opt/ros/melodic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
PKG_CONFIG_PATH=/workspaces/deepracer_ws/ros1_bridge_ws/melodic/lib/pkgconfig:/opt/ros/melodic/lib/pkgconfig
CMAKE_PREFIX_PATH=/workspaces/deepracer_ws/ros1_bridge_ws/eloquent:/opt/ros/eloquent
```

Now you should be able to compile the `ros1_bridge`

```bash
colcon build --base-paths ros1_bridge --merge-install --install-base bridge --packages-select ros1_bridge --cmake-force-configure
```

The breakdown:

* `--base-paths` : the workspace for the ros1_bridge
* `--merge-install` :  Use the "--install-base" as the install prefix for all packages
* `--install-base bridge` : Install compiled outputs into the "bridge" directory
* `--packages-select ros1_bridge` : Build just the ros1_bridge
* `--cmake-force-configure` : Force cmake to run the configure step

If everything went well, there should be a mapping between the ROS1 and ROS2 message and service types.

You can check this by running the following:

```bash
source bridge/local_setup.bash
ros2 run ros1_bridge dynamic_bridge --print-pairs
```

You should see a print out of pairs that includes the custom message we just build like below

```bash
Supported ROS 2 <=> ROS 1 message type conversion pairs:
  - 'actionlib_msgs/msg/GoalID' (ROS 2) <=> 'actionlib_msgs/GoalID' (ROS 1)
  - 'actionlib_msgs/msg/GoalStatus' (ROS 2) <=> 'actionlib_msgs/GoalStatus' (ROS 1)
  - 'actionlib_msgs/msg/GoalStatusArray' (ROS 2) <=> 'actionlib_msgs/GoalStatusArray' (ROS 1)
  - 'builtin_interfaces/msg/Duration' (ROS 2) <=> 'std_msgs/Duration' (ROS 1)
  - 'builtin_interfaces/msg/Time' (ROS 2) <=> 'std_msgs/Time' (ROS 1)
  - 'ctrl_pkg/msg/ServoCtrlMsg' (ROS 2) <=> 'ctrl_pkg/ServoCtrlMsg' (ROS 1)
  - 'diagnostic_msgs/msg/DiagnosticArray' (ROS 2) <=> 'diagnostic_msgs/DiagnosticArray' (ROS 1)
  - 'diagnostic_msgs/msg/DiagnosticStatus' (ROS 2) <=> 'diagnostic_msgs/DiagnosticStatus' (ROS 1)
  - 'diagnostic_msgs/msg/KeyValue' (ROS 2) <=> 'diagnostic_msgs/KeyValue' (ROS 1)
  - 'geometry_msgs/msg/Accel' (ROS 2) <=> 'geometry_msgs/Accel' (ROS 1)
  - 'geometry_msgs/msg/AccelStamped' (ROS 2) <=> 'geometry_msgs/AccelStamped' (ROS 1)
  - 'geometry_msgs/msg/AccelWithCovariance' (ROS 2) <=> 'geometry_msgs/AccelWithCovariance' (ROS 1)
  - 'geometry_msgs/msg/AccelWithCovarianceStamped' (ROS 2) <=> 'geometry_msgs/AccelWithCovarianceStamped' (ROS 1)
  - 'geometry_msgs/msg/Inertia' (ROS 2) <=> 'geometry_msgs/Inertia' (ROS 1)
  - 'geometry_msgs/msg/InertiaStamped' (ROS 2) <=> 'geometry_msgs/InertiaStamped' (ROS 1)
  - 'geometry_msgs/msg/Point' (ROS 2) <=> 'geometry_msgs/Point' (ROS 1)
  - 'geometry_msgs/msg/Point32' (ROS 2) <=> 'geometry_msgs/Point32' (ROS 1)
  - 'geometry_msgs/msg/PointStamped' (ROS 2) <=> 'geometry_msgs/PointStamped' (ROS 1)
  - 'geometry_msgs/msg/Polygon' (ROS 2) <=> 'geometry_msgs/Polygon' (ROS 1)
  - 'geometry_msgs/msg/PolygonStamped' (ROS 2) <=> 'geometry_msgs/PolygonStamped' (ROS 1)
  - 'geometry_msgs/msg/Pose' (ROS 2) <=> 'geometry_msgs/Pose' (ROS 1)
  - 'geometry_msgs/msg/Pose2D' (ROS 2) <=> 'geometry_msgs/Pose2D' (ROS 1)
  - 'geometry_msgs/msg/PoseArray' (ROS 2) <=> 'geometry_msgs/PoseArray' (ROS 1)
  - 'geometry_msgs/msg/PoseStamped' (ROS 2) <=> 'geometry_msgs/PoseStamped' (ROS 1)
  - 'geometry_msgs/msg/PoseWithCovariance' (ROS 2) <=> 'geometry_msgs/PoseWithCovariance' (ROS 1)
  - 'geometry_msgs/msg/PoseWithCovarianceStamped' (ROS 2) <=> 'geometry_msgs/PoseWithCovarianceStamped' (ROS 1)
  - 'geometry_msgs/msg/Quaternion' (ROS 2) <=> 'geometry_msgs/Quaternion' (ROS 1)
  - 'geometry_msgs/msg/QuaternionStamped' (ROS 2) <=> 'geometry_msgs/QuaternionStamped' (ROS 1)
  - 'geometry_msgs/msg/Transform' (ROS 2) <=> 'geometry_msgs/Transform' (ROS 1)
  - 'geometry_msgs/msg/TransformStamped' (ROS 2) <=> 'geometry_msgs/TransformStamped' (ROS 1)
  - 'geometry_msgs/msg/Twist' (ROS 2) <=> 'geometry_msgs/Twist' (ROS 1)
  - 'geometry_msgs/msg/TwistStamped' (ROS 2) <=> 'geometry_msgs/TwistStamped' (ROS 1)
  - 'geometry_msgs/msg/TwistWithCovariance' (ROS 2) <=> 'geometry_msgs/TwistWithCovariance' (ROS 1)
  - 'geometry_msgs/msg/TwistWithCovarianceStamped' (ROS 2) <=> 'geometry_msgs/TwistWithCovarianceStamped' (ROS 1)
  - 'geometry_msgs/msg/Vector3' (ROS 2) <=> 'geometry_msgs/Vector3' (ROS 1)
  - 'geometry_msgs/msg/Vector3Stamped' (ROS 2) <=> 'geometry_msgs/Vector3Stamped' (ROS 1)
  - 'geometry_msgs/msg/Wrench' (ROS 2) <=> 'geometry_msgs/Wrench' (ROS 1)
  - 'geometry_msgs/msg/WrenchStamped' (ROS 2) <=> 'geometry_msgs/WrenchStamped' (ROS 1)
  - 'inference_pkg/msg/InferResults' (ROS 2) <=> 'inference_pkg/InferResults' (ROS 1)
  - 'inference_pkg/msg/InferResultsArray' (ROS 2) <=> 'inference_pkg/InferResultsArray' (ROS 1)
  - 'nav_msgs/msg/GridCells' (ROS 2) <=> 'nav_msgs/GridCells' (ROS 1)
  - 'nav_msgs/msg/MapMetaData' (ROS 2) <=> 'nav_msgs/MapMetaData' (ROS 1)
  - 'nav_msgs/msg/OccupancyGrid' (ROS 2) <=> 'nav_msgs/OccupancyGrid' (ROS 1)
  - 'nav_msgs/msg/Odometry' (ROS 2) <=> 'nav_msgs/Odometry' (ROS 1)
  - 'nav_msgs/msg/Path' (ROS 2) <=> 'nav_msgs/Path' (ROS 1)
  - 'rcl_interfaces/msg/Log' (ROS 2) <=> 'rosgraph_msgs/Log' (ROS 1)
  - 'rosgraph_msgs/msg/Clock' (ROS 2) <=> 'rosgraph_msgs/Clock' (ROS 1)
  - 'sensor_msgs/msg/BatteryState' (ROS 2) <=> 'sensor_msgs/BatteryState' (ROS 1)
  - 'sensor_msgs/msg/CameraInfo' (ROS 2) <=> 'sensor_msgs/CameraInfo' (ROS 1)
  - 'sensor_msgs/msg/ChannelFloat32' (ROS 2) <=> 'sensor_msgs/ChannelFloat32' (ROS 1)
  - 'sensor_msgs/msg/CompressedImage' (ROS 2) <=> 'sensor_msgs/CompressedImage' (ROS 1)
  - 'sensor_msgs/msg/FluidPressure' (ROS 2) <=> 'sensor_msgs/FluidPressure' (ROS 1)
  - 'sensor_msgs/msg/Illuminance' (ROS 2) <=> 'sensor_msgs/Illuminance' (ROS 1)
  - 'sensor_msgs/msg/Image' (ROS 2) <=> 'sensor_msgs/Image' (ROS 1)
  - 'sensor_msgs/msg/Imu' (ROS 2) <=> 'sensor_msgs/Imu' (ROS 1)
  - 'sensor_msgs/msg/JointState' (ROS 2) <=> 'sensor_msgs/JointState' (ROS 1)
  - 'sensor_msgs/msg/Joy' (ROS 2) <=> 'sensor_msgs/Joy' (ROS 1)
  - 'sensor_msgs/msg/JoyFeedback' (ROS 2) <=> 'sensor_msgs/JoyFeedback' (ROS 1)
  - 'sensor_msgs/msg/JoyFeedbackArray' (ROS 2) <=> 'sensor_msgs/JoyFeedbackArray' (ROS 1)
  - 'sensor_msgs/msg/LaserEcho' (ROS 2) <=> 'sensor_msgs/LaserEcho' (ROS 1)
  - 'sensor_msgs/msg/LaserScan' (ROS 2) <=> 'sensor_msgs/LaserScan' (ROS 1)
  - 'sensor_msgs/msg/MagneticField' (ROS 2) <=> 'sensor_msgs/MagneticField' (ROS 1)
  - 'sensor_msgs/msg/MultiDOFJointState' (ROS 2) <=> 'sensor_msgs/MultiDOFJointState' (ROS 1)
  - 'sensor_msgs/msg/MultiEchoLaserScan' (ROS 2) <=> 'sensor_msgs/MultiEchoLaserScan' (ROS 1)
  - 'sensor_msgs/msg/NavSatFix' (ROS 2) <=> 'sensor_msgs/NavSatFix' (ROS 1)
  - 'sensor_msgs/msg/NavSatStatus' (ROS 2) <=> 'sensor_msgs/NavSatStatus' (ROS 1)
  - 'sensor_msgs/msg/PointCloud' (ROS 2) <=> 'sensor_msgs/PointCloud' (ROS 1)
  - 'sensor_msgs/msg/PointCloud2' (ROS 2) <=> 'sensor_msgs/PointCloud2' (ROS 1)
  - 'sensor_msgs/msg/PointField' (ROS 2) <=> 'sensor_msgs/PointField' (ROS 1)
  - 'sensor_msgs/msg/Range' (ROS 2) <=> 'sensor_msgs/Range' (ROS 1)
  - 'sensor_msgs/msg/RegionOfInterest' (ROS 2) <=> 'sensor_msgs/RegionOfInterest' (ROS 1)
  - 'sensor_msgs/msg/RelativeHumidity' (ROS 2) <=> 'sensor_msgs/RelativeHumidity' (ROS 1)
  - 'sensor_msgs/msg/Temperature' (ROS 2) <=> 'sensor_msgs/Temperature' (ROS 1)
  - 'sensor_msgs/msg/TimeReference' (ROS 2) <=> 'sensor_msgs/TimeReference' (ROS 1)
  - 'shape_msgs/msg/Mesh' (ROS 2) <=> 'shape_msgs/Mesh' (ROS 1)
  - 'shape_msgs/msg/MeshTriangle' (ROS 2) <=> 'shape_msgs/MeshTriangle' (ROS 1)
  - 'shape_msgs/msg/Plane' (ROS 2) <=> 'shape_msgs/Plane' (ROS 1)
  - 'shape_msgs/msg/SolidPrimitive' (ROS 2) <=> 'shape_msgs/SolidPrimitive' (ROS 1)
  - 'std_msgs/msg/Bool' (ROS 2) <=> 'std_msgs/Bool' (ROS 1)
  - 'std_msgs/msg/Byte' (ROS 2) <=> 'std_msgs/Byte' (ROS 1)
  - 'std_msgs/msg/ByteMultiArray' (ROS 2) <=> 'std_msgs/ByteMultiArray' (ROS 1)
  - 'std_msgs/msg/Char' (ROS 2) <=> 'std_msgs/Char' (ROS 1)
  - 'std_msgs/msg/ColorRGBA' (ROS 2) <=> 'std_msgs/ColorRGBA' (ROS 1)
  - 'std_msgs/msg/Empty' (ROS 2) <=> 'std_msgs/Empty' (ROS 1)
  - 'std_msgs/msg/Float32' (ROS 2) <=> 'std_msgs/Float32' (ROS 1)
  - 'std_msgs/msg/Float32MultiArray' (ROS 2) <=> 'std_msgs/Float32MultiArray' (ROS 1)
  - 'std_msgs/msg/Float64' (ROS 2) <=> 'std_msgs/Float64' (ROS 1)
  - 'std_msgs/msg/Float64MultiArray' (ROS 2) <=> 'std_msgs/Float64MultiArray' (ROS 1)
  - 'std_msgs/msg/Header' (ROS 2) <=> 'std_msgs/Header' (ROS 1)
  - 'std_msgs/msg/Int16' (ROS 2) <=> 'std_msgs/Int16' (ROS 1)
  - 'std_msgs/msg/Int16MultiArray' (ROS 2) <=> 'std_msgs/Int16MultiArray' (ROS 1)
  - 'std_msgs/msg/Int32' (ROS 2) <=> 'std_msgs/Int32' (ROS 1)
  - 'std_msgs/msg/Int32MultiArray' (ROS 2) <=> 'std_msgs/Int32MultiArray' (ROS 1)
  - 'std_msgs/msg/Int64' (ROS 2) <=> 'std_msgs/Int64' (ROS 1)
  - 'std_msgs/msg/Int64MultiArray' (ROS 2) <=> 'std_msgs/Int64MultiArray' (ROS 1)
  - 'std_msgs/msg/Int8' (ROS 2) <=> 'std_msgs/Int8' (ROS 1)
  - 'std_msgs/msg/Int8MultiArray' (ROS 2) <=> 'std_msgs/Int8MultiArray' (ROS 1)
  - 'std_msgs/msg/MultiArrayDimension' (ROS 2) <=> 'std_msgs/MultiArrayDimension' (ROS 1)
  - 'std_msgs/msg/MultiArrayLayout' (ROS 2) <=> 'std_msgs/MultiArrayLayout' (ROS 1)
  - 'std_msgs/msg/String' (ROS 2) <=> 'std_msgs/String' (ROS 1)
  - 'std_msgs/msg/UInt16' (ROS 2) <=> 'std_msgs/UInt16' (ROS 1)
  - 'std_msgs/msg/UInt16MultiArray' (ROS 2) <=> 'std_msgs/UInt16MultiArray' (ROS 1)
  - 'std_msgs/msg/UInt32' (ROS 2) <=> 'std_msgs/UInt32' (ROS 1)
  - 'std_msgs/msg/UInt32MultiArray' (ROS 2) <=> 'std_msgs/UInt32MultiArray' (ROS 1)
  - 'std_msgs/msg/UInt64' (ROS 2) <=> 'std_msgs/UInt64' (ROS 1)
  - 'std_msgs/msg/UInt64MultiArray' (ROS 2) <=> 'std_msgs/UInt64MultiArray' (ROS 1)
  - 'std_msgs/msg/UInt8' (ROS 2) <=> 'std_msgs/UInt8' (ROS 1)
  - 'std_msgs/msg/UInt8MultiArray' (ROS 2) <=> 'std_msgs/UInt8MultiArray' (ROS 1)
  - 'stereo_msgs/msg/DisparityImage' (ROS 2) <=> 'stereo_msgs/DisparityImage' (ROS 1)
  - 'trajectory_msgs/msg/JointTrajectory' (ROS 2) <=> 'trajectory_msgs/JointTrajectory' (ROS 1)
  - 'trajectory_msgs/msg/JointTrajectoryPoint' (ROS 2) <=> 'trajectory_msgs/JointTrajectoryPoint' (ROS 1)
  - 'trajectory_msgs/msg/MultiDOFJointTrajectory' (ROS 2) <=> 'trajectory_msgs/MultiDOFJointTrajectory' (ROS 1)
  - 'trajectory_msgs/msg/MultiDOFJointTrajectoryPoint' (ROS 2) <=> 'trajectory_msgs/MultiDOFJointTrajectoryPoint' (ROS 1)
  - 'visualization_msgs/msg/ImageMarker' (ROS 2) <=> 'visualization_msgs/ImageMarker' (ROS 1)
  - 'visualization_msgs/msg/InteractiveMarker' (ROS 2) <=> 'visualization_msgs/InteractiveMarker' (ROS 1)
  - 'visualization_msgs/msg/InteractiveMarkerControl' (ROS 2) <=> 'visualization_msgs/InteractiveMarkerControl' (ROS 1)
  - 'visualization_msgs/msg/InteractiveMarkerFeedback' (ROS 2) <=> 'visualization_msgs/InteractiveMarkerFeedback' (ROS 1)
  - 'visualization_msgs/msg/InteractiveMarkerInit' (ROS 2) <=> 'visualization_msgs/InteractiveMarkerInit' (ROS 1)
  - 'visualization_msgs/msg/InteractiveMarkerPose' (ROS 2) <=> 'visualization_msgs/InteractiveMarkerPose' (ROS 1)
  - 'visualization_msgs/msg/InteractiveMarkerUpdate' (ROS 2) <=> 'visualization_msgs/InteractiveMarkerUpdate' (ROS 1)
  - 'visualization_msgs/msg/Marker' (ROS 2) <=> 'visualization_msgs/Marker' (ROS 1)
  - 'visualization_msgs/msg/MarkerArray' (ROS 2) <=> 'visualization_msgs/MarkerArray' (ROS 1)
  - 'visualization_msgs/msg/MenuEntry' (ROS 2) <=> 'visualization_msgs/MenuEntry' (ROS 1)
Supported ROS 2 <=> ROS 1 service type conversion pairs:
  - 'ctrl_pkg/srv/ActiveStateSrv' (ROS 2) <=> 'ctrl_pkg/ActiveStateSrv' (ROS 1)
  - 'ctrl_pkg/srv/EnableStateSrv' (ROS 2) <=> 'ctrl_pkg/EnableStateSrv' (ROS 1)
  - 'ctrl_pkg/srv/ModelStateSrv' (ROS 2) <=> 'ctrl_pkg/ModelStateSrv' (ROS 1)
  - 'ctrl_pkg/srv/NavThrottleSrv' (ROS 2) <=> 'ctrl_pkg/NavThrottleSrv' (ROS 1)
  - 'diagnostic_msgs/srv/AddDiagnostics' (ROS 2) <=> 'diagnostic_msgs/AddDiagnostics' (ROS 1)
  - 'diagnostic_msgs/srv/SelfTest' (ROS 2) <=> 'diagnostic_msgs/SelfTest' (ROS 1)
  - 'example_interfaces/srv/AddTwoInts' (ROS 2) <=> 'roscpp_tutorials/TwoInts' (ROS 1)
  - 'i2c_pkg/srv/BatteryLevelSrv' (ROS 2) <=> 'i2c_pkg/BatteryLevelSrv' (ROS 1)
  - 'inference_pkg/srv/InferenceStateSrv' (ROS 2) <=> 'inference_pkg/InferenceStateSrv' (ROS 1)
  - 'inference_pkg/srv/LoadModelSrv' (ROS 2) <=> 'inference_pkg/LoadModelSrv' (ROS 1)
  - 'inference_pkg/srv/ModelOptimize' (ROS 2) <=> 'inference_pkg/ModelOptimize' (ROS 1)
  - 'media_pkg/srv/VideoStateSrv' (ROS 2) <=> 'media_pkg/VideoStateSrv' (ROS 1)
  - 'nav_msgs/srv/GetMap' (ROS 2) <=> 'nav_msgs/GetMap' (ROS 1)
  - 'nav_msgs/srv/GetPlan' (ROS 2) <=> 'nav_msgs/GetPlan' (ROS 1)
  - 'nav_msgs/srv/SetMap' (ROS 2) <=> 'nav_msgs/SetMap' (ROS 1)
  - 'sensor_msgs/srv/SetCameraInfo' (ROS 2) <=> 'sensor_msgs/SetCameraInfo' (ROS 1)
  - 'servo_pkg/srv/CarCtrlSrv' (ROS 2) <=> 'servo_pkg/CarCtrlSrv' (ROS 1)
  - 'servo_pkg/srv/GetCalSrv' (ROS 2) <=> 'servo_pkg/GetCalSrv' (ROS 1)
  - 'servo_pkg/srv/GetLedCtrlSrv' (ROS 2) <=> 'servo_pkg/GetLedCtrlSrv' (ROS 1)
  - 'servo_pkg/srv/ServoCalSrv' (ROS 2) <=> 'servo_pkg/ServoCalSrv' (ROS 1)
  - 'servo_pkg/srv/ServoGPIOSrv' (ROS 2) <=> 'servo_pkg/ServoGPIOSrv' (ROS 1)
  - 'servo_pkg/srv/SetLedCtrlSrv' (ROS 2) <=> 'servo_pkg/SetLedCtrlSrv' (ROS 1)
  - 'software_update_pkg/srv/BeginSoftwareUpdateSrv' (ROS 2) <=> 'software_update_pkg/BeginSoftwareUpdateSrv' (ROS 1)
  - 'software_update_pkg/srv/ConsoleUploadModelSrv' (ROS 2) <=> 'software_update_pkg/ConsoleUploadModelSrv' (ROS 1)
  - 'software_update_pkg/srv/GetDeviceInfoSrv' (ROS 2) <=> 'software_update_pkg/GetDeviceInfoSrv' (ROS 1)
  - 'software_update_pkg/srv/OTGLinkStateSrv' (ROS 2) <=> 'software_update_pkg/OTGLinkStateSrv' (ROS 1)
  - 'software_update_pkg/srv/SoftwareUpdateGetStateSrv' (ROS 2) <=> 'software_update_pkg/SoftwareUpdateGetStateSrv' (ROS 1)
  - 'software_update_pkg/srv/SoftwareUpdateStatusSrv' (ROS 2) <=> 'software_update_pkg/SoftwareUpdateStatusSrv' (ROS 1)
  - 'software_update_pkg/srv/VerifyModelReadySrv' (ROS 2) <=> 'software_update_pkg/VerifyModelReadySrv' (ROS 1)
  - 'std_srvs/srv/Empty' (ROS 2) <=> 'std_srvs/Empty' (ROS 1)
  - 'std_srvs/srv/SetBool' (ROS 2) <=> 'std_srvs/SetBool' (ROS 1)
  - 'std_srvs/srv/Trigger' (ROS 2) <=> 'std_srvs/Trigger' (ROS 1)
```

## Run the ros1_bridge on the DeepRacer

The final step is, of course, to run the `ros1_bridge` dockerfile on the DeepRacer and communicate with it over ROS2.

### Set up a docker image

I made a docker image that will go through all of the steps outlined above inside of a multi-stage docker.  

Simply build it, and tag it with your local docker registry name.  See [how to set up a local docker registry](/articles/local_docker_registry.html) if you need help with that step.

```bash
cd deepracer_ws/ros1_bridge_ws
docker build -t localhost:5000/deepracer_ros1bridge -f Dockerfile ../
docker push localhost:5000/deepracer_ros1bridge
```

### Test it

Now the fun part!  Let's send a ROS2 command to the robot to make it move!

First, load the docker image into your DeepRacer

```bash
ssh deepracer@$DEEPRACER_HOST.local
docker pull $YOUR_HOSTNAME:5000/deepracer_ros1bridge
docker run --network=host $YOUR_HOSTNAME:5000/deepracer_ros1bridge
```

Then, you'll want to make a pure ROS2 container to test your commands!

```bash
cd ros2_ws
docker build -t localhost:5000/deepracer_ros2 -f Dockerfile .
docker push localhost:5000/deepracer_ros2
```

Next, try sending a ROS2 command to your DeepRacer using your ROS2 container.  Don't forget to share your network with the container!

```bash
docker run --network=host -it localhost:5000/deepracer_ros2
```

Inside the container, try to send first the ROS service to enable the drive

```bash
ros2 service call /enable_state ctrl_pkg/srv/EnableStateSrv '{is_active: True}'
```

Then send a motion command.

```bash
ros2 topic pub /auto_drive ctrl_pkg/msg/ServoCtrlMsg '{angle: 0, throttle: 1}'
```

Zoom zoom!
