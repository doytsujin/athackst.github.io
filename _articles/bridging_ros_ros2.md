---
layout: text_entry
title: Bridging ROS and ROS2
category: Software Development
tags: [ros, ros2]
series: ROS
summary: Bridging ROS and ROS2 is harder than you would think. Here's how I did it.
---

Running the `ros1_bridge` in a custom environment is tricky, and required me to make a patch to complete.  Here's what I learned:

1. The environment set up required to correctly build the `ros1_bridge` is very specific and requires manually setting (at least one) environment variable.
2. This pain is necessary if you're not interested or can't port all of your packages.
3. Topic support is much better than service support.
4. Building in a docker has huge advantages since you can be explicit on the environment both in build time and run time.  I'm not sure it's possible to run ROS, ROS2, and the bridge using the same environment.

Here, I've outlined all of the steps I took to convert a message package from ROS to ROS2 and build the corresponding `ros1_bridge` to use it.

## Port ROS messages to ROS2

Before we can bridge anything, we'll need to make sure we have ROS and ROS2 versions of any message we'd like to bridge.  The minimal set of things you'll _need_ to change in order to run your ROS messages in ROS2 is the following:

* CMakeLists.txt
* package.xml

### CMakeLists.txt

ROS2 uses a different build system than ROS, so you'll need to change the CMakeLists.txt file to the new build system, which changes some of the CMake macros that were used in ROS.

I've annotated a CMakeLists.txt file I converted with explanations. on each component.

This is the original file:

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(ctrl_pkg)

# Get the information about this package's build time dependencies
find_package(catkin REQUIRED
    COMPONENTS message_generation std_msgs)

# Declare the message files to be built
add_message_files(FILES
    ServoCtrlMsg.msg
)

# Declare the service files to be built
add_service_files(FILES
    ActiveStateSrv.srv
    EnableStateSrv.srv
    ModelStateSrv.srv
    NavThrottleSrv.srv
)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs)

# Declare that this catkin package's runtime dependencies
catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
)
```

This is the ROS2 version:

```cmake
# Update the cmake version to 3.5
cmake_minimum_required(VERSION 3.5)
# cmake_minimum_required(VERSION 2.8.3)

project(ctrl_pkg)

# Add flags to support compiling on windows, since ROS2 supports it
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

# Remove the catkin dependency and add ament_cmake
find_package(ament_cmake REQUIRED)
# find_package(catkin REQUIRED
#    COMPONENTS message_generation std_msgs)
#
# You specify the individual packages that are required for building like this:
find_package(builtin_interfaces REQUIRED)
# Instead of REQUIRED_COMPONENTS std_msgs
find_package(rosidl_default_generators REQUIRED)
# Instead of REQUIRED_COMPONENTS message_generation

# Change message files declaration to a CMake variable instead of the ROS add_message_files(...) custom macro.
# You will need to include the name of the folder since it's no longer implied.
set(msg_files
   "msg/ServoCtrlMsg.msg"
)
# add_message_files(FILES
#     ServoCtrlMsg.msg
# )

# Change service files declaration to a CMake variable instead of the ROS add_service_files(...) custom macro.
# You will need to include the name of the folder here too.
set(srv_files
   "srv/ActiveStateSrv.srv"
   "srv/EnableStateSrv.srv"
   "srv/ModelStateSrv.srv"
   "srv/NavThrottleSrv.srv"
)
# add_service_files(FILES
#     ActiveStateSrv.srv
#     EnableStateSrv.srv
#     ModelStateSrv.srv
#     NavThrottleSrv.srv
# )

# Generate the messages.
# Instead of calling generate_messages, use rosidl_generate_interfaces.
rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    DEPENDENCIES builtin_interfaces
)
# generate_messages(DEPENDENCIES std_msgs)

# Declare dependencies.
# Instead of catkin_package(CATKIN_DEPENDS ...), use ament_export_dependencies.
ament_export_dependencies(rosidl_default_runtime)
# catkin_package(
#     CATKIN_DEPENDS message_runtime std_msgs
# )

# Instead of catkin_package
ament_package()

# Install the mapping rules for parameter name matches within the message.
install(
  FILES rosbridge_mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME})
```

### package.xml

Since ROS2 requires an update to CMakeLists.txt, it's package description also needs to be updated.

Original package.xml

```xml
<?xml version="1.0"?>
<package format="2">
  <name>ctrl_pkg</name>
  <version>0.0.0</version>
  <description>
  This package contains the state control for the car.
  </description>
  <maintainer email="aws-deepracer@amazon.com">AWS DeepRacer</maintainer>
  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>rospy</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>message_generation</build_depend>

  <exec_depend>rospy</exec_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>message_runtime</exec_depend>
</package>
```

And this is what it looks like when it's been updated to ROS2.

```xml
<?xml version="1.0"?>
<!-- update to format 3 -->
<package format="3">
<!-- <package format="2"> -->
  <name>ctrl_pkg</name>
  <version>0.0.0</version>
  <description>
  This package contains the state control for the car.
  </description>
  <maintainer email="aws-deepracer@amazon.com">AWS DeepRacer</maintainer>
  <license>TODO</license>

  <!-- change the buildtool from catkin to ament_cmake -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  <!-- <buildtool_depend>catkin</buildtool_depend> -->

  <!-- use rosidl_default_generators instead of message_generation -->
  <!-- this time, it's a buildtool instead of a dependency -->
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <!-- <build_depend>message_generation</build_depend> -->

  <!-- use builtin_interfaces instead of std_msgs -->
  <build_depend>builtin_interfaces</build_depend>
  <!--  <build_depend>std_msgs</build_depend> -->

  <!-- ignore unneeded deps -->
  <!-- <exec_depend>rospy</exec_depend> -->
  <!-- <exec_depend>roscpp</exec_depend> -->
  
  <exec_depend>builtin_interfaces</exec_depend>
  <!-- <exec_depend>std_msgs</exec_depend>-->

  <!-- use rosidl_default_runtime instead of message_runtime -->
  <exec_depend>rosidl_default_runtime</exec_depend>
  <!-- <exec_depend>message_runtime</exec_depend> -->

  <!-- add to the rosidl_interface_packages group-->
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
    <!-- add custom mapping rules if any variable names changed -->
    <ros1_bridge mapping_rules="rosbridge_mapping_rules.yaml"/>
  </export>
</package>
```

### Update variable names

ROS2 has much more strict linting rules than ROS.  CamelCase variables are no longer allowed inside messages or services.  As a result, I needed to re-name quite a few variables from the original ROS packages when I made the ROS2 versions.

Example:

Original ROS Service

```yaml
bool isActive
---
int32 error
```

ROS2 Service

```yaml
bool is_active
---
int32 error
```

### mapping_rules

The final thing you need to do is set up your mapping_rules file.  You only need this if you changed (or had to change) any of the variable names.

For every service or message in which a variable name was changed, you'll need to tell the `ros1_bridge` how to link the new name to the old name.

These are stored in a yaml file, in the form of a vector of objects, with each item corresponding to a single complete message.

Services and Messages have different key field names.

Example message mapping:

```yaml
- ros1_package_name: "ctrl_pkg"
  ros1_message_name: "ServoCtrlMsg"
  ros2_package_name: "ctrl_pkg"
  ros2_message_name: "ServoCtrlMsg"
  fields_1_to_2:
    angle: "angle"
    throttle: "throttle"
```

Example service mapping:

```yaml
- ros1_package_name: "ctrl_pkg"
  ros1_service_name: "EnableStateSrv"
  ros2_package_name: "ctrl_pkg"
  ros2_service_name: "EnableStateSrv"
  request_fields_1_to_2:
    isActive: "is_active"
  response_fields_1_to_2:
    error: "error"
```

## Build the bridge

As I mentioned previously, using docker for the bridge is advantageous for being able to fully specify the build and run environments for the `ros1_bridge`.  I'll be walking through the creation of a [multi-stage dockerfile](https://docs.docker.com/develop/develop-images/multistage-build/) that will handle building all of the environments and finally creating a run time image from it.

Let's start setting up our ros1_bridge docker image!

I'll be using the [aws_deepracer_msgs](https://github.com/athackst/aws_deepracer_msgs) package as an example.

### Build the ROS messages

Since we're talking about a custom package, I'm going to assume that this package is one that you know how to compile in your local workspace.

Start a docker image that builds your ros messages

```docker
FROM athackst/ros:melodic-dev as ros_builder

WORKDIR /workspaces/ros

RUN mkdir -p src && cd src \
 && git clone --branch kinetic https://github.com/athackst/aws_deepracer_msgs.git \
 && cd ../ \
 && catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic
```

I'm using my development melodic image, which includes just the basic ros packages as well as required build tools.  Be sure to install any dependencies your packages require.  You'll need to repeat these dependencies in the ros1_bridge image, so you might want to install them by using a script.

> Note: I chose melodic so that the base operating system would match my desired destination ros2 version of eloquent.

I've chosen to install my packages in to the `/opt/ros/melodic` directory.  This makes it easier to copy and source in the resulting `ros1_bridge` image.  If you choose to put the ros install directory somewhere else, you'll need to add additional environment variables for that workspace in the `ros1_bridge` image.

### Build the ROS2 messages

Next we'll want to build the ROS2 messages, using a multi-stage dockerfile this can easily be done by starting a new image with `FROM` in the same file.

```docker
FROM athackst/ros2:eloquent-dev as ros2_builder

WORKDIR /workspaces/ros2

RUN mkdir -p src && cd src \
 && git clone --branch eloquent https://github.com/athackst/aws_deepracer_msgs.git \
 && cd ../ \
 && colcon build --merge-install --install-base /opt/ros/eloquent
```

I'm using my development eloquent image to build the files, which includes a basic install of ros2 eloquent and the build tools.

I have also chosen to install the packages into the `/opt/ros/eloquent` directory to make it easier to copy and source in the target `ros1_bridge` image.

### Build the ros1_bridge

Now that we have built the custom ros messages and their counterparts in ros2, we can build the bridge that will link them.

```docker
FROM athackst/ros2:eloquent-dev as ros1_bridge_builder

# install melodic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
  && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
  && apt-get update \
  && apt-get install -y ros-melodic-ros-base \
  && rm -rf /var/lib/apt/lists/*

# Get the ros messages
COPY --from=ros_builder /opt/ros/melodic /opt/ros/melodic

# Get the ros2 messages
COPY --from=ros2_builder /opt/ros/eloquent/ /opt/ros/eloquent

# Deps for the bridge
RUN apt-get update && apt-get install -y \
  # Test deps
  ros-eloquent-demo-nodes-cpp \
  ros-eloquent-launch-testing \
  ros-eloquent-launch-testing-ament-cmake \
  ros-eloquent-launch-testing-ros \
  ros-melodic-rospy-tutorials \
  ros-melodic-roscpp-tutorials \
  # Build deps
  libboost-filesystem-dev \
  libboost-math-dev \
  libboost-regex-dev \
  libboost-signals-dev \
  libboost-thread-dev \
  liblog4cxx-dev \
  && rm -rf /var/lib/apt/lists/*

# Set up the environment
ENV LD_LIBRARY_PATH=/opt/ros/eloquent/lib:/opt/ros/melodic/lib
ENV AMENT_PREFIX_PATH=/opt/ros/eloquent
ENV ROS_ETC_DIR=/opt/ros/melodic/etc/ros
ENV COLCON_PREFIX_PATH=/opt/ros/eloquent
ENV ROS_ROOT=/opt/ros/melodic/share/ros
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_VERSION=2
ENV ROS_LOCALHOST_ONLY=0
ENV ROS_PYTHON_VERSION=3
ENV PYTHONPATH=/opt/ros/eloquent/lib/python3.6/site-packages:/opt/ros/melodic/lib/python2.7/dist-packages
ENV ROS_PACKAGE_PATH=/opt/ros/melodic/share
ENV ROSLISP_PACKAGE_DIRECTORIES=
ENV PATH=/opt/ros/eloquent/bin:/opt/ros/melodic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV PKG_CONFIG_PATH=/opt/ros/melodic/lib/pkgconfig
ENV CMAKE_PREFIX_PATH=/opt/ros/eloquent:/opt/ros/melodic


# Build the bridge
WORKDIR /workspaces/ros1_bridge
RUN mkdir -p src && cd src \
&& git clone --branch eloquent_dev https://github.com/athackst/ros1_bridge.git \
&& cd ../ \
&& colcon build --merge-install --packages-select ros1_bridge --cmake-force-configure --install-base /opt/ros/eloquent

```

> Note, I had to patch the ros1_bridge to correctly handle mapped services. [#241](https://github.com/ros2/ros1_bridge/pull/241)

### Run the ros1_bridge

Next, create the final run time image.

```docker
FROM athackst/ros2:eloquent-base

# install melodic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
  && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
  && apt-get update \
  && apt-get install -y ros-melodic-ros-base \
  && rm -rf /var/lib/apt/lists/*

# Copy the ros outputs
COPY --from=ros1_bridge_builder /opt/ros/melodic /opt/ros/melodic

# Copy the ros2 outputs
COPY --from=ros1_bridge_builder /opt/ros/eloquent/ /opt/ros/eloquent

# Deps for the bridge
RUN apt-get update && apt-get install -y \
  # Test deps
  ros-eloquent-demo-nodes-cpp \
  ros-eloquent-launch-testing \
  ros-eloquent-launch-testing-ament-cmake \
  ros-eloquent-launch-testing-ros \
  ros-melodic-rospy-tutorials \
  ros-melodic-roscpp-tutorials \
  # Build deps
  libboost-filesystem-dev \
  libboost-math-dev \
  libboost-regex-dev \
  libboost-signals-dev \
  libboost-thread-dev \
  liblog4cxx-dev \
  && rm -rf /var/lib/apt/lists/*

# Set up the environment
ENV LD_LIBRARY_PATH=/opt/ros/eloquent/lib:/opt/ros/melodic/lib
ENV AMENT_PREFIX_PATH=/opt/ros/eloquent
ENV ROS_ETC_DIR=/opt/ros/melodic/etc/ros
ENV CMAKE_PREFIX_PATH=/opt/ros/melodic:/opt/ros/eloquent
ENV COLCON_PREFIX_PATH=/opt/ros/eloquent
ENV ROS_ROOT=/opt/ros/melodic/share/ros
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_VERSION=1
ENV ROS_LOCALHOST_ONLY=0
ENV ROS_PYTHON_VERSION=2
ENV PYTHONPATH=/opt/ros/melodic/lib/python2.7/dist-packages:/opt/ros/eloquent/lib/python3.6/site-packages
ENV ROS_PACKAGE_PATH=/opt/ros/melodic/share
ENV ROSLISP_PACKAGE_DIRECTORIES=
ENV PATH=/opt/ros/melodic/bin:/opt/ros/eloquent/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV PKG_CONFIG_PATH=/opt/ros/melodic/lib/pkgconfig

# Run the print-pairs command to check if things are working properly
RUN ros2 run ros1_bridge dynamic_bridge --print-pairs

CMD ["bash", "-c", "ros2 run ros1_bridge dynamic_bridge --bridge-all-pairs"]

```

Build the docker file.  I named mine `ros1_bridge.dockerfile`

```bash
docker build -t ros1_bridge -f ros1_bridge.dockerfile .
```

And then run it.  You'll need to pass in an environment variable that specifies the ROS_MASTER_URI

```bash
docker run -e ROS_MASTER_URI=http://localhost:11311 ros1_bridge
```

Full file [here](https://raw.githubusercontent.com/athackst/deepracer_ws/articles/deepracer_ros2_bridge/dockerfiles/deepracer_ros1_bridge.dockerfile)
