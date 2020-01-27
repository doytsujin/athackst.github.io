---
layout: text_entry
title: Controlling the DeepRacer with a gamepad
category: Robotics
tags: deepracer
---

The first thing I wanted to do when I got my DeepRacer was drive it around.  It comes with a web interface where you can run it in "manual" mode, but I wanted to use a gamepad controller. (By default the web interface is only controllable via mouse.)

Since the DeepRacer comes pre-loaded with [ROS Kinetic](http://wiki.ros.org/kinetic), I thought it would be a good idea to use the [ROS joystick package](http://wiki.ros.org/joy) which converts raw joystick driver data into a ROS message and [joy_teleop](http://wiki.ros.org/joy_teleop) to convert the joystick data into the right ROS message.

## ROS Messages

The real power of ROS lies in the standardization of messages.  With ROS there was suddenly a common messaging system and interfaces and it became much easier for researchers at different universities to share the code they developed for their research with each other.  They were also freed to focus on the portion of the robotics problem they were actually interested in researching instead of spending all of their time on the non-trivial task of bringing up an entire robotic software system.  

So it was an odd choice for Amazon to eschew all standard messages (except for video?) when they released the DeepRacer.  Also odd is that I couldn't find an open source version of the messages on their github page, making it much more difficult to hack into the system.

Since message headers are needed in order for other software to interface with your system, it's somewhat customary in ROS to separate messages into their own package.  That way, outside contributors can both introspect and publish to your interfaces without having to have the rest of your code or dependencies.  Amazon doesn't appear to have done this.  They seem to have interleaved the new messages they've created in with the packages they created.

I'm going to assume that Amazon ignorance of ROS standards/best practices is to blame here.  So I've created a package of the [deepracer messages](https://github.com/athackst/aws_deepracer_msgs) for you (so you don't have to).  

You're welcome Amazon.

## Workspace setup

I've also done all the work to get you up and running, so long as you have

* [vscode](https://code.visualstudio.com/)
* [docker](https://www.docker.com/products/container-runtime)
* [set up the DeepRacer for remote connection](/articles/2_aws_deepracer_software.html#connecting-to-another-computer)

you should be able to use my [deepracer_ws](https://github.com/athackst/deepracer_ws) as an IDE for the DeepRacer.  For more information on how I set up my workspaces, check my article on [docker development](/articles/docker_development.html).

```bash
git clone --branch articles/deepracer_joy --recurse-submodules https://github.com/athackst/deepracer_ws.git
```

This will give you my version of my workspace and code that I used for this article.  In it contains a docker container and my vscode settings.

> Note: I ran this on my laptop and connected to the deepracer through the network.

### Docker container

The docker container starts with a development version of ros kinetic.  I maintain a copy of the current versions of ROS on [dockerhub](https://hub.docker.com/u/athackst).  These are optimized versions of ROS that contain the bare minimum needed for development with ROS.  They are also tagged with a date, so you can always peg your development to a specific version.

The Dockerfile for the workspace then adds a non-root user with sudo access and sets up the working environment for the user.

In order to connect the docker container to the DeepRacer and use a gamepad as input, you'll need to run it with a couple of special settings.  These are set for you in the devcontainer.json file.

#### Privileged

You'll need to run the container in "privileged" mode by setting the `--privileged` tag in the run arguments for the container.  This will allow you to access the gamepad.

> Note for windows users:  the --privileged tag is not supported in docker for windows.  You will need to develop through a VM, or run directly on the deepracer to connect the gamepad for testing.

#### Network

You'll also want to share the host network since ROS communicates over ephemeral ports by setting `--network=host` tag in the run arguments for the container.

### VSCode Tasks

I've added a couple of tasks you can use to help use the workspace.  Tasks are how I [super charge my development](/articles/vscode_tasks.html).

1. build

   The standard build task.  Just runs the make command on your workspace.

   ```bash
   catkin_make
   ```

2. build (debug)

   Build the code in debug mode.  Useful for connecting to debuggers. (You won't need this one for this since we just configured already existing packages)

   ```bash
   catkin_make -DCMAKE_BUILD_TYPE=Debug
   ```

3. install

   The standard install task.  It's a good practice to install your workspace

   ```bash
   catkin_make install
   ```

4. launch

   This will launch the deepracer_joy with your settings.  Note this depends on the "source" tasks, which is convenient because re-sourcing your workspace after you make a change is an often overlooked step.

   ```bash
   source install/setup.bash 2> /dev/null || source devel/setup.bash && roslaunch deepracer_joy deepracer_joy.launch
   ```

## DeepRacer ROS messages

The first thing you need to do to interface with a ROS topic is be able to link against the headers and/or python bindings. I copied all of the `.msg` and `.srv` files from my DeepRacer into aws_deepracer_msgs, re-creating each package so that they can be compiled and linked against locally.  

There are six packages with custom messages or services:

1. ctrl_pkg
2. i2c_pkg
3. inference_pkg
4. media_pkg
5. servo_pkg
6. software_update_pkg

Of these, the only one the gamepad needs to interface with is the `ctrl_pkg`.

## DeepRacer joystick control

Knowing ROS has joystick teleoperation packages, I figured it wouldn't be too difficult to set up my DeepRacer to use one of those.  The two packages I chose to use is the `joy` package and the `joy_teleop` package.

### Setting up your gamepad

The first thing you'll want to do to set up your gamepad is verify that you have connection to it.  You can do this with the jstest package (which is installed into the docker container).

1. Find the name of your joystick/gamepad

   ```bash
   ls /dev/input
   ```

   Mine is `/dev/input/js0`

2. Run jstest to make sure your joystick/gamepad can appropriately connect to your docker container

   ```bash
   sudo jstest --normal /dev/input/js0
   ```

   You should see a line of axis and button numbers that change when you press the buttons on your gamepad.

    ![jstest](/assets/img/jstest_gamepad.gif)

3. Edit the launch file to update the joystick device name (optionally, you can specify it in the command line in tasks.json)

   ```xml
    <launch>
        <arg name="joystick_device" default="/dev/input/js0" />
   ```

4. Edit the configuration file for your gamepad/joystick

   If you're using a different gamepad/joystick, you may need to create a new configuration file.

   Using jstest, you can see the axis and button numbers of your device.  In the [config file](https://github.com/athackst/deepracer_joy/blob/master/config/logitech_dual_action.yaml) you can edit the axis and button indexes to match your desired control

   ```yaml
   teleop:
    drive:
        type: topic
        message_type: ctrl_pkg/ServoCtrlMsg
        topic_name: /manual_drive
        deadman_buttons: [4]
        axis_mappings:
        - axis: 1
            target: throttle
            scale: 0.5
        - axis: 0
            target: angle
            scale: 0.5
    deadman:
        type: service
        service_name: /enable_state
        service_request:
        isActive: true
        buttons: [4]
    ```

    > Note: in order to control the vehicle you have to both call the /enable_state service and set isActive to true _and_ provide a command input on the `manual_drive` topic.

    Unfortunately, the joy_teleop package only has hooks for "buttons" which acts like a "button_press" event and doesn't have a way to react to "button_release" events.  It would be better if all deadman functionality used the `/enable_state` service (as I'm sure was intended) instead of listing it both as a service call and in deadman_buttons.

    I've always found ROS packages like this.  They get you 90% to what you want to develop, but still require some additional tweaking to get all the functionality you want.

    Another thing to not is that the deepracer command is 'throttle' and 'angle' which means that mapping both to a single control stick on a gamepad won't work the way you think it will.  As you move the joystick toward a pure turning motion, the forward velocity goes to zero.  This means that your deepracer won't be able to turn much unless you either publish out the kinematic conversion or you map the fields to different control sticks.  For now, I've just mapped the axis.

5. Build and source the code

   ```bash
   catkin_make install
   source install/setup.bash
   ```

6. Set your ros master uri to your DeepRacer IP address and launch the driver

   ```bash
   export ROS_MASTER_URI=http://$DEEPRACER_IP:11311
   export ROS_IP=<your ip address>
   roslaunch deepracer_joy deepracer_joy.launch
   ```

   You should now be able to hold down your deadman button and move the gamepad controller around to control your DeepRacer.

    ![deepracer joystick control](/assets/img/deepracer_joy.gif)

> Note: If you want to run this onboard, checkout the next post on [running the deepracer_joy package onboard](/articles/4_aws_deepracer_joy_onboard.html)
>
> This is only tested as working with Ubuntu 18.04 as the host.  In theory this should work with other flavors of linux based systems as well.  This has not been tested on MacOsx or Windows systems and they may require additional settings.
