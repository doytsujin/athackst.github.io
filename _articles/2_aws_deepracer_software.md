---
layout: text_entry
title: Digging into ROS on the DeepRacer
category: Robotics
tags: deepracer
---

So you've gotten a deep racer and you want to learn more about the software that's on it.

## ROS Kinetic

![ros kinetic](/assets/img/kinetic.png#left)

At the time of this writing, the Deep Racer runs ROS Kinetic.  The Robot Operating System (ROS) isn't really an operating system, it's more like a software development kid (SDK).  It's comprised of a set of software libraries and tools that help you build robot applications. It's a staple in a lot of university programs on robotics and it is easy to add new algorithms or packages and share them with the world.  Best of all, it's all open source.

ROS Kinetic is the long term support (LTS) version of ROS that works with Ubuntu Xenial 16.04.  It's support ends April 2021, so I hope that Amazon is planning on releasing an update!

## Setting up the DeepRacer

The first thing you'll do when setting up your DeepRacer is to connect it to your computer so you can set up it's wifi.  You should begin by following the [setup instructions](https://aws.amazon.com/deepracer/getting-started/).

After the wifi network has been set up, you can reconnect to the DeepRacer via your browser.  Just type in the IP address of your deep racer into your browser and you'll be presented with the vehicle control and settings.

In order to interface with ROS instead of using the web console, you'll need to enable SSH.  Inside the DeepRacer console, click on settings then SSH.

![deepracer ssh](/assets/img/aws_deepracer_settings.png)

Go ahead and click "enable" and set up your ssh password.

> Note that the default username is deepracer.

After you've enabled SSH, you can now log into your deep racer from the terminal.

```bash
export DEEPRACER_IP=<the ip address>
ssh deepracer@$DEEPRACER_IP
```

When it asks you for a password, enter the one you created in the webpage.

> Note: this password is different from the default password to log into the DeepRacer console webpage.

Now you can start exploring the software that's running

## Exploring the software

The first thing I did was to find where all the software was.  Since ROS by default is installed into the `/opt` directory, that's the first place I went.

```bash
deepracer@amss-hd4i:~$ cd /opt
deepracer@amss-hd4i:/opt$ ls
aws  ros
```

As you can see, we have the base ros installation in `ros` and a custom `aws` folder.  Let's see what's in there!

```bash
deepracer@amss-hd4i:/opt$ cd aws/
deepracer@amss-hd4i:/opt/aws$ ls
deepracer  intel  pyudev
deepracer@amss-hd4i:/opt/aws$ cd deepracer/
deepracer@amss-hd4i:/opt/aws/deepracer$ ls
artifacts            env.sh           password.txt    setup.zsh                    token.txt
brazil-npm-registry  led_values.json  setup.bash      share                        util
calibration.json     lib              setup.sh        software_update_status.json
camera               nginx            _setup_util.py  start_ros.sh
```

As you can see, there's a suspiciously titled `start_ros.sh` script that probably tells us a whole lot about how the software is started.  The contents of this file are:

```bash
source /opt/ros/kinetic/setup.bash
source /opt/aws/deepracer/setup.bash
source /opt/aws/intel/dldt/bin/setupvars.sh
export PYTHONPATH=/opt/aws/pyudev/pyudev-0.21.0/src:$PYTHONPATH
roslaunch deepracer_launcher deepracer.launch
```

Here, we can see that this file:

1. Sources the ROS Kinetic installation
2. Sources the AWS DeepRacer installation
3. Sets up the inference engine
4. Sets up usb devices so they can be accessed through python
5. Launches the software suite.

After sourcing the kinetic and deepracer installations, we can now run ROS commands.

There are a couple of commands that let you introspect the ROS software system.  The first we are going to use is `rosnode list`.  This will tell us all of the ROS programs that are running.

```bash
deepracer@amss-hd4i:/opt/aws/deepracer$ rosnode list
/battery_node
/control_node
/inference_engine
/media_engine
/model_optimizer
/navigation_node
/rosout
/servo_node
/software_update
/web_video_server
/webserver
```

Since there isn't any documentation on the ROS nodes running, I'm going to guess at their function.  

* battery_node : A battery monitoring process
* control_node : The interface to the robot drive train that incorporates calibration
* inference_engine : ML inference engine for controlling the bot
* media_engine : Publishes camera data
* model_optimizer : Loads/compresses your model for the inference engine
* navigation_node : Appears to be inference to control command converter
* rosout : default ros status
* servo_node : Low level servo controller
* software_update : Update software/ml model
* web_video_server : Compress the video stream and serve it on a port
* webserver : Serve the webpage for the robot

If you want to learn more about any particular node, just enter `rosnode info <node_name>`

For example, if you want to know more about the navigation node:

```bash
deepracer@amss-hd4i:/opt/aws/deepracer$ rosnode info /navigation_node
--------------------------------------------------------------------------------
Node [/navigation_node]
Publications:
 * /auto_drive [ctrl_pkg/ServoCtrlMsg]
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /rl_results [unknown type]

Services:
 * /load_action_space
 * /navigation_node/get_loggers
 * /navigation_node/set_logger_level
 * /navigation_throttle


contacting node http://amss-hd4i:38873/ ...
Pid: 2996
Connections:
 * topic: /auto_drive
    * to: /control_node
    * direction: outbound
    * transport: TCPROS
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
```

You can see that it publishes two topics -- `/auto_drive` which is of type `ctrl_pkg/ServoCtrlMsg` and `/rosout` which is the default ros logging topic of type `rosgraph_msgs/Log`.  It also subscribes to one topic -- `/rl_results` which is reporting an `unknown type`.  ROS will sometimes report topic types as `unknown` when no module is publishing them.  

If you want to know more about that topic, you can use `rostopic info`.

```bash
deepracer@amss-hd4i:/opt/aws/deepracer$ rostopic info /rl_results
Type: inference_pkg/InferResultsArray

Publishers: None

Subscribers:
 * /navigation_node (http://amss-hd4i:38873/)
```

 Which tells us the type is actually `inference_pkg/InferResultsArray` and that no one is currently publishing to the topic, although the `navigation_node` is subscribed to it.

 If you want to know more about the message, you can use `rosmsg show` to see the message format

 ```bash
 deepracer@amss-hd4i:/opt/aws/deepracer$ rosmsg show inference_pkg/InferResultsArray
inference_pkg/InferResults[] results
  int32 classLabel
  float32 classProb
  float32 xMin
  float32 yMin
  float32 xMax
  float32 yMax
sensor_msgs/Image img
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  uint32 height
  uint32 width
  string encoding
  uint8 is_bigendian
  uint32 step
  uint8[] data
```

 The `navigation_node` also lists the services which (other than logging) are `/load_action_space` and `/navigation_throttle`.

 Services, like topics, allow you to find out more information. But this time you'll need to use `rosservice info`

```bash
deepracer@amss-hd4i:/opt/aws/deepracer$ rosservice info /load_action_space
Node: /navigation_node
URI: rosrpc://amss-hd4i:38519
Type: inference_pkg/LoadModelSrv
Args: artifactPath taskType preProcessType
```

And also like topics, you can find more about the message format of a service by using `rossrv show`

```bash
deepracer@amss-hd4i:/opt/aws/deepracer$ rossrv show inference_pkg/LoadModelSrv
string artifactPath
int8 taskType
int8 preProcessType
---
int32 error
```

## Connecting to another computer

One of the biggest advantages of ROS is being able to connect to other computers on your network.  To do this, you will need to ROS on your computer, or you can use my [docker image](https://hub.docker.com/repository/docker/athackst/ros) to get started quickly.  

But since ROS uses _ephemeral_ ports to connect nodes, you'll need to first disable the firewall on the DeepRacer with the following command:

```bash
deepracer@amss-hd4i:/opt/aws/deepracer$ sudo ufw disable
```

> Note the password for sudo is the same as your SSH password

Now on your computer you can run my docker image with the following command:

```bash
docker run --network=host -it athackst/ros:kinetic-dev bash
```

This starts my docker image in interactive terminal and shares your network with the image.  You'll want to share your network with the image for the same reason you needed to disable the firewall.

Now all you need to do is set the ROS_MASTER_URI to your DeepRacer.  This is done simply through an environment variable.

```bash
root@x1-carbon:/$ export ROS_MASTER_URI=http://$DEEPRACER_IP:11311
```

Don't forget to add the `11311` for the port the rosmaster host is listening to!

Once the ROS master has been set, you can do things like list, echo, and publish to topics to control your DeepRacer.

```bash
root@x1-carbon:/$ rostopic list
/auto_drive
/calibration_drive
/manual_drive
/rl_results
/rosout
/rosout_agg
/video_mjpeg
```
