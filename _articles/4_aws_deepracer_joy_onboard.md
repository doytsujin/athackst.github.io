---
layout: text_entry
title: Running the gamepad onboard the DeepRacer
category: Robotics
series: DeepRacer
tags: deepracer
---

After I hooked up a [gamepad through ROS](/articles/3_aws_deepracer_joy.html) to the DeepRacer, the next step was to get it loaded directly onto the DeepRacer.  Because what good is a remote control for an RC car if you need to also set up a laptop to use it?

There are two main ways in which the gamepad controller could be loaded into the DeepRacer.  One way is to simply copy over the installation directory on to the DeepRacer, source it, and launch it.  The other is to load a docker container.

If you just want to copy over the installation directory, then type this into the terminal

```bash
rcp -rp deepracer_ws/install deepracer@$DEEPRACER_IP:/opt/deepracer_ws
ssh deepracer@$DEEPRACER_IP
source /opt/deepracer_ws/setup.bash
roslaunch deepracer_joy deepracer_joy.launch
```

This is probably the most straight-forward way of doing it.  You can even edit the `/opt/aws/deepracer/start_ros.sh` file to include your new code so that it will start at launch.

You may also want to use something like `rsync` to avoid re-copying files that haven't changed.

```bash
rsync --progress -r deepracer_ws/install deepracer@$DEEPRACER_IP:/opt/deepracer_ws
```

Neat!

I, however, would like to be able to also load ROS2 into my deepracer eventually. So I want to have docker on my DeepRacer.

## Docker on the DeepRacer

### Install Docker

Follow the Docker CE [installation instructions for ubuntu](https://docs.docker.com/install/linux/docker-ce/ubuntu/)

At the time of this writing, the instructions are:

```bash
sudo apt-get update
# Install packages to allow apt to use a repository over HTTPS:
sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common
# Add Docker’s official GPG key:
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
# Add the stable repository
sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"
# Update the package index
sudo apt-get update
# Install docker-ce
sudo apt-get install docker-ce docker-ce-cli containerd.io
```

Next, add the `deepracer` user to the docker group so you don't have to run commands with sudo

```bash
sudo groupadd docker
sudo usermod -aG docker $USER
```

Log out and back in.

### Build the deployment image

You need the compiled code into a docker container in order to use it (duh).  The code worked in your development container because we had mounted the source and built it within the container.   Since VSCode [doesn't yet support](https://github.com/microsoft/vscode-remote-release/issues/46) multi-stage containers, we need to create a new deployment image to copy the compiled artifacts into.

First, get the code that this article refers to.

```bash
git clone --branch articles/deepracer_joy_onboard --recurse-submodules https://github.com/athackst/deepracer_ws.git
```

Since I've included a deployment image in the `deepracer_ws`, let's build and tag that.

```bash
docker build -f .deploycontainer/Dockerfile . -t deepracer_joy
```

My deployment image _does_ use multi-stage, so you don't need to set up vscode or build it in another image in order to create the deployment image. You just need to have docker installed.

You could build the deployment image on the DeepRacer, but I prefer to do all compilation on my workstation.

In order to facilitate transfer of the docker image between my workstation and the DeepRacer, I set up a local Docker repository.

### Set up a local Docker repository

If you google how to transfer a docker container to another computer, all you'll get is [instructions to create a tar file](https://stackoverflow.com/questions/23935141/how-to-copy-docker-images-from-one-host-to-another-without-using-a-repository) of your container so you can copy it to another computer.

This is **NOT** what you want to do though.  This method is slow and kind of terrible.

What you want to do is host a local docker repository on your computer, and use _that_ to transfer your container to your DeepRacer.  This way, you get transfer speed optimization since it will only download new layers. Huzzah!

And it's *super* easy to set up -- you just run a docker container as your docker registry!

```bash
docker run -d -p 5000:5000 --restart always --name registry registry:2
```

Congratulations, you are now the proud host of a docker registry.

Now all you need to do is build and tag any image with the prefix `localhost:5000` and you'll be able to push your image to your local registry.

```bash
docker build -f .deploycontainer/Dockerfile . -t localhost:5000/deepracer_joy
docker push localhost:5000/deepracer_joy
```

### Connect the DeepRacer to the local docker registry

Awesome, we have built and pushed the joystick image into our local docker registry. Now we need to set up the DeepRacer to be able to pull from it.

Since it's a local registry, it doesn't have the security (https) that remote registries do.  So we will need to add it to the exception list so that we'll be allowed to pull from it.

On the DeepRacer make or edit the `/etc/docker/daemon.json` to have the following line

```json
{ "insecure-registries":["your_hostname.local:5000"] }
```

Replace 'your_hostname' with the hostname of the computer you set up as the local docker registry.

> Note: `your_hostname.local` uses avahi to resolve the appropriate IP address.  This is normally preferred since routers typically dynamically assign IP addresses and this lets you reference a specific device without having to know what the IP address is, and without having to updated it.  If avahi doesn't work on your network for some reason, you can replace `your_hostname.local` with an IP address.

Now you can pull the docker image!

```bash
docker pull your_hostname.local:5000/deepracer_joy
```

## Setup your gamepad

The steps to launch the gamepad controller on the DeepRacer are similar to the [previous article](/articles/3_aws_deepracer_joy.html).

1. Find the device name of your joystick.

    Plug in your gamepad wireless dongle.  I'm using an old school xbox 360 gamepad and receiver.  From what I hear, Ubuntu has gotten a lot better at supporting joystick devices, because I didn't need to install anything in order for the device to show up on the DeepRacer (even for a joystick this old!).

    See if your joystick mounted as a device:

    ```bash
    ls /dev/input
    ```

    and look for a device beginning with `js`.

2. Test your joystick in the container.

    Run the container and check that the device is connected appropriately inside the container.

    ```bash
    docker run --privileged --network=host -it your_hostname.local:5000/deepracer_joy bash
    jstest --normal /dev/input/js0
    ```

    You should see the output change as you press the buttons on your gamepad.

3. Launch the deepracer_joy

    ```bash
    roslaunch deepracer_joy deepracer_joy.launch
    ```

    You should see the deepracer joy launch successfully

    > Note: Mine always says that it can't connect to the device, but it is actually successful at connecting
    >
    > ```bash
    > [ERROR] [1577818476.385114334]: Couldn't open joystick force feedback!
    >```
    >
    > You can test the interface by running `rostopic echo /joy`  to see if there is any output when you press the buttons.

4. Launch at startup.

   The easiest way to make sure the deepracer_joy is always loaded, is by configuring docker to always restart it.

   ```bash
   docker run --network=host --privileged --restart always --name deepracer_joy your_hostname:5000/deepracer_joy bash -c "roslaunch deepracer_joy deepracer_joy.launch teleop_config:=/opt/deepracer_ws/share/deepracer_joy/config/xbox_360.yaml"
   ```
