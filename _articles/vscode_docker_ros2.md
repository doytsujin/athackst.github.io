---
layout: text_entry
title: VSCode, Docker, and ROS2
category: Software Development
tags: [vscode, docker, ros2]
---

I started out playing with ROS2 by using a docker container.  It was a fast and easy way for me to try out ROS2.  As an avid user of ROS, I naturally wanted to keep up with the new changes being made.  However, it can be difficult to set up, especially for new users.  This guide is intended to be used by people that are familiar with coding and software development, but maybe not ROS.

## What is ROS2

ROS2 is the next generation of software libraries for robotics development.  Even though ROS stands for Robot Operating System, it's not an operating system like Windows or Ubuntu, instead it is acts as an software development kit for robotics.

## VSCode and Docker

First, I'm going to assume you already have [vscode](https://code.visualstudio.com/) and [docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/) installed.  If not, check out my installation instructions [here](/articles/docker_development.html).

Ok, so now that you have docker and vscode installed, let's add some [ros2](https://index.ros.org/doc/ros2/)!

## Adding ROS2

You will need to add both the base ROS libraries and the development packages in order to develop in ROS2.  I keep an updated version of ROS2 packages on [dockerhub](https://hub.docker.com/repository/docker/athackst/ros2).  I maintain both a base version and a development version of the docker container.

Why two?

The base version has the minimal dependencies needed to run ROS2.  You  can use this image for code you want to deploy, but don't want or need the ability to write new code.  The development version is based on the base version, and adds all the dependencies you need to compile and create new packages.

### Folder structure

For the following files, I will assume you have the following folder structure.  This structure is very typical for ROS2/vscode development.  

The upper level directory is the folder you open in vscode.  It contains the special folders `.devcontainer` and `.vscode` which vscode uses to load your workspace and preferences.  

ROS2 code is located inside the src directory, organized as packages within folders.

```text
--[ base_folder
    |--[ .devcontainer
        |-- devcontainer.json
        |-- Dockerfile
    |--[ .vscode
    |--[ src
        |-- <ros packages>
```

### Make the docker file

In order to use the docker container in VSCode, you will need to make a dockerfile and update it to install your own dependencies.

For this reason, I decided to share this as a file, instead of posting it as a container on dockerhub.

```docker
# This is the development environment
FROM athackst/ros2:dashing-dev

# Uncomment this line and add custom packages and dependencies you want to install here.
#sudo apt-get update && sudo apt-get install -y \
# put dependent packages here
# && rm -rf /var/lib/apt/lists/*

# This Dockerfile adds a non-root 'vscode' user with sudo access. However, for Linux,
# this user's GID/UID must match your local user UID/GID to avoid permission issues
# with bind mounts. Update USER_UID / USER_GID if yours is not 1000. See
# https://aka.ms/vscode-remote/containers/non-root-user for details.
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user to use if preferred - see https://aka.ms/vscode-remote/containers/non-root-user.
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # [Optional] Add sudo support for the non-root user
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    # Cleanup
    && rm -rf /var/lib/apt/lists/*

# The entrypoint for the container to source the environment
COPY entrypoint.sh /setup/entrypoint.sh
ENTRYPOINT [ "/setup/entrypoint.sh" ]

```

### Set up the devcontainer

You'll need a devcontainer file in order for vscode to know how to mount your docker container as a workspace.  I suggest using one like the following:

```javascript
// See https://aka.ms/vscode-remote/devcontainer.json for format details.
{
    "context": "../",
    "dockerFile": "Dockerfile",
    // This will launch the container as a non-root user
    "remoteUser" : "vscode",
    "runArgs": [
        // This will allow you to use a ptrace-based debugger like C++, Go, and Rust.
        "--cap-add=SYS_PTRACE",
        "--security-opt", "seccomp=unconfined",
        // This lets you mount your ssh credentials.  
        // In order to work with all platforms (including windows)
        // you have to mount your credentials and then set them
        // up using the postCreateCommand below
        "-v", "${env:HOME}${env:USERPROFILE}/.ssh:/root .ssh-localhost:ro"
    ],
    // This allows all platforms to use the host ssh credentials
    "postCreateCommand": "sudo cp -r /root/.ssh-localhost ~ &&  sudo chown -R $(id -u):$(id -g) ~/.ssh-localhost && mv ~  .ssh-localhost ~/.ssh && chmod 700 ~/.ssh && chmod 600 ~  .ssh/*",
    // These are the extensions I like to use with ROS2
    "extensions": [
        "ms-azuretools.vscode-docker",
        "ms-python.python",
        "ms-vscode.cpptools",
        "twxs.cmake",
        "ms-vscode.cmake-tools",
        "ms-iot.vscode-ros",
        "smilerobotics.urdf",
        "yzhang.markdown-all-in-one"
    ]
}
```

#### context

The context setting is the location where the docker file will be built from.  You can specify the path relative to this file.

#### dockerfile

This is the name of the dockerfile.  It is also specified as a path relative to this file.  To make things easy, you can just put your Dockerfile within the same folder.

#### remoteUser

The user name should match a non-root user inside your docker container.  VSCode will update that users User ID/Group ID to match yours so that files created within the container will have your user id/group

#### runArgs

These are the arguments you want to pass in to the `docker run` command.  You can place any argument that is valid to use with [docker run](https://docs.docker.com/engine/reference/commandline/run/)

The ones I find the most useful are:

* Support for gdb (debugging)

    ```javascript
    "--cap-add=SYS_PTRACE",
    "--security-opt", "seccomp=unconfined",
    ```

* SSH credentials.  This will add [ssh credentials](https://code.visualstudio.com/docs/remote/containers#_using-ssh-keys) to your container without saving them inside the container.  This somewhat lengthy entry is cross-platform compatible, so you can use the same devcontainer on linux and windows.

    ```javascript
    "-v", "${env:HOME}${env:USERPROFILE}/.ssh:/root .ssh-localhost:ro"
    ```

    You'll also need to set up the corresponding `postCreateCommand` to give the .ssh credentials the right file permissions within your container

    ```javascript
    "postCreateCommand": "sudo cp -r /root/.ssh-localhost ~ &&  sudo chown -R $(id -u):$(id -g) ~/.ssh-localhost && mv ~  .ssh-localhost ~/.ssh && chmod 700 ~/.ssh && chmod 600 ~  .ssh/*"
    ```

#### extensions

Extension are vscode plugins you'd like to have in the container.  By hosting them in the container, you can be sure that specific ones are installed and configured for everyone.

I like using the following extensions with ROS2.

```javascript
    "extensions": [
        "ms-azuretools.vscode-docker",
        "ms-python.python",
        "ms-vscode.cpptools",
        "twxs.cmake",
        "ms-vscode.cmake-tools",
        "ms-iot.vscode-ros",
        "smilerobotics.urdf",
        "yzhang.markdown-all-in-one"
    ]
```

Et voila!

You can now open your folder inside a docker container!
