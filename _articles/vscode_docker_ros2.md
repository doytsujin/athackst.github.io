---
layout: text_entry
title: VSCode, Docker, and ROS2
category: Software Development
series: ROS
tags: [vscode, docker, ros2]
summary: How to set up VSCode to develop with ROS2
---

I started out playing with ROS2 by using a docker container.  It was a fast and easy way for me to try out ROS2.  As an avid user of ROS, I naturally wanted to keep up with the new changes being made.  However, it can be difficult to set up, especially for new users.  This guide is intended to be used by people that are familiar with coding and software development, but maybe not ROS.

> **Get the template**
> [https://github.com/athackst/vscode_ros2_workspace](https://github.com/athackst/vscode_ros2_workspace)

## What is ROS2

ROS2 is the next generation of software libraries for robotics development.  Even though ROS stands for Robot Operating System, it's not an operating system like Windows or Ubuntu, instead, it acts as a software development kit for robotics.

## VSCode and Docker

First, I'm going to assume you already have [vscode](https://code.visualstudio.com/) and [docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/) installed.  If not, check out my installation instructions [here](/articles/docker_development.html).

Ok, so now that you have Docker and VSCode installed, let's add some [ros2](https://index.ros.org/doc/ros2/)!

## Adding ROS2

You will need to have the ROS libraries as well as the dependant development packages to develop in ROS2.  I keep an updated version of ROS2 packages on [dockerhub](https://hub.docker.com/repository/docker/athackst/ros2).  I maintain both a base version and a development version of the docker container.

Why two?

The base version has the minimal dependencies needed to run ROS2.  You can use this image for code you want to deploy, but don't want or need the ability to write new code.  The development version is based on the base version and adds all the dependencies you need to compile and create new packages.

### Folder structure

For the following files, I will assume you have the following folder structure.  This structure is very typical for ROS2/vscode development.  

The upper-level directory is the folder you open in VSCode.  It contains the special folders `.devcontainer` and `.vscode` which VSCode uses to load your workspace and preferences.  

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

## Set up the docker development container

### Make the docker file

The first thing you’ll want to do is create a dockerfile for your project. This will let you specify any custom dependencies you have.

example:

```docker
# This is the development environment
FROM athackst/ros2:foxy-dev

# Uncomment this line and add custom packages and dependencies you want to install here.
# sudo apt-get update && sudo apt-get install -y \
# put dependent packages here
# && rm -rf /var/lib/apt/lists/*

# Uncomment these lines to set an entry point for the container.
# COPY entrypoint.sh /setup/entrypoint.sh
# ENTRYPOINT [ "/setup/entrypoint.sh" ]

```

### Set up the development container

You'll need a`.devcontainer` file for VSCode to know how to mount your docker container as a workspace.  I suggest using one like the following:

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
    ],
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

Line by line explaination:

#### context

The context setting is the location where the docker file will be built from.  You can specify the path relative to this file.

#### dockerfile

This is the name of the dockerfile.  It is also specified as a path relative to this file.  To make things easy, you can just put your Dockerfile within the same folder.

#### remoteUser

The user name should match a non-root user inside your docker container.  VSCode will update that users User ID/Group ID to match yours so that files created within the container will have your user id/group

#### runArgs

These are the arguments you want to pass into the `docker run` command.  You can place any argument that is valid to use with [docker run](https://docs.docker.com/engine/reference/commandline/run/)

The ones I find the most useful are:

* Support for gdb (debugging)

    ```javascript
    "--cap-add=SYS_PTRACE",
    "--security-opt", "seccomp=unconfined",
    ```

#### extensions

Extensions are VSCode plugins you'd like to have in the container.  By hosting them in the container, you can be sure that specific ones are installed and configured for everyone.

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

## Set up CPP properties

Since we've added the `cpptools` plugin, you will need to configure the include paths so that VSCode can properly do IntelliSense.

This file is located in the .vscode directory in your workspace

c_cpp_properties.json

```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/foxy/include"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c11",
            "cppStandard": "c++17",
            "intelliSenseMode": "clang-x64"
        }
    ],
    "version": 4
}
```

Now that you have the properties set up, you should be able to tab-complete and follow definitions of objects within your code. Yay!

## Set up tasks

Next, set up your tasks so that you can build and test your code.

.vscode/tasks.json

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "build",
            "type": "shell",
            "command": "colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Debug'"
        },
        {
            "label": "test",
            "type": "shell",
            "command": "colcon test && colcon test-result"
        }
    ]
}
```

## Set up the debugger

Once you can build your code, you may want to run and debug it.  You can do this by adding different configurations to the .vscode/launch.json file

Different debuggers are used for c++ and python

### Python

Since python is a scripting language, it's somewhat easier to set up to debug.

Add the following configuration to .vscode/launch.json

```json
        {
            "name": "Python: Current File",
            "type": "python",
            "request": "launch",
            "program": "${file}",
            "console": "integratedTerminal"
        }
```

### C++

C++ requires the docker container to be set up properly for gdb.  The development image and devcontainer.json file have these set up for you already.

Add the following configuration to .vscode/launch.json

```json
        {
            "name": "(gdb) Launch",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/install/${input:package}/lib/${input:package}/${input:program}",
            "args": [],
            "stopAtEntry": true,
            "cwd": "${workspaceFolder}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ],
            "inputs": [
            {
            "id": "package",
            "type": "promptString",
            "description": "Package name"
            },
            {
            "id": "program",
            "type": "promptString",
            "description": "Program name"
            }
        ]

        }
```

**Note** The location of the "program" you will need to enter the full path of the program you are debugging.

Et Voila!
