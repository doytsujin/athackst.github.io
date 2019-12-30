---
layout: text_entry
title: "My Secret: VSCode and Docker"
category: Software Development
tags: [vscode, docker]
---

![vscode](/assets/img/IMG_5118.jpg#left)

Lately, I've been experimenting with using Docker for all of my software development.

Yes. All of it.

In the past, I've only really used docker in a professional setting. Using it to capture and share somewhat esoteric code dependencies ensures a developer doesn't become derailed on an errant system update. It also helps new hires get into the code base faster since they don't have to download and install a million different dependencies of a specific version. It also helps companies support (or stage upgrades on) different workstation builds, since hosting the code in a docker container decouples it from the host operating system.  

But this all seems like flagrant overkill for at home, for-fun coding.  

You only have one workstation you care about. Plus, it's not like you have to on-board anyone else into your personal code base. So why would you ever care about using Docker for personal code development?  

I've found that even with my personal code development, I often want to have experimental spaces where I can download different libraries just to play around with them. I don't really want to have them installed on my system because:

1. they can be hard to remove (especially if it was some beta that didn't *quite* have it's uninstall hooks done correctly)
2. they may conflict with some other library I have on my system and
3. I'd have to continually keep it up to date and synced with compatible library versions.

I realized when I decided to make a new website for [ProjectRobotPlayground](http://projectrobotplayground.com) that I had to refer back to my previous notes on how to set up the environment again. And of course, this time some dependent library in github pages was no longer compatible with the new version of Ubuntu I was running. This all contributed to me wasting my precious free time wrestling with dependencies.  

And then it hit me.

I don't have to keep doing this. I can just make a Docker image and use that! And with vscode, I can do all of my development for that one project within that one container as well!  

So I've decided to share this little hack with you.

## VS Code

For those of you who are unaware.  [VS Code](https://code.visualstudio.com/) is an open source editor that was developed by Microsoft.  It is deliciously compatible on nearly any operating system and has a large base of extensions you can download and install.

For me, it has been far more stable than [atom](https://atom.io/) and less complicated to set up than [sublime](https://www.sublimetext.com/).  Also, vscode has intellisense (which is just awesome), and it has built in debugging.  

_Note: I am not affiliated. I'm just a fan, and I'm rarely a fan of this kind of thing._

## Docker

[Docker](https://www.docker.com/) is one of the biggest container platforms around.  Containers let you wrap up software with its dependencies into a standardized unit that can be run anywhere.  For software development, it's great because it lets you keep everything that is needed to get your software up and running in one place.

This makes it so you can pause your personal projects for as long as you need to (hey! life happens) and you can jump right back in where you left off.  

Exactly where you left off.

You don't need to mess around with dependencies, or worry if you accidentally uninstalled something, or have versioning conflicts.  Plus, if you ever want to share your work, you can share not only your code, but also all of your dependencies easily.

## How to set up VS Code with Docker

One of the main reasons I really love vscode, is it's ability to be used in a ["remote" configuration](https://code.visualstudio.com/docs/remote/remote-overview).  Meaning, I can be editing files that don't live on my computer (like those in a client like docker).

I first started using this for work, since I could ssh into my machine and then bring up my familiar vscode editor to work remotely.  I found this much more productive than ssh-ing into my machine and using Vim.  Yes, I know there are people out there that only code in Vim.  I prefer a more fully-featured IDE.  But for those die-hards out there, vscode has Vim key bindings, so you can switch without having to re-memorize your key combos.

Later, I started playing around with [docker based development](https://github.com/athackst/workstation_setup/tree/master/examples) as I was playing with new [ROS2](https://index.ros.org/doc/ros2/) versions.  Then, I started using docker and vscode for everything.

The following instructions assumes you are somewhat familiar with vscode and docker.  If you're not, I encourage you to follow the links to learn more!

### 1. Install VS Code and Docker

[Download and install](https://code.visualstudio.com/download) VS Code from the [website](https://code.visualstudio.com/) to install the editor.

Then, follow the instructions on [dockerhub](https://hub.docker.com/search/?type=edition&offering=community) to install docker CE (community edition).

Finally, you'll want to install the vscode plugin that allows you to attach to a docker container. [Instructions here](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### 2. Set up your workspace

Open VS Code and clock on the green square in the bottom right corner.

![vscode container menu](/assets/img/vscode_container_open_ss.png)

This will bring up a menu that will let you chose which docker container to use.  There are many default containers, feel free to pick one!  

![vscode container menu](/assets/img/vscode_container_menu_ss.png)

This will make a `.devcontainer` folder inside your workspace.  If you look at it, you will see a `devcontainer.json` file, which tells VS Code how to launch your container and a Dockerfile that defines your container.

![vscode container menu](/assets/img/vscode_container_devcontainer_ss.png)

### 3. Download and configure extensions inside the container

You can even set up vscode plugins _inside your container_ so they will automatically load when you start the container.

Hello linting tools!

![vscode container menu](/assets/img/vscode_extensions_ss.png)

### 4. Some Gotchas

It's a good idea to start the docker container with your user id and group (instead of root). This is because any edits you make in VS Code are made inside the container and will inherit the owner of the containers user.  This means your containers will have to have a non-root user built into them with the same user id and group that you have.

It's also convenient to share your git credentials with the container.  If you use a credential manager, this will be automatically done for you!  If you use ssh, you will need to mount your credentials inside the container.

Finally, you'll want to set up you're favorite bash tools, including completion.
