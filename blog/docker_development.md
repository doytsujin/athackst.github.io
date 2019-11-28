---
layout: blog_entry
---

# The essential tools for easy code development

Lately, I've been experimenting with using Docker for all of my software development.

Yes. All of it.

In the past, I've only really used docker in a professional setting; Using it to capture and share somewhat esoteric code dependencies ensures a developer doesn't become derailed on an errant system update. It also helps new hires get into the code base faster since they don't have to download and install a million different dependencies of a specific version. And it helps companies support (or stage upgrades) on different workstation builds, since hosting the code in a docker container decouples it from the hosts operating system.  

This all seems like flagrant overkill for at home, for-fun coding.  

You only have one workstation you care about, and it's not like you have to on-board anyone else into your personal code base. So why would you ever care about using Docker for personal code development?  

I've found that even with my personal code development, I often want to have experimental spaces where I can download different libraries just to play around with them. I don't really want to have them installed on my system because:

1. they can be hard to remove (especially if it was some beta that didn't *quite* have it's uninstall hooks done correctly)
2. they may conflict with some other library I have on my system and
3. I'd have to continually keep it up to date and synced with compatible library versions.

I realized when I decided to make a new website for [ProjectRobotPlayground](http://projectrobotplayground.com) that I had to refer back to my previous notes on how to set up the environment again. And of course, this time some dependent library in github pages was no longer compatible with the new version of Ubuntu I was running. This all contributed to me wasting my precious free time wrestling with dependencies.  

And then it hit me. I don't have to keep doing this. I can just make a Docker image and use that! And with vscode, I can do all of my development for that one project within that one container as well!  

So I've decided to share this little hack with you.

## How to set up VS Code with Docker

### 1. Install VS Code and Docker

[Download and install](https://code.visualstudio.com/download) VS Code from the [website](https://code.visualstudio.com/) to install the editor.

Then, follow the instructions on [dockerhub](https://hub.docker.com/search/?type=edition&offering=community) to install docker CE (community edition).

Finally, you'll want to install the vscode plugin that allows you to attach to a docker container. [Instructions here](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### 2. Set up your workspace

Open VS Code and clock on the green square in the bottom right corner.

This will bring up a menu that will let you chose which docker container to use.  There are many default containers, feel free to pick one!  

This will make a `.devcontainer` folder inside your workspace.  If you look at it, you will see a `devcontainer.json` file, which tells VS Code how to launch your container and a Dockerfile that defines your container.

### 3. Download and configure extensions inside the container

You can even set up vscode plugins _inside your container_ so they will automatically load when you start the container.

Hello linting tools!

### 4. Some Gotchas

It's a good idea to start the docker container with your user id and group (instead of root). This is because any edits you make in VS Code are made inside the container and will inherit the owner of the containers user.  This means your containers will have to have a non-root user built into them with the same user id and group that you have.

It's also convenient to share your git credentials with the container.  If you use a credential manager, this will be automatically done for you!  If you use ssh, you will need to mount your credentials inside the container.

## See also

[VSCode, Docker, and Github Pages](vscode_docker_github_pages.html)