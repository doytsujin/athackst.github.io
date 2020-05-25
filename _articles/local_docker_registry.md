---
layout: text_entry
title: Setting Up a Local Docker Registry
category: Software Development
tags: [docker]
---

After you've been developing your code [using docker](/articles/docker_development.html) there will inevitably come a time when you'd like to share your workspace (code and all!) with someone else or put it on another computer.

A small little foray into google will tell you that there is a docker command called `docker save` that will let you do just that.  

But don't! (Please don't.)

There's a better way that doesn't involve compressing an entire disk image!

Docker images are really meant to be transferred through a Docker Registry.

You could set one up on a cloud service (aws, gcloud, or even git lets you do this!)

But if you intend on transferring the image inside of the same network, seriously consider using a local docker registry.  

Why?

* faster transfers through your local switch
* private
* why the hell not?

## Set up a Docker Registry

It's a really simple one line command.

```bash
docker run -d -p 5000:5000 --restart always --name registry registry:2
```

Yep, that's it!

Now your computer will run a docker registry, even if you restart your computer.

## Accessing the Docker Registry

For any other machine on your local network that you'd like to use the docker registry for, all you need to do is set up the docker daemon to use it.

Since it's a local registry, it doesn't have the security (https) that remote registries do.  So we will need to add it to the exception list so that we'll be allowed to pull from it.

Edit the `/etc/docker/daemon.json` to have the following line

```json
{ "insecure-registries":["your_hostname.local:5000"] }
```

Replace 'your_hostname' with the hostname of the computer you set up as the local docker registry, or its IP address.

> Note: `your_hostname.local` uses avahi to resolve the appropriate IP address.  This is normally preferred since routers typically dynamically assign IP addresses and this lets you reference a specific device without having to know what the IP address is, and without having to updated it.  If avahi doesn't work on your network for some reason, you can replace `your_hostname.local` with an IP address.

## Pushing to the Local Docker Registry

Pushing to a local Docker Registry is as easy as tagging an image with the registry name (the host name or IP of your registry + port number) and pushing.

For example:

```bash
docker tag your_docker_image your_hostname.local:5000/your_docker_image
docker push your_hostname.local:5000/your_docker_image
```

## Pulling from a Local Docker Registry

Pulling is similarly simple.

```bash
docker pull your_hostname.local:5000/your_docker_image
```

That's it!

All the layers are appropriately cached so incremental updates should be fast!
