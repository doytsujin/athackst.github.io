---
layout: text_entry
title: Unboxing the AWS Deep Racer
category: Robotics
series: DeepRacer
tags: deepracer
---

![aws deep racer](/assets/img/IMG_5239.jpg)

It arrived!

I pre-ordered the AWS DeepRacer months ago in anticipation of getting my hands dirty both with AWS and ML.  But it was delayed.

And delayed.

And delayed.

And then they canceled my order.

I thought that was going to be it, but nope!

## About the AWS Deep Racer

The AWS DeepRacer is a robot developed by Amazon to help you get started with reinforcement learning.  They've built an entire ecosystem around it, including a simulation, the race car, and a global racing league.  I was less interested in the racing league than getting a pre-built robot that supports ROS right out of the box.  Finally!  An affordable ROS-enabled robot that I won't have to write the drivers for.  I am also interested in exploring more machine learning techniques and thought this would be a quick way to get started.

## The hardware

I was _shocked_ when it showed up on my doorstep one day. The last email I received from Amazon notified me that my order was canceled and I'd have to re-order again when it was available.

I didn't.

I must've gotten the last one in the warehouse though because the box arrived damaged.

![deepracer broken box](/assets/img/IMG_5238.jpg)

Womp. Womp.

At least it still works?

![deepracer](/assets/img/IMG_5257.jpg#right)

The AWS Deep racer is an 18th scale 4WD with monster truck chassis. So it's got some decent shock absorption and can get over uneven terrain pretty easily.  It's small enough and light enough to pick up and move around without having to worry about it damaging things.  Although, I'm guessing the chassis breaks fairly easily since they included extra stanchions.  It has an Intel Atom processor with 4 GB RAM and 32 GB of storage.  It's definitely enough to get you started.  You can even load in an SD card if you need more memory.  The only sensors it comes with is an imu/accelerometer combo and a 4 MP camera, but there are a lot of expansion ports if you want to add your own custom hardware.

Love it or hate it, the DeepRacer is very much a prototype bot.

You have to charge the main chassis separately from the compute.  You'll also need to take the whole thing apart to install the batteries, just like you would if you were hacking together your own.  This isn't a fully baked consumer product like a [sphero](https://www.sphero.com/).  You'll need to be comfortable taking electronics apart and putting them back together again.

## The software

It comes pre-loaded with ROS Kinetic (which is a little dated at this point).  I'm hoping it will be fairly easy to port it to a different version of ROS or use a docker container for development since that's my preferred mode of development anyway.  Check out my tutorial on how I set up [vscode for container-based development](/articles/docker_development.html).

I'll be sharing my development notes and journey with the deep racer here, so stay tuned!

![deepracer](/assets/img/IMG_5256.jpg)
