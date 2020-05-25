---
layout: text_entry
title: Supercharging VSCode with Tasks
category: Software Development
tags: [vscode]
---

In my previous articles I showed you how I set up [github pages](/articles/vscode_docker_github_pages.html) and [ros2](/articles/vscode_docker_ros2.html) with vscode.

Tasks are my super weapon when it comes to productivity.  I make nearly every command I would type into a terminal a task.

This may seem like overkill to you, but there are some serious advantages to using tasks.  The types of commands I turn into a task are:

1. Commonly used commands. You can shorten the number of keystrokes it takes to execute a command.
    Yes, you can also do this with aliases, but that implies that you will use the same command in all of your contexts.

2. Rarely used commands. It lets you save them so you don't have to remember them.  
   I sometimes find that I have to re-lookup a command that I rarely use, which requires me to switch contexts and google it.  This is disruptive to my workflow.  Not only do I have to leave my IDE, but I have to take my hands off of my keyboard and use the mouse.

3. Chained commands.  You can also chain/group commands so that they will all be executed in their own (separate) terminals.
   If you've ever developed in ROS, you'll know that having many terminals, each with their own rosnode or roslaunch on it is very common practice.  You wind up with a lot of terminals that becomes hard to manage.

   Bonus: if you re-run a task, it will only restart the tasks that have stopped, letting you restart processes that have died and let long running processes continue to run undisturbed.

So yeah, almost _all_ of the commands I would ever use.

Notably missing is git commands.  I find it easier to have a unified git experience that spans across all my contexts, so I have a custom alias for commonly used git sequences but otherwise type those into the terminal.  Git requires more interaction than is really useful for a task.  Turns out commands that change often (hello git branch names!) are poorly suited to tasks.  Sure, you can make a task that takes in user input, but I find the user input interface a little lacking.  For example, you can't go back in history to the last input you typed in like you can with a terminal.

## Create a keyboard shortcut

### Run tasks

The first thing you're going to want to do is create a keyboard shortcut to the tasks menu.

![vscode keyboard shortcut](/assets/img/vscode_keyboard_shortcuts.gif)

1. Go to File->Preferences->Keyboard Shortcuts.

2. On the menu that opens, search for `workbench.action.tasks.runTask`

3. Assign the command to the key combination of your choice.  I use `ctrl+;`

## Name your tasks with speed in mind

You can make your task names as descriptive as you'd like, but here's a tip:  If you begin the task name with a unique key combination it will be easier to select the task.  As you type in the window, tasks with matching names are shown in the drop down.  If you put a unique key combination as the first couple of characters (just like you would an alias), you can quickly select the task you want.  This makes it so your task names can be as descriptive as you need them to be (handy for those rarely used tasks!) yet still enables quick selection of a commonly used tasks.

![vscode run task](/assets/img/vscode_run_task.gif)

I prefix my tasks with a short letter combination so that I can easily type it into search bar.

Also you can easily get to the last run command, since it is always the first task in the drop down.  Just scroll down!
