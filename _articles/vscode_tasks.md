---
layout: text_entry
title: Supercharging VSCode with Tasks
category: Software Development
tags: [vscode]
---

In my previous articles, I showed you how I set up [github pages](/articles/vscode_docker_github_pages.html) and [ros2](/articles/vscode_docker_ros2.html) with VSCode.

Tasks are my superweapon when it comes to productivity.  I make nearly every command I would type into a terminal a task.

I mean it!

This may seem like overkill to you, but there are some serious advantages to using tasks.  The types of commands I turn into a task are:

1. **Commonly used commands.** You can shorten the number of keystrokes it takes to execute a command.
    You could also do this with aliases, but what's the fun in that?  Also, tasks allow you to have _custom_ commands that change with _context_.

2. **Rarely used commands.** You can save rare-but-important commands (with comments!) so you don't have to remember them.  
   I find that I have to re-lookup commands that I rarely use, which is disruptive to my workflow.  Not only do I have to leave my IDE to ask google, but I have to take my hands off of my keyboard and use the mouse. The horror!

3. **Chained commands.**  You can also chain/group commands so that they will all be executed in their own (separate) terminals.
   If you've ever developed in ROS, you'll know that having many terminals, each starting a new rosnode or roslaunch on it is a very common practice.  You always wind up with a lot of terminals that become hard to manage.

>   Bonus: if you re-run a task, it will only restart the tasks that have stopped, letting you restart processes that have died and let long-running processes continue to run undisturbed.

So yeah, almost _all_ of the commands I would ever use.

Notably missing from this list are git commands.  I find it easier to have a unified git experience that spans across all my contexts, so I have a custom alias for commonly used git sequences but otherwise type those commands into the terminal.  Git requires more interaction than is useful for a task.  Turns out commands that change often (hello, git branch names!) are poorly suited to tasks.  Sure, you can make a task that takes in user input, but I find the user input interface a little lacking.  For example, you can't go back in history to the last input you typed in like you can with a terminal.

## Create a keyboard shortcut

### Run tasks

The first thing you're going to want to do is to create a keyboard shortcut for the tasks menu.

![vscode keyboard shortcut](/assets/img/vscode_keyboard_shortcuts.gif)

1. Go to File->Preferences->Keyboard Shortcuts.

2. On the menu that opens, search for `workbench.action.tasks.runTask`

3. Assign the command to the key combination of your choice.  I use `ctrl+;`

## Name your tasks with speed in mind

You can make your task names as descriptive as you'd like, but here's a tip:  If you begin the task name with a unique key combination it will be easier to select the task.  As you type in the window, tasks with matching names are shown in the dropdown.  If you put a unique key combination as the first couple of characters (just like you would an alias), you can quickly select the task you want.  This makes it so your task names can be as descriptive as you need them to be (handy for those rarely used tasks!) yet still enables quick selection of commonly used tasks.

![vscode run task](/assets/img/vscode_run_task.gif)

I prefix my tasks with a short letter combination so that I can easily type them into the search bar.

You can also easily find the last run command since it is always the first task in the dropdown.  Just scroll down!
