---
layout: text_entry
title: VSCode Tasks Problem Matchers
category: Software Development
tags: [vscode]
image: /assets/img/vscode_run_task.gif
summary: VSCode tasks are a powerful tool.  This is how i use them.
---

If you use VSCode tasks, you'll probably be familiar with the warning that's reported in the command pallette if you haven't specified a "problem matcher" for your task.

![vscode no problem matcher](/assets/img/vscode_tasks_no_problemmatcher.png)

If you're like me, you probably just clicked "never scan the task output for this task" and moved on with your life, but there's a good reason to make one.

## What is a problem matcher

First, you should understand just what the problem matcher is.  In the panels beneath your code there is a "Problems" panel.  

![vscode problems panel](/assets/img/vscode_problems_panel.png)

This is an interactive panel that shows all of the problems that were detected, and lets you click on an issue to automatically bring up the offending file+line in the editor.

## Why use it for tasks

You may be thinking, "That's all well and good for background tasks, but why would I want this for tasks?"  

For one simple reason: it collates everything into one place and makes it easy to iterate through.

If you have several different linters you'd like to run plus tests, this will put all issues into one place so you don't have to look at the terminal output from many sources.  It turns out, this is insanely useful for things like my [vscode_ros2_workspace](https://github.com/athackst/vscode_ros2_workspace) template.

## How to make a problem matcher

There are some [excellent tutorials](https://code.visualstudio.com/docs/editor/tasks#_defining-a-problem-matcher) on how to make a problem matcher.  The TL;DR is that you should create a regex filter that matches the output of your task when there is an issue.

I test my regex by copy+pasting an output line from the terminal into an [interactive regex testing site](https://regex101.com/).  It's important to know that VSCode expects an extra `\` escape character, so you will have to modify the expression you create to add it.

For example `\d` to match a digit in a regular expression becomes `\\d` in tasks.

### Example

Let's create a simple problem matcher for `ament_flake8`.

When `ament_flake8` encounters a problem, the output looks like:

```bash
> Executing task: ament_flake8 src/ <


src/examples/rclpy/actions/minimal_action_client/examples_rclpy_minimal_action_client/client_cancel.py:38:5: E303 too many blank lines (2)
    def goal_response_callback(self, future):
    ^

src/examples/rclpy/actions/minimal_action_client/examples_rclpy_minimal_action_client/client_cancel.py:52:5: E303 too many blank lines (2)
    def feedback_callback(self, feedback):
    ^

2     E303 too many blank lines (2)

61 files checked
2 errors

'E'-type errors: 2
```

The problem matcher matches regex strings on a *single line*.  So we see that the most applicable line is

```bash
src/examples/rclpy/actions/minimal_action_client/examples_rclpy_minimal_action_client/client_cancel.py:38:5: E303 too many blank lines (2)
```

Where we see `<file>:<line>:<column>: <code> <message>`

Now we'll make a regex expression to match.  We can test it by pasting the output into [regex101](https://regex101.com)

[![regex pattern match](/assets/img/vscode_pattern_regex_match.png)](https://regex101.com/r/unMyUh/1)

The pattern I came up with is

```regex
^(.+):(\d+):(\d+): (\w\d+) (.+)$
```

Play with it [here](https://regex101.com/r/unMyUh/1)!

When you move this expression into vscode, it's important to know that you need to escape all `\`, with a `\\`.

The corresponding `task.json` setting then looks like:

```json
    "problemMatcher": [
        {
            "owner": "flake8",
            "source": "flake8",
            "pattern": [
            {
                "code": 4,
                "column": 3,
                "file": 1,
                "line": 2,
                "message": 5,
                "regexp": "^(.+):(\\d+):(\\d+): (\\w\\d+) (.+)$"
            }
            ]
        }
        ]
```

## Advanced topics

Simple single line outputs are the easiest to test + parse, but it also supports more complicated multi-line matches.  [This tutorial](https://code.visualstudio.com/docs/editor/tasks#_defining-a-multiline-problem-matcher) shows you how.

However, it is fairly limited in that it expects continuos matching patterns from a _single line_.  This works for most output, but not all.
