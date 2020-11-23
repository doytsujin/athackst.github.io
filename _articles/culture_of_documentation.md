---
layout: text_entry
title: Creating a Culture of Documentation
category: Leadership
tags: [docker, documentation]
---
Documentation is one of those things that often gets overlooked.  After all, it's not strictly required for your code to run!  But there is a great amount of value to be gained by creating a culture of documentation.  I'm not talking about the "low quality" documentation that insists on simply re-writing in prose the name of the variable.

```c++
double foo; //a double variable named foo
```

That is _not_ what I mean by a "culture of documentation".  

What I mean is capturing all of the non-obvious steps you took and decisions you had to make as you were coding.  It's capturing all the steps it would take someone else to use the API you created. It's leaving yourself breadcrumbs so that if you had to come back to this line of code a year from now, you would know what was going on.

It is easier than you think to create a Culture of Documentation, but let's look at some common rationales against documentation amongst programmers.

## Common Complaints

### The code is self-documenting

So you're using "best coding practices", and your function and variable names are easily parsable.  Congratulations!  But you're only halfway there.  While it may be easy to understand "what" your code is doing, it may be much harder to parse "why".

Developers almost always overestimate the level of intuitiveness of the code they wrote and assume more capability in their audience than the audience has [[1](https://idratherbewriting.com/learnapidoc/docapiscode_research_on_documenting_code.html)].

This is especially true for any bug that was found and fixed.  If it wasn't completely obvious to you the first time around, leave your future self a hint as to why a line of code was wrong or had to change.  Bonus points for including any analysis you had to do to find the error.

### It's too hard to keep it up to date

It's true, documentation that's kept in many different places is hard to keep up to date.  If developers aren't consistently updating documentation with the source, it will get stale.  It should, therefore, be your goal to make documentation as ingrained into the process of developing code as possible.  If done right, documentation can become an essential step of programming, not a frivolity that can easily be skipped.  

### It adds too much time to development

This argument stems from the same issues that can cause documentation hard to keep up to date.  If your documentation steps aren't aligned with the process of developing code, it will seem like a bunch of "extra" work that isn't necessary.  The developers that feel this way often come from companies who prioritize "velocity", or academia where it isn't a part of a student's final grade.

It is, of course, not true that documentation takes too long or slows coding velocity.  If you've ever spent any time trying to fix a bug, you'll know how much time can be wasted just trying to understand why the code is the way it is.  You probably also have some stories where you forgot why a particular piece of code was added, removed it, and then had to "rediscover" what the issue was.

Don't be this person.

Documentation saves you future time.

## How to document your code

### Where to document (and why)

There are a couple of types of documentation you'll want to create.  They generally fall into the following categories:

* Inline comments
* Header/class documentation
* Usage documentation

#### Inline comments

The lowest friction documentation to make is inline comments.  You write these as you're creating your code and act as pointers to your future self for why you're code looks the way it does.  These types of comments explain _why_ something is happening in your code or for particularly dense functions, explain _what_ the code is intended to do.

```c++
      // We need to keep 'tf_buffer' small because it becomes very inefficient
      // otherwise. We make sure that tf_messages are published before any data
      // messages, so that tf lookups always work.
      std::deque<rosbag::MessageInstance> delayed_messages;
```

#### Header or class documentation

The second easiest documentation to generate is documentation from comments in a header file.  These comments are the ones that get exposed to the various auto-documentation tools like [autodoc](https://www.sphinx-doc.org/en/master/usage/extensions/autodoc.html#module-sphinx.ext.autodoc) or [mkdocstrings](https://pawamoy.github.io/mkdocstrings/).  

The documentation here should be aimed at other developers and clearly explain how the class should be used.  

```c++
/**
 * @brief Joint command interface allows other modules to set either a position
 * or a velocity command (or both), and the implementation class translates this
 * to a hardware command.
 *
 */
class JointCommand {

```

#### Usage documentation

Usage documentation lives outside of the main coding files.  This documentation includes things like definitions of commonly used terms and higher-level instructions on how to use the program or code like tutorials.  This type of documentation is normally what would go into a documentation website.  You write this documentation expecting an end-user will be the one to read it, as opposed to a fellow developer.

Often, this type of documentation is placed in a `docs` directory, but it doesn't have to be.  I find the most value when it is interspersed with my code.  This way, if something changes in the code it's very easy to find the corresponding doc to update.  I couldn't find any documentation generators that did this, so I created a plugin called [mkdocs-simple-plugin](https://athackst.github.io/mkdocs-simple-plugin) for [mkdocs](https://www.mkdocs.org/).

See how I created documentation for [mkdocs-simple-plugin](https://github.com/athackst/mkdocs-simple-plugin/tree/master/mkdocs_simple_plugin).

## How to encourage more documentation

### Have decent documentation tools

I cannot stress this enough.  You must do everything you can to lower the friction to creating and consuming good documentation.  This can be as simple as limiting context switching as much as possible.  I find that I prefer to document _as close to the code as possible_.  Not only does this make documenting easier as a developer, it also makes it easier to keep up-to-date.  Part of a good documentation tool is one that will surface documents in an easy-to-consume format, like creating a searchable documentation site for you.  It's important to realize that you'll want to lower friction on both document generation and documentation consumption, and that these two activities often happen through different platforms.  For example, a nicely formatted documentation site is better suited to reading documentation than raw markdown files.  However, you shouldn't make a developer switch to HTML to generate documentation content.

* [mkdocs](https://www.mkdocs.org/)
* [sphynx](https://www.sphinx-doc.org/en/master/index.html)

### Do it for yourself

Probably the best argument for documenting your code is to realize just how beneficial it is for yourself.  You will be saving your future self a whole lot of time writing down a few quick words on the problems you're encountering and why you decided to solve it the way you did.  You may also forget just what you had to install to get everything up and running, so go ahead and put that in there too.  

### Have maintainer support

Culture is created by re-affirming norms and rewarding those who follow good practices.  It's the duty of the maintainer to uphold code style and quality within the repositories they maintain.  It's also their duty to uphold the quality of documentation.  On this note, it's good to look for ways to make the maintainer's job easier.  For example

* [live previews from netlify](https://www.netlify.com/blog/2016/07/20/introducing-deploy-previews-in-netlify/): This will let a maintainer see the documentation along with the code they're reviewing.
* [htmlproofer in your CI](https://github.com/marketplace/actions/htmlproofer): This will automatically check links and other HTML properties are valid.
* [linters for style, spelling, and grammar](https://github.com/errata-ai/vale): This will check the documentation for common errors in prose.

#### References

* [1] [https://idratherbewriting.com/learnapidoc/docapiscode_research_on_documenting_code.html](https://idratherbewriting.com/learnapidoc/docapiscode_research_on_documenting_code.html)
* [2] [https://www.writethedocs.org/guide/tools/testing/](https://www.writethedocs.org/guide/tools/testing/)
* [3] [https://www.oreilly.com/content/the-eight-rules-of-good-documentation/](https://www.oreilly.com/content/the-eight-rules-of-good-documentation/)
