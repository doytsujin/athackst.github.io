---
layout: text_entry
title: VSCode, Docker, and Github Pages
category: Software Development
tags: [vscode, docker, github_pages]
---

As a casual user of Github Pages, I find that it's difficult to remember how I set up the development environment so I can edit my website.  There's nothing worse to losing hours to configuration nightmares on coding that is supposed to be for fun.  And since I use [docker and vscode](/articles/docker_development.html), it was a pretty easy decision to try to put my website workspace into a container.

The following describes how I use docker to set up a standardized environment for Github pages.

## Make a docker image

The first thing you need to do is set up an appropriate docker container.  This turned out to be a little more difficult than I thought it would be.  Since the container needs to support as both a server for the webpage and a development environment, I couldn't find a vanilla Dockerfile I could use.

At least easily.

But now that I've made one, I'm sharing it with you!

### Find a base docker image

The first thing you need to do is create a docker image.  Docker images always have a `FROM` tag which lets you build off of other docker images. Sometimes, the best you can do is start with a blank operating system and build your image from scratch.  Luckily, Jekyll already supports a `pages` version in their [docker repository](https://hub.docker.com/r/jekyll/jekyll/tags).

So let's go ahead and use Jekyll's docker image for pages.  By doing this, we can keep up to date with the latest compatible library set for Github pages easily.

```docker
FROM jekyll/jekyll:pages
```

By using a supported image, the makers of Jekyll will be the ones to ensure that all of the packages are compatible and work with Github pages.  This means that the container is more or less guaranteed to be compatible with Github pages.  Before I started using this container, I had issues tracking which versions of packages were compatible.  There was even a change in bundler that was [ABI incompatible](https://bundler.io/blog/2019/01/04/an-update-on-the-bundler-2-release.html) and the old instructions I was using weren't working anymore.

Sign me up for getting to the good stuff faster!

### Copy setup files

Next, you'll want to copy the Gemfile from your workspace into the docker container so docker can use it during the build

```docker
FROM jekyll/jekyll:pages

COPY Gemfile /srv/jekyll/
```

My Gemfile is simple.  It only has the Github pages gem in it.

```js
source 'https://rubygems.org'
gem 'github-pages', group: :jekyll_plugins
```

### Install dependencies

Even though the base docker image is pages compatible, you can't develop on it yet. The tools you'll need to have to serve your Github pages website locally aren't included by default.  You will need to download and install the development libraries for your Gemfile.  But first, you will need to make sure that you have all of the basic libraries you need.  Since the image this is based on is a smaller, stripped-down image, it may not have many of the "standard" development libraries.

I added the following to my dockerfile:

```docker
# Install development packages
RUN apk update && apk add \
    # development packages
    ruby-dev \
    gcc \
    make \
    curl \
    build-base \
    libc-dev \
    libffi-dev \
    zlib-dev \
    libxml2-dev \
    libgcrypt-dev \
    libxslt-dev \
    python \
    # pushing to git via ssh
    openssh \
    # permissions to install packages
    sudo \
    # tab completion inside the container
    git-bash-completion

RUN bundle install && bundle update
```

Note that I've added a couple of extra dependencies here as well.  I've added OpenSSH, sudo, and git-bash-completion.

### Update the user

Since we'll be developing inside a docker container, it should have a non-root user.  The non-root user will allow files saved within the container to have your user and group permissions.  You'll also want that user to have `sudo` access.

You may have heard that adding a non-root user to your dockerfile is good practice, but giving that user sudo access defeats the purpose.  This is true for an image you're planning on deploying (why lock the door if you're handing out keys?) but not necessary for a docker container that's only going to be used for development like this one.

The user needs sudo access so that you can install extensions and other addons (and is just dang useful to have it).

```docker
# Set up container user.
ENV USERNAME=jekyll
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME
```

One last thing you may want to do is install bash-completion scripts.  Oftentimes when I'm working on code, I'll want to do things like commit my changes to Github from within the integrated terminal with nice things like tab-completion working appropriately.

```docker
# Set up git completion.
RUN echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc
```

### Expose the port

Finally, you'll want to expose the port that the container will publish to when you serve the website locally using `bundle exec jekyll serve`

```docker
EXPOSE 4000
```

One thing to note here is that the port that this attaches to on your host computer is in the _ephemeral port range_ and *not* the port that you exposed.  You can see what port it's mapped to on your computer through

```bash
$ docker ps
CONTAINER ID        IMAGE                                                     COMMAND                  CREATED             STATUS              PORTS                                 NAMES
3d0128109045        vsc-athackst.github.io-b3b45c0d050bd0f420e78b620fe5b730   "/bin/sh -c 'echo Co…"   24 hours ago        Up 22 seconds       127.0.0.1:4000->4000/tcp, 35729/tcp   laughing_jepsen
```

The neat thing is, if you click on the website link within the docker terminal window, it will _automatically_ open the page in your browser with the right mapped port!

### Put it all together

You should now have a docker file that looks like this:

```docker
FROM jekyll/jekyll:pages

COPY Gemfile /srv/jekyll/

WORKDIR /srv/jekyll

RUN apk update && apk add \
    # development packages
    ruby-dev \
    gcc \
    make \
    curl \
    build-base \
    libc-dev \
    libffi-dev \
    zlib-dev \
    libxml2-dev \
    libgcrypt-dev \
    libxslt-dev \
    python \
    # pushing to git via ssh
    openssh \
    # permissions to install packages
    sudo \  
    # tab completion inside the container
    git-bash-completion

RUN bundle install && bundle update

# Set up a non-root user.
ENV USERNAME=jekyll
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set up git completion.
RUN echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

EXPOSE 4000
```

## Set up VSCode

In order for VSCode to use your docker file, it will create a `devcontainer.json` file with all of the docker settings.  This includes things like how to launch the docker.

Let's take a look:

```javascript
// For format details, see https://aka.ms/vscode-remote/devcontainer.json or the definition README at
// https://github.com/microsoft/vscode-dev-containers/tree/master/containers/ubuntu-18.04-git
{
    "name": "Github Pages",
    "dockerFile": "Dockerfile",
    "context": "../",
    // The optional 'runArgs' property can be used to specify   additional runtime arguments.
    "runArgs": [
        "-u", "jekyll",
        "-v", "${env:HOME}${env:USERPROFILE}/.ssh:/root/.ssh-localhost:ro"
    ],

    // Use 'settings' to set *default* container specific   settings.json values on container create.
    // You can edit these settings after create using File >    Preferences > Settings > Remote.
    "settings": {
        "terminal.integrated.shell.linux": "/bin/bash"
    },

    // The port(s) to open from your container.
    "appPort": [4000],

    // Commands to run after the container is created.
    "postCreateCommand": "sudo cp -r /root/.ssh-localhost ~ && sudo chown -R $(id -u):$(id -g) ~/.ssh-localhost && mv ~/.ssh-localhost ~/.ssh && chmod 700 ~/.ssh && chmod 600 ~/.ssh/*",

    // Add the IDs of extensions you want installed when the container is created in the array below.
    "extensions": [
        // yaml for data files
        "redhat.vscode-yaml",
        // liquid templating syntax highlighting
        "sissel.shopify-liquid",
        // jekyll
        "ginfuru.ginfuru-vscode-jekyll-syntax",
        "ginfuru.vscode-jekyll-snippets",
        // markdown
        "yzhang.markdown-all-in-one",
        "davidanson.vscode-markdownlint",
        // html/css
        "ecmel.vscode-html-css",
        "aeschli.vscode-css-formatter",
        // editing
        "streetsidesoftware.code-spell-checker",
        "ms-vscode.wordcount"

    ]
}
```

### Remote user

The first thing you're going to want to do is set the remote user so that your id and group will be properly exported into the container.

```javascript
    "remoteUser": "jekyll",
```

### App port

Since this container is meant to serve your site locally to debug, you need to be sure to expose the port to your host system.

```javascript
    // The port(s) to open from your container.
    "appPort": [4000],
```

You can optionally set it up to run on a _specific_ port by adding the following to the run args.

```bash
    "runArgs": [
        "--publish", "4000:4000",
    ]
```

### SSH Credentials

SSH Credentials are automatically shared with the container after [VS Code v1.41](https://github.com/microsoft/vscode-docs/blob/master/remote-release-notes/v1_41.md)

### Extensions

The last setting of note is the extensions setting.  These are the extensions that will be loaded into the docker container after it has been built.  These are not required but are nice to include to ensure that productivity-boosting extensions are available and configured correctly.

For Github pages, I like the following extensions.

```javascript
    "extensions": [
        // yaml for data files
        "redhat.vscode-yaml",
        // liquid templating syntax highlighting
        "sissel.shopify-liquid",
        // jekyll
        "ginfuru.ginfuru-vscode-jekyll-syntax",
        "ginfuru.vscode-jekyll-snippets",
        // markdown
        "yzhang.markdown-all-in-one",
        "davidanson.vscode-markdownlint",
        // html/css
        "ecmel.vscode-html-css",
        "aeschli.vscode-css-formatter",
        // editing
        "streetsidesoftware.code-spell-checker",
        "ms-vscode.wordcount"
    ]
```

And that's it!  You're ready to start your own Github pages site.
