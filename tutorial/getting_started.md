# Getting Started

This first tutorial takes you through setting up your machine to be able to run Starling in order to follow the rest of the tutorial.

[TOC]

## Preamble

For this tutorial, it will be assumed that you have a functional understanding of the Linux, Mac or Windows interface. This includes use of the command line and terminal applications on linux or Mac, and powershell or Windows Subsystem for Linux (WSL) on windows. If you do not feel comfortable with either of these, it is recommended you have a read of [this tutorial](https://docs.starlinguas.dev/tutorials/introduction/#a-brief-introduction-to-linux) first, which covers some of the set up shown here in much more detail.

## Prerequisites

### Git and Docker

You will need to install [git](https://git-scm.com/downloads) to access the software and [Docker](https://docs.docker.com/get-docker/) to run it.  These are both supported on Windows, Mac and Linux. The Docker Desktop application should also be suitable.

> Linux users, please verify that the `docker-compose` tool is installed by running `docker-compose --version`. If it fails, install using `sudo apt-get install docker-compose-plugin`. [see here](https://docs.docker.com/compose/install/compose-plugin/#installing-compose-on-linux-systems).

> Windows users, it is highly recommended that you also install the Windows Subsystem for Linux (WSL) and use that as the backend for your Docker installations. [see here for instructions](https://docs.docker.com/desktop/windows/wsl/)

It is also recommended that you sign up for a Github account and a Docker Hub account.

### Murmuration - Starling Command Line Interface

You will need to download the [Murmuration repository](https://github.com/StarlingUAS/Murmuration) which contains a useful command line interface (cli). This will hopefully abstract away the need to remember all of the different commands.

To install, first go to your work directory. Then run the following to clone the repository and go to the `Murmuration` directory

```bash
git clone https://github.com/StarlingUAS/Murmuration.git # clones locally
cd Murmuration
```

In the `bin` directory of the repository, there is the core CLI script named `starling`. `starling` includes a further installation script to help install further requirements. This installation script will need to be run using root. Run:

```bash
sudo starling install
# or if you have not added starling to path and are in the Murmuration directory.
sudo ./bin/starling install
```

> If running within Murmuration, swap `starling` for `./bin/starling`. However for convenience, you may want to create a PATH variable for `starling` so you can run `starling` from anywhere. Do this by adding the line `export PATH=<Path to murmuration>/bin:$PATH` to your `~/.bashrc` file. This lets `bash` know where to find the executable for `starling`. Run `source ~/.bashrc` to refresh your current bash environment and load this new variable.

You will now have available the Starling CLI. It incorporates the most used functions of running Starling UAV systems. You can see the available commands by running:

```console
> starling help
starling

Starling CLI

Usage: starling [command]

Commands:

  deploy	Starts Starling Server
  install	Installs Base Starling Dependencies
  simulator	Starts Starling Server
  start	Starts Starling Server
  status		Starts Starling Server
  stop		Stops Starling Server
  utils		Utility functions
  *		Help
```

### Cookiecutter

> Prerequisites: You need to have installed Python.

The template generation uses the [`cookiecutter`](https://cookiecutter.readthedocs.io/en/stable/README.html) tool for generating custom projects from a template. Then to install cookiecutter, run the following:

```sh
python3 -m pip install --user cookiecutter
# or
easy_install --user cookiecutter
```

> See [cookiecutter installation](https://cookiecutter.readthedocs.io/en/stable/installation.html) for further details on different platforms.

This will give you access to the `cookiecutter` command line interface.

### Useful Programs

We highly recommend [Visual Studio Code](https://code.visualstudio.com/download) as your editing environment as it has a number of nice features, extensions and allows easy access to terminal windows. Once installed, you can open VS Code on any directory via the terminal by running `code <my directory>`, or `code .` if you are already in the directory.

## Next Steps

This should give you all the tools to be able to run the tutorials. Next, we'll be introducing you to the core technologies of Starling. This will give you enough background to start creating your own controllers!
