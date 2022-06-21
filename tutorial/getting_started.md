# Getting Started

This first tutorial takes you through setting up your machine to be able to run Starling in order to follow the rest of the tutorial.

[TOC]

## Preamble

For this tutorial, it will be assumed that you have a functional understanding of the Linux, Mac or Windows interface. This includes use of the command line and terminal applications on linux or Mac, and powershell or Windows Subsystem for Linux (WSL) on windows. If you do not feel comfortable with either of these, it is recommended you have a read of this tutorial first: [https://docs.starlinguas.dev/tutorials/introduction/#a-brief-introduction-to-linux](https://docs.starlinguas.dev/tutorials/introduction/#a-brief-introduction-to-linux). This tutorial covers some of the setup shown in this first tutorial in much more detail.

## Prerequisits

### Git and Docker

You will need to install [git](https://git-scm.com/downloads) to access the software and [docker](https://docs.docker.com/get-docker/) to run it.  These are both supported on Windows, Mac and Linux. The Docker Desktop application should also be suitable.

> Linux users, please verify that the `docker-compose` tool is installed by running `docker-compose --version`. If it fails, install using `sudo apt-get install docker-compose-plugin`. [see here](https://docs.docker.com/compose/install/compose-plugin/#installing-compose-on-linux-systems).

> Windows users, it is highly recommended that you also install the Windows Subsystem for Linux (WSL) and use that as the backend for your Docker installations. [see here for instructions](https://docs.docker.com/desktop/windows/wsl/)

It is also recommended that you sign up for a github account and a dockerhub account.

### Murmuration - Starling Command Line Interface

You will need to download the [Murmuration repository](https://github.com/StarlingUAS/Murmuration) which contains a useful command line interface (cli). This can hopefully abstract away the need to remember all of the different commands.

To install, go to your work directory and clone the repository using the command line or gui and run the following commands:

```bash
git clone https://github.com/StarlingUAS/Murmuration.git # clones locally
cd Murmuration
```

In the bin directory of the repository, there is the core cli script named `starling`. `starling` includes a further installation script to help install further requirements. This installation script will need to be run using root. See the following guide on which arguments you should give.

Then to finish the installation run:

```bash
sudo starling install
# or if you have not added starling to path and are in the Murmuration directory.
sudo ./bin/starling install
```

> If running within Murmuration, swap `starling` for `./bin/starling`. However for convenience, you can put `starling` onto your path. This can be done by adding `export PATH=<Path to murmuration>/bin:$PATH` into `~/.bashrc` followed by `source ~/.bashrc` , or running the same command locally in the terminal. Then you can use the `starling`

### Cookiecutter

The template generation uses the [`cookiecutter`](https://cookiecutter.readthedocs.io/en/stable/README.html) tool for generating custom projects from a template. To install, first install Python and then run the following:

```sh
python3 -m pip install --user cookiecutter
# or
easy_install --user cookiecutter
```

> See [cookiecutter installation](https://cookiecutter.readthedocs.io/en/stable/installation.html) for further details on different platforms.

This will give you access to the `cookiecutter` command line interface.

### Useful Programs

We highly recommend [Visual Studio Code](https://code.visualstudio.com/download) as our editing environment as it has a number of nice features, extensions and allows easy access to terminal windows. Once installed you can run vscode from any directory by running `code <my directory>`, e.g. `code .`.

## Next Steps

This should give you all the tools to be able to run the tutorials. Before we dive into creating your own, let us first introduce all the core technologies which Starling makes use of, in order to give you enough background for you all to create your own controllers.