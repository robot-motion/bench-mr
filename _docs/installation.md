---
layout: default
title:  "Installation"
date:   2021-01-04 13:20:59 +0100
has_children: true
permalink: /docs/installation
nav_order: 1
---

# Installation

Bench-MR is known to work on UNIX-based operating systems and uses CMake to find the C++ dependencies.

## Dependencies
* [libccd-1.4+](https://github.com/danfis/libccd/releases/tag/v1.4) (because of the `chomp` implementation used here), included as submodule and automatically built
* [OMPL](https://github.com/ompl/ompl) - included as submodule, needs to be installed first
* [nlohmann/json](https://github.com/nlohmann/json) - not provided, needs to be installed first
* [SBPL](https://github.com/sbpl/sbpl) - not provided, needs to be installed
* [Jupyter Lab](https://github.com/jupyterlab/jupyterlab) with [Python 3 kernel](https://ipython.readthedocs.io/en/latest/install/kernel_install.html#kernels-for-python-2-and-3) for plotting and evaluation (see [plotting/README.md](plotting/README.md))

The following boost libraries (version 1.58+) need to be installed:
* `boost_serialization`
* `boost_filesystem`
* `boost_system`
* `boost_program_options`

The provided CHOMP implementation requires, GLUT and other OpenGL libraries to be present, which can be installed through the `freeglut3-dev` package. PNG via `libpng-dev`, expat via `libexpat1-dev`.

Optionally, to support visual debugging, Qt5 with the `Charts` and `Svg` modules needs to be installed.

## Frontend Dependencies

The following system-wide dependencies need to be set up:

* Python 3.5+
* PIP 3 (install via `apt install python3-pip`)
* Jupyter Lab

The Python frontend dependencies are defined in `plotting/requirements.txt` which can be installed through
```
pip install -r plotting/requirements.txt
```
