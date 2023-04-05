automatic-design-on-demand
========

[![Github license](https://img.shields.io/github/license/akhilsathuluri/x-ray-tool)](https://github.com/akhilsathuluri/x-ray-tool)
[![python](https://img.shields.io/badge/python-3.8-green)](https://github.com/akhilsathuluri/x-ray-tool)
[![Github release](https://img.shields.io/github/release/akhilsathuluri/x-ray-tool)](https://github.com/akhilsathuluri/x-ray-tool/releases)
[![Github issues](https://img.shields.io/github/issues/akhilsathuluri/x-ray-tool)](https://github.com/akhilsathuluri/x-ray-tool)

<p float="middle">
    <img src="https://github.com/akhilsathuluri/automatic-design-on-demand/blob/main/assets/robot_designDomain.png" alt="Image of a robot composition build with a given set of modules"/>
</p>

The `automatic-design-on-demand` is a Python based library to generate robot compositions using a prescribed set of modules. This library is developed and maintained by the [Robot Systems Group, Laboratory for Product Development and Lightweight Design, TU Munich](https://www.mec.ed.tum.de/en/lpl/research/research-groups/robot-systems/). It allows users to test their search algorithms or create interesting robot architectures by combining 3D-printable modules. The generated robot compositions can be simulated using the [Drake robotics toolbox](https://drake.mit.edu/).


## Platform
Tested with Python 3.8 on:
* Ubuntu 20.04 LTS and Ubuntu 22.04 LTS

## Installation
* Download the released version or clone the repository 
* Dependencies can be installed using the `requirements.txt` file or create a new conda environment using the `environment.yml` file

## Examples
An example demonstrating the usage and problem setup to find a robot composition to reach two given positions is available as [`stowingProblem`](https://github.com/akhilsathuluri/automatic-design-on-demand/blob/main/examples/example_robot_systems_design_v1.ipynb).

## Help
Use the `docs` folder present within the repo or raise an [issue](https://github.com/akhilsathuluri/automatic-design-on-demand/issues) for help.
