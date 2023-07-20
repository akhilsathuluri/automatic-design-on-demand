automatic-design-on-demand
========

The `automatic-design-on-demand` is a Python based library to generate robot compositions using a prescribed set of modules. This library is developed and maintained by the [Robot Systems Group, Laboratory for Product Development and Lightweight Design, TU Munich](https://www.mec.ed.tum.de/en/lpl/research/research-groups/robot-systems/). It allows users to test their search algorithms or create interesting robot architectures by combining 3D-printable modules. The generated robot compositions can be simulated using the [Drake robotics toolbox](https://drake.mit.edu/).

[![Github license](https://img.shields.io/github/license/akhilsathuluri/automatic-design-on-demand)](https://github.com/akhilsathuluri/automatic-design-on-demand/blob/main/LICENSE)
[![python](https://img.shields.io/badge/python-3.8-green)](https://github.com/akhilsathuluri/automatic-design-on-demand/)
[![Github release](https://img.shields.io/github/v/release/akhilsathuluri/automatic-design-on-demand)](https://github.com/akhilsathuluri/automatic-design-on-demand/releases)
[![Github issues](https://img.shields.io/github/issues/akhilsathuluri/automatic-design-on-demand)](https://github.com/akhilsathuluri/automatic-design-on-demand/issues)

<table>
  <tr>
    <td rowspan="2"><img src="https://github.com/akhilsathuluri/automatic-design-on-demand/blob/main/assets/robot_designDomain.png" alt="Image of some of the modules" width="400"></td>
    <td colspan="1"><img src="https://github.com/akhilsathuluri/automatic-design-on-demand/blob/main/assets/gh_mod2-bg.png" alt="Image of some of the modules" width="600"></td>
  </tr>
  <tr>
    <td colspan="1"><img src="https://github.com/akhilsathuluri/automatic-design-on-demand/blob/main/assets/gh_mod1-bg.png" alt="Image of a robot composition build with a given set of modules" width = "700"></td>
  </tr>
</table>

## Physically realised prototype

https://user-images.githubusercontent.com/12780967/230316071-202554d5-281e-484d-b2e3-fa27d7017543.mp4

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

## Citation
You can see how to use this library for the automatic design of low-cost lightweight modular robots [here](https://www.mdpi.com/2218-6581/12/4/91)

If you use this library or find the documentation useful for your research, consider citing the work as:
```
@Article{robotics12040091,
AUTHOR = {Sathuluri, Akhil and Sureshbabu, Anand Vazhapilli and Frank, Jintin and Amm, Maximilian and Zimmermann, Markus},
TITLE = {Computational Systems Design of Low-Cost Lightweight Robots},
JOURNAL = {Robotics},
VOLUME = {12},
YEAR = {2023},
NUMBER = {4},
ARTICLE-NUMBER = {91},
URL = {https://www.mdpi.com/2218-6581/12/4/91},
ISSN = {2218-6581},
DOI = {10.3390/robotics12040091}
}
```
