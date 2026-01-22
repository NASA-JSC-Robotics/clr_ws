# CLR Sim Demos

This repository contains demonstration applications using the [MuJoCo simulation](https://github.com/NASA-JSC-Robotics/chonkur_l_raile/) of ChonkUR L Rail-E.

This project is intended to be included in an application workspace such as the [clr_dynamic_sim_demo](https://github.com/NASA-JSC-Robotics/clr_dynamic_sim_demo) repository.

## Overview

The examples here, as well as the simulation itself, have been used to trial a sim-to-real pipeline for developing behaviors to run in iMETRO facilities.
These include behaviors requiring color/depth perception, force/torque sensors, or realistic dynamics of the robot in its environment.

As of now there is a single example available for public consumption made with open-source tools:

* [Pick and Place with a CTB](./clr_pick_and_place_demo/README.md)

![alt text](./clr_mujoco_simulation.png "CLR MuJoCo simulation")

## Citation

This project falls under the purview of the iMETRO project. If you use this in your own work, please cite the following paper:

```bibtex
@INPROCEEDINGS{imetro-facility-2025,
  author={Dunkelberger, Nathan and Sheetz, Emily and Rainen, Connor and Graf, Jodi and Hart, Nikki and Zemler, Emma and Azimi, Shaun},
  booktitle={2025 22nd International Conference on Ubiquitous Robots (UR)},
  title={Design of the iMETRO Facility: A Platform for Intravehicular Space Robotics Research},
  year={2025},
  volume={},
  number={},
  pages={390-397},
  keywords={NASA;Moon;Seals;Maintenance engineering;Maintenance;Robots;Standards;Open source software;Testing;Logistics},
  doi={10.1109/UR65550.2025.11077983}}
```
