# AstroROS
Spacecraft Orbit/Attitude Simulation for ROS2
# Introduction
This repository is for AstroROS : ADR simulator for On-Orbit Servicing.

This repo is a fork repo from https://github.com/uscl/AstroROS .

Every changes (even they are trivial) will be applied in this repo and revised version will be uploaded in original repository.

# How to use AstroROS

AstroROS for ROS2 is on-progress.

First, you have to install ROS2 Humble and ignition Fortress.

And then, git clone this package.

```
mkdir -p astroros_ws && cd astroros_ws
```
```
git clone https://github.com/MichaelBentleyOh/AstroROS
```
Finally build the package with colcon.
```
colcon build --symlink-install
```
# Update Log & Plan

2025.03.03 : Upload "satellite_description" folder

2025.03.31 : update for orbit propagation

2025.09.?? : add impedance controller for tentacle grasp
