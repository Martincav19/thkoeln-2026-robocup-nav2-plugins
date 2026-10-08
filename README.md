# RoboCup Nav2 Plugins

This repository contains the custom Navigation2 plugins developed for our RoboCup@Work navigation stack.

The code is based on the original [Navigation2](https://github.com/ros-navigation/navigation2) package structure. Only the files that were added or modified for our robot are included here.

The repository contains:

- Custom Nav2 plugins used in our navigation stack
- Modified plugin XML files required to register the custom plugins
- Modified `CMakeLists.txt` files required to build and link them

The original Nav2 source code is **not included**. The folder structure mirrors the upstream Nav2 repository so that the custom files can be placed into the corresponding locations of a compatible Nav2 checkout.

## Usage

1. Clone the appropriate version of Navigation2.
2. Copy the files from this repository into the corresponding Nav2 directories.
3. Build the workspace normally using `colcon build`.

## Background

These modifications were developed as part of the navigation system of our RoboCup@Work mobile robot, with a focus on improving navigation behavior in tight industrial environments.

Navigation2 itself is maintained by the ROS Navigation community and is available at:

https://github.com/ros-navigation/navigation2