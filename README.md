# Project MIRA

This repository contains the ROS packages for the AUV2024 Dreadnought Autonomous Underwater Vehicle (AUV) project. The project aims to develop a robust and capable AUV for various underwater missions.

## Overview

- **src**: This folder contains the following ROS packages:
 - **controller**: This package handles the control and navigation systems of the AUV.
 - **perception**: This package deals with sensor data processing and perception algorithms for obstacle detection, mapping, and localization.
 - **visualize**: This package provides visualization tools for debugging and monitoring the AUV's state and sensor data.
- **custom_msgs**: This folder contains custom message definitions for communication between the ROS nodes.

## Usage

### Controller

To launch the teleoperation node, run the following command:
This will start the teleoperation node, allowing you to control the AUV manually.

### Perception

The perception package contains two subscriber nodes for processing data from different cameras. The source files for these nodes are located in the `src` directory of the perception package.

### Custom Messages

The `custom_msgs` folder contains the following custom message definitions:

- **thruster_commands.msg**: This message type is used to send command values to the Pixhawk flight controller for controlling the thrusters.
- **criticals.msg**: This message type is used to control the mode of the Pixhawk flight controller.

