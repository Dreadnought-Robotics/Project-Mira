# Project MIRA

This repository contains the ROS packages for the AUV2024 Dreadnought Autonomous Underwater Vehicle (AUV) project. The project aims to develop a robust and capable AUV for various underwater missions.

## Overview

- **src**:
 - **mira_controller**: This package handles the control and navigation systems of the AUV.
 - **mira_perception**: This package deals with sensor data processing and perception algorithms for obstacle detection, mapping, and localization.
 - **mira_docking**: This package contains the code for the docking task.
 - **visualize**: This package provides visualization tools for debugging and monitoring the AUV's state and sensor data.
 - **custom_msgs**: This folder contains custom message definitions for communication between the ROS nodes.

## Usage

### Setup

Before running the packages, add the following lines to your `~/.bashrc` file:

```bash
export ROS_IP=192.168.2.3
export ROS_MASTER_URI=http://192.168.2.1:11311
export ROS_HOSTNAME=192.168.2.1
```

After adding the lines, source your workspace where this repository is cloned:
```bash
source /path/to/workspace_name/devel/setup.bash
```

Then, build the packages using catkin_make.

## Prerequisites
Before running the code, make sure you have completed the setup steps mentioned in the main README file. This includes setting the ROS environment variables and sourcing the workspace.

## Running the Code

1. **Launch the Perception Node**

  ```bash
  $ roslaunch mira_perception perception.launch
  ```
  This will start the perception nodes responsible for processing sensor data and detecting obstacles, mapping, and localization.

2. **Launch the ROV Node**
  ```bash
  $ roslaunch mira_rov teleop.launch
  ```
  This will launch the Remotely Operated Vehicle (ROV) node, which handles the control and navigation systems of the AUV.

3. **Launch the Docking Node**
  ```bash
  $ roslaunch mira_docking dock.launch
  ```
  Launch the docking node, which contains the code for the docking task, and kill the ROV node.

4. **Launch the Docking Node**
  ```bash
  $ roslaunch mira_docking dock.launch
  ```
  Launch the docking node, which contains the code for the docking task, and kill the ROV node.




