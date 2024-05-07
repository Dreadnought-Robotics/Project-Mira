# Project MIRA

This repository contains the ROS packages for the AUV2024 Dreadnought Autonomous Underwater Vehicle (AUV) project. The project aims to develop a robust and capable AUV for various underwater missions.

## Overview

- **Docker**:
- Contains a custom docker image file for running the entire system on Docker container

- **src**:
- **mira_controller**: This package handles the control and navigation systems of the AUV.
- **mira_perception**: This package handles image processing and perception algorithms.
- **mira_docking**: This package contains the code for the docking task.
- **visualize**: This package provides visualization tools for debugging and monitoring the AUV's state and sensor data.
- **custom_msgs**: This folder contains custom message definitions for communication between the ROS nodes.

## Usage

### Setup

**Install the required packages, and dependencies**

Install Joy node for controller communication

```bash
sudo apt-get install ros-noetic-joy
```

Install cv_bridge and image_transport packages

```bash
sudo apt-get install ros-noetic-cv-bridge ros-noetic-image-transport
```

Install OpenCV (preferably version 4.6)

```bash
sudo apt-get install -y libopencv-dev
```

Before running the packages, add the following lines to your `~/.bashrc` file:

```bash
export ROS_IP=192.168.2.3
export ROS_MASTER_URI=http://192.168.2.1:11311
export ROS_HOSTNAME=192.168.2.3
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

This will start the perception nodes responsible for subscribing camera topics and publishes Aruco id, waypoints and pose.

2. **Launch the ROV Node**

```bash
$ roslaunch mira_rov teleop.launch
```

This will launch the Remotely Operated Vehicle (ROV) node, to move the AUV using the controller using the joystick. Run this till the Aruco market is detected.

3. **Launch the Docking Node**

```bash
$ roslaunch mira_docking dock.launch
```

Launch the docking node, which contains the code for the docking task, and kill the ROV node.

4. **Launch the Controller Node**

```bash
$ roslaunch mira_controller controller.launch
```

Launch the controller node for the AUV to navigate autonomously.
