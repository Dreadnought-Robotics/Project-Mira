#!/bin/bash

sudo apt-get update

sudo apt-get install -y \
    ros-noetic-joy \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport

sudo apt-get install -y \
    libncurses5-dev \
    libncursesw5-dev \
    libeigen3-dev \
    libopencv-dev

echo "All dependencies installed successfully!"
