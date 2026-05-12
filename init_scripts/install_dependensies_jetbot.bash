#!/bin/bash

sudo update

sudo apt install \
    python3-pip \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-compressed-image-transport

pip install \
    pyserial==3.5
    
