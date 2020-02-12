#!/bin/bash

# Project Directory as an argument
dir=$1
cd $dir
source /opt/ros/${ROS_DISTRO}/setup.sh

rosdep update     
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y 
apt clean && rm -rf /var/lib/apt/lists/* 
