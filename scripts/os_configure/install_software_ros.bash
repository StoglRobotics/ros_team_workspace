#!/usr/bin/env bash
# Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
script_own_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
SCRIPT_PATH=$script_own_dir
source "$SCRIPT_PATH"/../../setup.bash

#### ROS Repository
if [[ ! -f "/etc/apt/sources.list.d/ros-latest.list" ]]; then
  sudo sh -c 'echo "deb http://packages.ros.org.ros.informatik.uni-freiburg.de/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
fi
sudo apt update

ROS_DISTRO=$1
if [ -z "$1" ]; then
  ROS_DISTRO=noetic
  echo "Ros version not provided! Using default distribution \"${ROS_DISTRO}\""
  echo "Press <ENTER> to continue..."
  read
fi

# Build Essentials
sudo apt -y install build-essential

# Libraries
sudo apt -y install libgoogle-glog-dev libatlas-base-dev

# ROS Base
sudo apt -y install ros-${ROS_DISTRO}-desktop-full ros-${ROS_DISTRO}-desktop ros-${ROS_DISTRO}-simulators

# ROS: Useful tools
sudo apt -y install python3-catkin-*
sudo apt -y install python3-catkin-lint python3-pip
sudo pip3 install osrf-pycommon
sudo apt -y install python3-wstool
sudo apt -y install python3-vcstool
sudo apt -y install clang-format
sudo apt -y install python3-rosinstall rospack-tools

sudo apt -y install ros-${ROS_DISTRO}-rospy-message-converter

sudo apt -y install ros-${ROS_DISTRO}-rosparam-handler

# ROS: Visualisation
sudo apt -y install ros-${ROS_DISTRO}-rqt-*
sudo apt -y install ros-${ROS_DISTRO}-plotjuggler-ros

# ROS: Robot packages
sudo apt -y install ros-${ROS_DISTRO}-cob-*
sudo apt -y install ros-${ROS_DISTRO}-moveit
sudo apt -y install ros-${ROS_DISTRO}-moveit-*
sudo apt -y install libmuparser-dev ros-${ROS_DISTRO}-brics-actuator ros-${ROS_DISTRO}-openrave ros-${ROS_DISTRO}-move-base
sudo apt -y install ros-${ROS_DISTRO}-teleop-twist-joy ros-${ROS_DISTRO}-twist-mux

# ROS Control
sudo apt -y install ros-${ROS_DISTRO}-control-*
sudo apt -y install ros-${ROS_DISTRO}-ros-control*
sudo apt -y install ros-${ROS_DISTRO}-ros-control ros-${ROS_DISTRO}-position-controllers ros-${ROS_DISTRO}-velocity-controllers ros-${ROS_DISTRO}-joint-trajectory-controller ros-${ROS_DISTRO}-joint-state-controller

sudo apt -y install ros-${ROS_DISTRO}-gazebo-ros-control ros-${ROS_DISTRO}-four-wheel-steering-msgs ros-${ROS_DISTRO}-urdf-geometry-parser ros-${ROS_DISTRO}-base-local-planner

# ROS Industrial
sudo apt -y install ros-${ROS_DISTRO}-industrial-robot-client

# ROS: Additional packages
sudo apt -y install ros-${ROS_DISTRO}-rosparam-handler ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-opencv3 ros-${ROS_DISTRO}-srdfdom ros-${ROS_DISTRO}-warehouse-ros

# TF2 Packages
sudo apt -y install ros-${ROS_DISTRO}-tf2 ros-${ROS_DISTRO}-tf2-sensor-msgs
