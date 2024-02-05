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

# based on https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/
script_own_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
SCRIPT_PATH=$script_own_dir
source "$SCRIPT_PATH"/../../setup.bash

ROS_DISTRO=$1
if [ -z "$1" ]; then
  ROS_DISTRO=${ROS_DISTRO}
  echo "ROS 2 version not specified! Using default distribution \"${ROS_DISTRO}\""
  echo "Press <ENTER> to continue..."
  read
fi

if [[ ! -f "/etc/apt/sources.list.d/ros2.list" ]]; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
fi
sudo apt update

# ROS: Useful tools
sudo apt -y install python3-vcstool

sudo apt -y install ros-${ROS_DISTRO}-desktop

# sudo apt install -y python3-pip
# sudo pip3 install -U argcomplete
# sudo pip3 install -U rosdep

# echo "ROS - ROS 2 bridge also gets installed"
# echo "Press <ENTER> to continue..."
# read  # probably to be removed later
# sudo apt install -y ros-${ROS_DISTRO}-ros1-bridge

sudo apt install -y python3-colcon-common-extensions #Install colcon for workspace creation

sudo apt -y install ccache

sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

# visualizations
sudo apt -y install ros-${ROS_DISTRO}-plotjuggler-ros

# ros2_control
sudo apt -y install ros-${ROS_DISTRO}-forward-command-controller ros-${ROS_DISTRO}-joint-state-broadcaster ros-${ROS_DISTRO}-joint-trajectory-controller ros-${ROS_DISTRO}-xacro
sudo apt -y install ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-gazebo-ros2-control

# MoveIt
sudo apt -y install ros-${ROS_DISTRO}-geometric-shapes ros-${ROS_DISTRO}-moveit-msgs ros-${ROS_DISTRO}-moveit-resources ros-${ROS_DISTRO}-srdfdom ros-${ROS_DISTRO}-warehouse-ros
