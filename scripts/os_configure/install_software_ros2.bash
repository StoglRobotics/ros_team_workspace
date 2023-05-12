#!/usr/bin/bash
# based on https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/

ROS_DISTRO=$1
if [ -z "$1" ]
then
  ROS_DISTRO=${ROS_DISTRO}
  echo "ROS 2 version not specified! Using default distribution \"${ROS_DISTRO}\""
  echo "Press <ENTER> to continue..."
  read
fi

if [[ ! -f "/etc/apt/sources.list.d/ros2-latest.list" ]]; then
  sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
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


sudo apt install ccache

sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

# visualizations
sudo apt -y install ros-${ROS_DISTRO}-plotjuggler-ros

# ros2_control
sudo apt -y install ros-${ROS_DISTRO}-forward-command-controller ros-${ROS_DISTRO}-joint-state-controller ros-${ROS_DISTRO}-joint-trajectory-controller ros-${ROS_DISTRO}-xacro

# MoveIt
sudo apt -y install ros-${ROS_DISTRO}-geometric-shapes ros-${ROS_DISTRO}-moveit-msgs ros-${ROS_DISTRO}-moveit-resources ros-${ROS_DISTRO}-srdfdom ros-${ROS_DISTRO}-warehouse-ros
