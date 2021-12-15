#!/usr/bin/env bash
xhost +local:docker
docker run --net=host -h ubuntu_20_04_ros2_multi-docker -e DISPLAY --tmpfs /tmp -v /tmp/.X11-unix/:/tmp/.X11-unix:rw -v $PWD/../../../../ros_team_workspace/:$HOME/workspace/ros_team_workspace:rw --name ubuntu_20_04_ros2_multi-instance -it ubuntu_20_04_ros2_multi /bin/bash

