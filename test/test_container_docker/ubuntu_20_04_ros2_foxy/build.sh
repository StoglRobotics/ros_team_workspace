#!/usr/bin/env bash
docker build \
  --build-arg user=$USER \
  --build-arg uid=$UID \
  --build-arg gid=$GROUPS \
  --build-arg home=$HOME \
  -t ubuntu_20_04_ros2_foxy .
