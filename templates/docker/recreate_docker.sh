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

build_docker_image () {
    docker build \
  --build-arg user=$USER \
  --build-arg uid=$UID \
  --build-arg gid=$GROUPS \
  --build-arg home=$HOME \
  -t DUMMY_DOCKER_IMAGE_TAG .
}

create_docker_container () {
  xhost +local:docker
  docker run \
  --net=host \
  -h DUMMY_DOCKER_HOSTNAME \
  -e DISPLAY \
  --tmpfs /tmp \
  -v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
  -v "$HOME/DUMMY_WS_FOLDER":"$HOME/$DUMMY_WS_FOLDER":rw \
  -v "$HOME/.ssh":"$HOME/.ssh":ro \
  --name DUMMY_DOCKER_IMAGE_TAG-instance \
  -it DUMMY_DOCKER_IMAGE_TAG /bin/bash
}

build_docker_image
create_docker_container
