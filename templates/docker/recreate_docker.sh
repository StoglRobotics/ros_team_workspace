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

  local docker_image_tag=DUMMY_DOCKER_IMAGE_TAG

  docker build \
  --build-arg user=$USER \
  --build-arg uid=$UID \
  --build-arg gid=$GROUPS \
  --build-arg home=$HOME \
  -t "$docker_image_tag" .
}

create_docker_container () {

  local docker_host_name=DUMMY_DOCKER_HOSTNAME
  local docker_image_tag=DUMMY_DOCKER_IMAGE_TAG
  local ws_folder="DUMMY_WS_FOLDER"

  local xauth_file_name=/tmp/${docker_host_name}.docker.xauth

  # BEGIN: Needed for Nvidia support
  if [ ! -f $xauth_file_name ]
  then
    touch $xauth_file_name
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
      echo $xauth_list | xauth -f $xauth_file_name nmerge -
    fi
    chmod a+r $xauth_file_name
  fi
  # END: Needed for Nvidia support

  echo "Instantiating docker image '$docker_image_tag' and mapping workspace folder to '$ws_folder'."
  echo "ros_team_ws is located under /opt/RosTeamWS/ros_ws_${RosTeamWS_DISTRO}/src/ros_team_workspace"
  xhost +local:docker
  docker run \
  --net=host \
  --gpus all \
  -h ${docker_host_name} \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=$xauth_file_name \
  --tmpfs /tmp \
  -v "$xauth_file_name:$xauth_file_name" \
  -v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
  -v "$HOME/.ssh":"$HOME/.ssh":ro \
  -v "$ws_folder":"$ws_folder":rw \
  --name "$docker_image_tag"-instance \
  -it "$docker_image_tag" /bin/bash
}

build_docker_image
create_docker_container
