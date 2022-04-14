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

# Load Framework defines
docker_script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $docker_script_own_dir/../_RosTeamWs_Defines.bash

ros_team_ws_dir="$docker_script_own_dir/../../../ros_team_workspace"

# assoziative array which maps the chosen rosdistro to the name which is used to install the corresponding
# ros distribution inside docker
declare -r -A map_to_docker_ros_distro_name=( ["foxy"]="foxy" ["galactic"]="galactic" ["rolling"]="rolling" )

# $1 = name (tag) of the created docker image
# $2 = Dockerfile which is used for creating the image
build_docker_container () {
  if [ -z "$1" ]; then
    echo "No docker image tage specified. Can not create image."
    return 1
  fi
  if [ -z "$2" ]; then
    echo "No docker file specified. Can not create image."
    return 1
  fi
  local docker_image_tag=$1
  local docker_file_path=$2

  # apparently docker checks every file in dir, its best to be in new dir
  prev_pwd=$(pwd)
  cd $(dirname "$docker_file_path")

  echo "Building docker image $docker_image_tag with docker file $docker_file_path. This can take a while..."
  sleep 1 # sleep a second, so that user can read above message
  docker build \
  --build-arg user=$USER \
  --build-arg uid=$UID \
  --build-arg gid=$GROUPS \
  --build-arg home=$HOME \
  -t "$docker_image_tag" \
  -f "$docker_file_path" .
  
  cd "$prev_pwd"
}

create_docker_image () {
  if [ -z "$1" ]; then
    print_and_exit "No docker image (tag) specified. Can not instantiate image."
  fi
  local docker_image_tag=$1

  if [ -z "$2" ]; then
    print_and_exit "No workspace which should be mapped to the container is given. Can not instantiate image."
  fi
  local ws_folder=$2

  if [ -z "$3" ]; then
    print_and_exit "No RosTeamWS_DISTRO given. Can not instantiate image."
  fi
  local RosTeamWS_DISTRO=$3

  local ws_folder_name
  ws_folder_name=$(basename "$ws_folder") # assigen separate https://github.com/koalaman/shellcheck/wiki/SC2155

  echo "Instantiating docker image $docker_image_tag and map workspace folder $ws_folder_name to $HOME/."
  echo "ros_team_ws is mounted under /opt/RosTeamWS/ros_ws_"$RosTeamWS_DISTRO"/src/ros_team_workspace"
  xhost +local:docker
  docker run \
  --net=host -h "$docker_image_tag"-docker \
  -e DISPLAY \
  --tmpfs /tmp \
  -v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
  -v "$HOME/$ws_folder":"$HOME/$ws_folder_name":rw \
  -v "$ros_team_ws_dir":/opt/RosTeamWS/ros_ws_"$RosTeamWS_DISTRO"/src/ros_team_workspace:rw \
  --name "$docker_image_tag"-instance \
  -it "$docker_image_tag" /bin/bash
}

connect_user () {
  if [ -z "$1" ]; then
    print_and_exit "No container name given. Can not connect to instance."
  fi
  local container_instance_name
  container_instance_name=$1 # assigen separate https://github.com/koalaman/shellcheck/wiki/SC2155

  echo "Connecting to container instance: $container_instance_name as user."
  docker exec -u $USER -it "$container_instance_name" /bin/bash
}

connect_root_user () {
  if [ -z "$1" ]; then
    print_and_exit "No container name given. Can not connect to instance."
  fi
  local container_instance_name
  container_instance_name=$1 # assigen separate https://github.com/koalaman/shellcheck/wiki/SC2155

  echo "Connecting to container instance: $container_instance_name as root-user."
  docker exec -u root -it "$container_instance_name" /bin/bash
}

start_container () {
  if [ -z "$1" ]; then
    print_and_exit "No container name given. Can not start dockerinstance."
  fi
  local container_instance_name
  container_instance_name=$1 # assigen separate https://github.com/koalaman/shellcheck/wiki/SC2155

  echo "Starting container instance: $container_instance_name "
  xhost +local:docker
  docker start "$container_instance_name"
}

stop_container () {
  if [ -z "$1" ]; then
    print_and_exit "No container name given. Can not stop dockerinstance."
  fi
  local container_instance_name
  container_instance_name=$1 # assigen separate https://github.com/koalaman/shellcheck/wiki/SC2155

  echo "Stopping container instance: $container_instance_name "
  xhost -local:docker
  docker stop "$container_instance_name"
}
