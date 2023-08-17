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

# assoziative array which maps the chosen rosdistro to the name which is used to install the corresponding
# ros distribution inside docker
declare -r -A map_to_docker_ros_distro_name=( ["noetic"]="noetic" ["foxy"]="foxy" ["galactic"]="galactic" ["rolling"]="rolling" ["humble"]="humble" ["iron"]="iron")
declare -r -A ros_distro_to_rtw_branch=( ["noetic"]="master" ["foxy"]="foxy" ["galactic"]="master" ["rolling"]="master" ["humble"]="master" ["iron"]="master")

# $1 = name (tag) of the created docker image
# $2 = Dockerfile which is used for creating the image
RTW_Docker_build_docker_image () {
  if [ -z "$1" ]; then
    echo "No docker image tag specified. Can not create image."
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
  cd "$(dirname "$docker_file_path")" || { echo "Could not change directory to new workspace"; return 1; }

  notify_user "Building docker image $docker_image_tag with docker file $docker_file_path. This can take a while..."
  sleep 1 # sleep a second, so that user can read above message
  docker build \
  --build-arg user=$USER \
  --build-arg uid=$UID \
  --build-arg gid=$GROUPS \
  --build-arg home=$HOME \
  -t "$docker_image_tag" . \
  -f "$docker_file_path" || { return 1; }

  cd "$prev_pwd" || { print_and_exit "Build of docker container succeeded but changing back previous working directory failed."; }
}

RTW_Docker_create_docker_container () {
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

  if [ -z "$4" ]; then
    print_and_exit "No docker host name given. Can not instantiate image."
  fi
  local docker_host_name=$4

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

  notify_user "Instantiating docker image '$docker_image_tag' and mapping workspace folder to '$ws_folder'."
  xhost +local:docker
  docker run \
  --net=host \
  $([ $(ls -la /dev | grep nvidia | wc -l) "!=" "0" ] && echo "--gpus all") \
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

RTW_Docker_connect_user () {
  if [ -z "$1" ]; then
    print_and_exit "No container name given. Can not connect to instance."
  fi
  local container_instance_name
  container_instance_name=$1 # assigen separate https://github.com/koalaman/shellcheck/wiki/SC2155

  echo -e "${RTW_COLOR_NOTIFY_USER}Connecting to container instance as user:${TERMINAL_COLOR_NC} $container_instance_name"
  docker exec -u $USER -it "$container_instance_name" /bin/bash
}

RTW_Docker_connect_root_user () {
  if [ -z "$1" ]; then
    print_and_exit "No container name given. Can not connect to instance."
  fi
  local container_instance_name
  container_instance_name=$1 # assigen separate https://github.com/koalaman/shellcheck/wiki/SC2155

  echo -e "${RTW_COLOR_NOTIFY_USER}Connecting to container instance as root-user:${TERMINAL_COLOR_NC} $container_instance_name"
  docker exec -u root -it "$container_instance_name" /bin/bash
}

RTW_Docker_start_container () {
  if [ -z "$1" ]; then
    print_and_exit "No container name given. Can not start dockerinstance."
  fi
  local container_instance_name
  container_instance_name=$1 # assigen separate https://github.com/koalaman/shellcheck/wiki/SC2155

  echo -e "${RTW_COLOR_NOTIFY_USER}Starting container instance:${TERMINAL_COLOR_NC} $container_instance_name "
  xhost +local:docker
  docker start "$container_instance_name"
}

RTW_Docker_stop_container () {
  if [ -z "$1" ]; then
    print_and_exit "No container name given. Can not stop dockerinstance."
  fi
  local container_instance_name
  container_instance_name=$1 # assigen separate https://github.com/koalaman/shellcheck/wiki/SC2155

  echo -e "${RTW_COLOR_NOTIFY_USER}Stopping container instance:${TERMINAL_COLOR_NC} $container_instance_name"
  xhost -local:docker
  docker stop "$container_instance_name"
}

RTW_Docker_start_and_connect_user_to_docker () {
  if [ -z "$1" ]; then
    print_and_exit "The given docker tag is empty, something went wrong. Does your current workspace contain a RosTeamWS_DOCKER_TAG in .ros_team_ws_rc ?"
  fi
  local container_instance_name="$1-instance"

  RTW_Docker_start_container "$container_instance_name"
  RTW_Docker_connect_user "$container_instance_name"
}

RTW_Docker_start_and_connect_root_to_docker () {
  if [ -z "$1" ]; then
    print_and_exit "The given docker tag is empty, something went wrong. Does your current workspace contain a RosTeamWS_DOCKER_TAG in .ros_team_ws_rc ?"
  fi
  local container_instance_name="$1-instance"

  RTW_Docker_start_container "$container_instance_name"
  RTW_Docker_connect_root_user "$container_instance_name"
}

RTW_Docker_stop_docker_container () {
  if [ -z "$1" ]; then
    print_and_exit "The given docker tag is empty, something went wrong. Does your current workspace contain a RosTeamWS_DOCKER_TAG in .ros_team_ws_rc ?"
  fi
  local container_instance_name="$1-instance"

  RTW_Docker_stop_container "$container_instance_name"
}

RTW_Docker_clean_container_and_image () {
  if [ -z "$1" ]; then
    print_and_exit "The given docker tag is empty, something went wrong. Does your current workspace contain a RosTeamWS_DOCKER_TAG in .ros_team_ws_rc ?"
  fi
  local image_tag="$1"
  local container_instance_name="$image_tag-instance"

  RTW_Docker_stop_container "$container_instance_name"

  echo -e "${RTW_COLOR_NOTIFY_USER}Removing container instance:${TERMINAL_COLOR_NC} $container_instance_name"
  docker container rm "$container_instance_name"

  echo -e "${RTW_COLOR_NOTIFY_USER}Removing image:${TERMINAL_COLOR_NC} $image_tag"
  docker image rm "$image_tag"
}
