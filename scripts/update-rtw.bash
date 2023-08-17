#!/bin/bash
#
# Copyright 2023 Stogl Robotics Consulting
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

script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $script_own_dir/../setup.bash

cd $FRAMEWORK_BASE_PATH

# Safe state if there are local changes
git stash

git pull --rebase origin

source setup.bash


# Revert state
git stash pop


## Update variables with "exports"
ros_team_ws_file_name=".ros_team_ws_rc"
ros_team_ws_file="$HOME/$ros_team_ws_file_name"

rtw_setup_ros_team_ws_file() {
  if [ -z "$1" ]; then
    print_and_exit "No ros_team_ws_file given. Cannot setup workspace defines."
  fi
  local ros_team_ws_file=$1

  if [ -f "$ros_team_ws_file" ]; then
    new_rtw_file_name="${ros_team_ws_file_name}.bkp-$(ls ${ros_team_ws_file}* | wc -l)"
    echo ""
    cp "$ros_team_ws_file" "$HOME/$new_rtw_file_name"
    notify_user "${ros_team_ws_file_name} already exists. Copied it to ${new_rtw_file_name}."
  else
    print_and_exit "No $ros_team_ws_file found! Please first setup auto sourcing with the \"setup-auto-sourcing\" command."
  fi

  # sed all needed files
  STRINGS_TO_SED=("RosTeamWS_BASE_WS" "RosTeamWS_DISTRO" "RosTeamWS_WS_FOLDER" "RosTeamWS_WS_DOCKER_SUPPORT" "RosTeamWS_DOCKER_TAG")

  for SED_STRING in "${STRINGS_TO_SED[@]}"; do
    sed -i "s/  $SED_STRING/  export $SED_STRING/g" $ros_team_ws_file
  done
}

rtw_setup_ros_team_ws_file "$ros_team_ws_file"
