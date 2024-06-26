# Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

## BEGIN: definitions

# TODO(denis): We have two example teams. On is working with industrial and other with mobile robots
# TEAM_TEAM_NAMES=("Industrial" "Mobile")

# Define a path to a repository with your internal assets and configurations
TEAM_INTERNAL_ASSETS="/opt/RosTeamWS/assets/"

## END: definitions

# TODO(denis): implement here support for internal repos
if [ -f $TEAM_INTERNAL_ASSETS/setup.bash ]; then
  source $TEAM_INTERNAL_ASSETS/setup.bash
fi

## BEGIN: Framework definition adjustable by users
# TBD
#FRAMEWORK_BASE_PATH="/tmp/ros_team_workspace/"

## END: Framework definitions adjustable by users

# BEGIN: Define aliases for standard internal functions
#cd to often used places
alias cd_ros_team_ws='cd $FRAMEWORK_BASE_PATH'
alias cd_rtw='cd $FRAMEWORK_BASE_PATH'
alias cd_rtw_scripts='cd $RosTeamWS_FRAMEWORK_SCRIPTS_PATH'

alias setup_exports=RosTeamWS_setup_exports

alias setup_aliases=RosTeamWS_setup_aliases

alias setup_ros1_exports=RosTeamWS_setup_ros1_exports

alias setup_ros1_aliases=RosTeamWS_setup_ros1_aliases

alias setup_ros2_exports=RosTeamWS_setup_ros2_exports

alias setup_ros2_aliases=RosTeamWS_setup_ros2_aliases
# END: Define aliases for standard functions


# BEGIN: Define aliases for standard scripts
# Change those to your custom ones you would like to use.
alias create-new-package=$RosTeamWS_FRAMEWORK_SCRIPTS_PATH/create-new-package.bash

alias setup-repository=$RosTeamWS_FRAMEWORK_SCRIPTS_PATH/setup-repository.bash

alias setup-repository-ci=$RosTeamWS_FRAMEWORK_SCRIPTS_PATH/setup-repository-ci.bash

alias setup-formatting=$RosTeamWS_FRAMEWORK_SCRIPTS_PATH/setup-formatting.bash

alias setup-ros-workspace=$RosTeamWS_FRAMEWORK_SCRIPTS_PATH/setup-ros-workspace.bash

setup-ros-workspace () {
  source "$RosTeamWS_FRAMEWORK_SCRIPTS_PATH"/setup-ros-workspace.bash
  create_workspace "$@"
}

alias setup-robot-bringup=$RosTeamWS_FRAMEWORK_SCRIPTS_PATH/setup-robot-bringup.bash

alias setup-robot-description=$RosTeamWS_FRAMEWORK_SCRIPTS_PATH/setup-robot-description.bash

# ros2_control
alias ros2_control_setup-hardware-interface-package=$RosTeamWS_FRAMEWORK_SCRIPTS_PATH/ros2_control/setup-hardware-interface-package.bash
alias ros2_control_setup-controller-package=$RosTeamWS_FRAMEWORK_SCRIPTS_PATH/ros2_control/setup-controller-package.bash

# setup auto-sourcing
alias setup-auto-sourcing=$RosTeamWS_FRAMEWORK_SCRIPTS_PATH/setup_auto_sourcing.bash

# VCS Aliases and helpers
function rtw-ws-import () {
  # TODO: if not argument use WS default path for this after #169 is merged
  vcs import --debug -w 1 --input "$*" $ROS_WS/src
}

# Temporary files cleaning
function rtw-find-and-remove-core-files () {
  if [ -z "$1" ]; then
    search_dir="${HOME}"
  else
    search_dir=$1
  fi

  find "${search_dir}" -type f -regex ".*core\.[0-9]+"

  user_decision "Do you want to delete those files?" user_answer
  if [[ " ${positive_answers[*]} " =~ " ${user_answer} " ]]; then
    find "${search_dir}" -type f -regex ".*core\.[0-9]+" -delete
    notify_user "Files ARE deleted!"
  else
    notify_user "Files NOT deleted!"
  fi
}

# Team General aliases and functions
function generate_gif_from_video {

  if [ -z "$1" ]; then
    print_and_exit "File name has to be defined!"
  fi

  ffmpeg -i $1 -filter_complex "[0:v] fps=12,scale=w=960:h=-1,split [a][b];[a] palettegen [p];[b][p] paletteuse" $1.gif
}

# END Define aliases for standard functions
