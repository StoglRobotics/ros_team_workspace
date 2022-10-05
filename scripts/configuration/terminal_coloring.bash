#!/bin/bash
#
# Copyright 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
#
# Authors: Manuel Muth, Denis Štogl
#

# BEGIN: Stogl Robotics custom setup for nice colors and showing ROS workspace

# Check this out: https://www.shellhacks.com/bash-colors/
export TERMINAL_COLOR_NC='\e[0m' # No Color
export TERMINAL_COLOR_BLACK='\e[0;30m'
export TERMINAL_COLOR_GRAY='\e[1;30m'
export TERMINAL_COLOR_RED='\e[0;31m'
export TERMINAL_COLOR_LIGHT_RED='\e[1;31m'
export TERMINAL_COLOR_GREEN='\e[0;32m'
export TERMINAL_COLOR_LIGHT_GREEN='\e[1;32m'
export TERMINAL_COLOR_BROWN='\e[0;33m'
export TERMINAL_COLOR_YELLOW='\e[1;33m'
export TERMINAL_COLOR_BLUE='\e[0;34m'
export TERMINAL_COLOR_LIGHT_BLUE='\e[1;34m'
export TERMINAL_COLOR_PURPLE='\e[0;35m'
export TERMINAL_COLOR_LIGHT_PURPLE='\e[1;35m'
export TERMINAL_COLOR_CYAN='\e[0;36m'
export TERMINAL_COLOR_LIGHT_CYAN='\e[1;36m'
export TERMINAL_COLOR_LIGHT_GRAY='\e[0;37m'
export TERMINAL_COLOR_WHITE='\e[1;37m'

export TERMINAL_BG_COLOR_BLACK='\e[40m'
export TERMINAL_BG_COLOR_GRAY='\e[1;40m'
export TERMINAL_BG_COLOR_RED='\e[41m'
export TERMINAL_BG_COLOR_LIGHT_RED='\e[1;41m'
export TERMINAL_BG_COLOR_GREEN='\e[42m'
export TERMINAL_BG_COLOR_LIGHT_GREEN='\e[1;42m'
export TERMINAL_BG_COLOR_BROWN='\e[43m'
export TERMINAL_BG_COLOR_YELLOW='\e[1;43m'
export TERMINAL_BG_COLOR_BLUE='\e[44m'
export TERMINAL_BG_COLOR_LIGHT_BLUE='\e[1;44m'
export TERMINAL_BG_COLOR_PURPLE='\e[45m'
export TERMINAL_BG_COLOR_LIGHT_PURPLE='\e[1;45m'
export TERMINAL_BG_COLOR_CYAN='\e[46m'
export TERMINAL_BG_COLOR_LIGHT_CYAN='\e[1;46m'
export TERMINAL_BG_COLOR_LIGHT_GRAY='\e[47m'
export TERMINAL_BG_COLOR_WHITE='\e[1;47m'

if [ -n "$SSH_CLIENT" ]; then text="-ssh-session"
fi

function parse_git_branch_and_add_brackets {
  gitbranch=`git branch --no-color 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/\1/'`

  if [[ "$gitbranch" != '' ]]; then
    echo "<${gitbranch}"
#   else
#     echo "<no-git-branch"
  fi
}

function set_ros_workspace_color {
  if [[ -n ${ROS_WS} ]]; then

    if [[ $PWD == *"${ROS_WS}"* ]]; then
      color=${TERMINAL_COLOR_CYAN}
    else
      color=${TERMINAL_BG_COLOR_RED}
    fi

    echo -e "${color}"
  fi
}

function parse_ros_workspace {
  if [[ -n ${ROS_WS} ]]; then

    echo "[${ROS_WS##*/}]"
  fi
}

# Version mit time infront of values
# export PS1="\[\e]0;"'$(parse_ros_workspace)'"\a\]\[${TERMINAL_COLOR_LIGHT_GRAY}\]"'[\t]\['"\[${TERMINAL_COLOR_LIGHT_GREEN}\]"'\u\['"\[${TERMINAL_COLOR_LIGHT_GRAY}\]"'@\['"\[${TERMINAL_COLOR_BROWN}\]"'\h\['"\[${TERMINAL_COLOR_YELLOW}\]"'${text}\['"\[${TERMINAL_COLOR_LIGHT_GRAY}\]"':'"\["'$(set_ros_workspace_color)'"\]"'$(parse_ros_workspace)\['"\[${TERMINAL_COLOR_GREEN}\]"'$(parse_git_branch_and_add_brackets)>\['"\[${TERMINAL_COLOR_LIGHT_PURPLE}\]"'\W\['"\[${TERMINAL_COLOR_LIGHT_PURPLE}\]"'$\['"\[${TERMINAL_COLOR_NC}\]"'\[\e[m\] '

export PS1="\[\e]0;"'$(parse_ros_workspace)'"\a\]\[${TERMINAL_COLOR_LIGHT_GREEN}\]"'\u\['"\[${TERMINAL_COLOR_LIGHT_GRAY}\]"'@\['"\[${TERMINAL_COLOR_BROWN}\]"'\h\['"\[${TERMINAL_COLOR_YELLOW}\]"'${text}\['"\[${TERMINAL_COLOR_LIGHT_GRAY}\]"':'"\["'$(set_ros_workspace_color)'"\]"'$(parse_ros_workspace)\['"\[${TERMINAL_COLOR_GREEN}\]"'$(parse_git_branch_and_add_brackets)>\['"\[${TERMINAL_COLOR_LIGHT_PURPLE}\]"'\W\['"\[${TERMINAL_COLOR_LIGHT_PURPLE}\]"'$\['"\[${TERMINAL_COLOR_NC}\]"'\[\e[m\] '


# END: Stogl Robotics custom setup for nice colors and showing ROS workspace
