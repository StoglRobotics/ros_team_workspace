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
# Authors: Manuel Muth, Denis Štogl, Atticus Russell
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

function get_gitbranch {
  echo `git branch --no-color 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/\1/'`
}

function set_git_color {
  if [[ "$(get_gitbranch)" != '' ]]; then
    # Get the status of the repo and chose a color accordingly
    local STATUS=`LANG=en_GB git status 2>&1`
    if [[ "$STATUS" != *'working tree clean'* ]]; then
      # red if need to commit
      color=${TERMINAL_COLOR_RED}
    else
      if [[ "$STATUS" == *'Your branch is ahead'* ]]; then
        # yellow if need to push
        color=${TERMINAL_COLOR_YELLOW}
      else
        # else green
        color=${TERMINAL_COLOR_GREEN}
      fi
    fi

    echo -e "${color}"
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


# Start with an empty PS1
PS1=""

# Add a newline character to move to the next line and visually separate from the previous terminal command
PS1+="\n"

# Set the terminal title to the current ROS workspace
PS1+="\[\e]0;\$(parse_ros_workspace)\a\]"

# Set the user text color to light green
PS1+="\[${TERMINAL_COLOR_LIGHT_GREEN}\]\u"

# Add a light gray '@'
PS1+="\[${TERMINAL_COLOR_LIGHT_GRAY}\]@"

# Set the host name color to brown
PS1+="\[${TERMINAL_COLOR_BROWN}\]\h"

# Add a yellow '-ssh-session' if SSH client is connected
PS1+="\[${TERMINAL_COLOR_YELLOW}\]${text}"

# separate parts of header with a space with no highlighting
PS1+="\[${TERMINAL_COLOR_NC}\] "

# Set the color of the ROS workspace
PS1+="\[\$(set_ros_workspace_color)\]"

# Add the name of the ROS workspace
PS1+="\$(parse_ros_workspace)"

# separate parts of header with a space with no highlighting
PS1+="\[${TERMINAL_COLOR_NC}\] "

# if in a git repo add the branch name in brackets
# conditionals *in* the PS are needed so its evaluated each time the PS is printed
PS1+="\[\$([[ \$(get_gitbranch) != '' ]] && echo -n \$(set_git_color))\]"
PS1+="\$(if [[ \$(get_gitbranch) != '' ]]; then echo -n '<' && echo -n \$(get_gitbranch) && echo -n '>'; fi)"

# Add a newline character to move to the next line
PS1+="\n"

# Add the full file path relative to the user's home directory
PS1+="\[${TERMINAL_COLOR_LIGHT_PURPLE}\]\w"

# Add a newline character to move to the next line
PS1+="\n"

# Add the time
PS1+="\[${TERMINAL_COLOR_NC}\]\D{%T}"

# separate parts of header with a space with no highlighting
PS1+="\[${TERMINAL_COLOR_NC}\] "

# Add the prompt symbol ('$')
PS1+="\[${TERMINAL_COLOR_NC}\]\$"

# Add a space after the prompt symbol
PS1+=" "

# Reset the color
PS1+="\[${TERMINAL_COLOR_NC}\]\[\e[m\]"

# Export the PS1 variable
export PS1


# END: Stogl Robotics custom setup for nice colors and showing ROS workspace
