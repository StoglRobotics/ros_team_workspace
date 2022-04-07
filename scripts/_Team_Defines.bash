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

## BEGIN: Team specific definitions

DEFAULT_ROS_DISTRO="rolling"
DEFAULT_ROS_VERSION=2

# TODO(denis): We have two example teams. On is working with industrial and other with mobile robots
# TEAM_TEAM_NAMES=("Industrial" "Mobile")

TEAM_LICENSE="Apache License 2.0"

TEAM_REPOSITORY_SERVER="https://github.com"

TEAM_PRIVATE_CONFIG_PATH=""

# Define a path to a repository with your internal assets and configurations
TEAM_INTERNAL_ASSETS="/opt/RosTeamWS/assets/"

## END: definitions

# TODO(denis): implement here support for internal repos
source $TEAM_INTERNAL_ASSETS/setup.bash


## BEGIN: Framework definition adjustable by users
# TBD
#FRAMEWORK_BASE_PATH="/tmp/ros_team_workspace/"

## END: Framework definitions adjustable by users

# Check ROS and Load base paths
check_ros_distro $DEFAULT_ROS_DISTRO

# BEGIN: Define aliases for standard internal functions

alias setup_exports=RosTeamWS_setup_exports

alias setup_aliases=RosTeamWS_setup_aliases

alias setup_ros1_exports=RosTeamWS_setup_ros1_exports

alias setup_ros1_aliases=RosTeamWS_setup_ros1_aliases

alias setup_ros2_exports=RosTeamWS_setup_ros2_exports

alias setup_ros2_aliases=RosTeamWS_setup_ros2_aliases
# END: Define aliases for standard functions


# BEGIN: Define aliases for standard scripts
# Change those to your custom ones you would like to use.

create-new-package () {
    "$RosTeamWS_FRAMEWORK_SCRIPTS_PATH"/create-new-package.bash create_workspace "$@"
}

create-new-package-docker () {
    "$RosTeamWS_FRAMEWORK_SCRIPTS_PATH"/create-new-package.bash create_workspace_docker "$@"
}

alias setup-repository=$RosTeamWS_FRAMEWORK_SCRIPTS_PATH/setup-repository.bash

alias setup-repository-ci=$RosTeamWS_FRAMEWORK_SCRIPTS_PATH/setup-ci-config.bash

alias setup-ros-workspace=$RosTeamWS_FRAMEWORK_SCRIPTS_PATH/setup-ros-workspace.bash

alias setup-robot-bringup=$RosTeamWS_FRAMEWORK_SCRIPTS_PATH/setup-robot-bringup.bash

alias setup-robot-description=$RosTeamWS_FRAMEWORK_SCRIPTS_PATH/setup-robot-description.bash

# ros2_control
alias ros2_control_setup-hardware-interface-package=$RosTeamWS_FRAMEWORK_SCRIPTS_PATH/ros2_control/setup-hardware-interface-package.bash
alias ros2_control_setup-controller-package=$RosTeamWS_FRAMEWORK_SCRIPTS_PATH/ros2_control/setup-controller-package.bash

# END Define aliases for standard functions
