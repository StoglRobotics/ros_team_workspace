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

## BEGIN: Default RosTeamWS Definitions

function RosTeamWS_script_own_dir {
  echo "$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
}

function RosTeamWS_setup_exports {

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
  export RAW_TERMINAL_COLOR_NC=$'\e[0m' # No Color
  export RAW_TERMINAL_COLOR_BLACK=$'\e[0;30m'
  export RAW_TERMINAL_COLOR_GRAY=$'\e[1;30m'
  export RAW_TERMINAL_COLOR_RED=$'\e[0;31m'
  export RAW_TERMINAL_COLOR_LIGHT_RED=$'\e[1;31m'
  export RAW_TERMINAL_COLOR_GREEN=$'\e[0;32m'
  export RAW_TERMINAL_COLOR_LIGHT_GREEN=$'\e[1;32m'
  export RAW_TERMINAL_COLOR_BROWN=$'\e[0;33m'
  export RAW_TERMINAL_COLOR_YELLOW=$'\e[1;33m'
  export RAW_TERMINAL_COLOR_BLUE=$'\e[0;34m'
  export RAW_TERMINAL_COLOR_LIGHT_BLUE=$'\e[1;34m'
  export RAW_TERMINAL_COLOR_PURPLE=$'\e[0;35m'
  export RAW_TERMINAL_COLOR_LIGHT_PURPLE=$'\e[1;35m'
  export RAW_TERMINAL_COLOR_CYAN=$'\e[0;36m'
  export RAW_TERMINAL_COLOR_LIGHT_CYAN=$'\e[1;36m'
  export RAW_TERMINAL_COLOR_LIGHT_GRAY=$'\e[0;37m'
  export RAW_TERMINAL_COLOR_WHITE=$'\e[1;37m'

  ## Define semantics of each color
  export TERMINAL_COLOR_USER_NOTICE=${TERMINAL_COLOR_YELLOW}
  export TERMINAL_COLOR_USER_INPUT_DECISION=${TERMINAL_COLOR_PURPLE}
  export TERMINAL_COLOR_USER_CONFIRMATION=${TERMINAL_COLOR_BLUE}
  export RTW_COLOR_NOTIFY_USER=${TERMINAL_COLOR_YELLOW}
  export RTW_COLOR_ERROR=${TERMINAL_COLOR_RED}
}

# TODO(denis): add this into setup.bash
function RosTeamWS_setup_aliases {

# ROS
  alias rosd="cd \$ROS_WS"
  alias rosds="cd \$ROS_WS/src"
  alias rosdb="cd \$ROS_WS/build"
  alias rosdi="cd \$ROS_WS/install"

}

function RosTeamWS_setup_ros1_exports {

export ROSCONSOLE_FORMAT='[${severity}] [${walltime}: ${logger}] [${node}@${file}.${function}:${line}]: ${message}'
export ROSCONSOLE_CONFIG_FILE='~/workspace/ros_ws/rosconsole.config'

}

function RosTeamWS_setup_ros1_aliases {

# ROS
  alias rosdd="cd \$ROS_WS/devel"

# Catkin
  alias cb="catkin build"

}


function RosTeamWS_setup_ros2_exports {

  export RTI_LICENSE_FILE=/opt/rti.com/rti_connext_dds-5.3.1/rti_license.dat

}

function RosTeamWS_setup_ros2_aliases {

# ROS
  alias rosdi="cd \$ROS_WS/install"

# COLCON
  alias cb="colcon_build"
  alias cbd="colcon_build_debug"
  alias cbr="colcon_build_release"
  alias cbup="colcon_build_up_to"

  alias ct="colcon_test"
  alias ctup="colcon_test_up_to"

  alias ctres="colcon_test_results"

  alias ca="colcon_all"
  alias caup="colcon_all_up_to"

  alias crm="colcon_remove"
}


## some colcon helpers
function colcon_helper_ros2 {
  if [ -z "$1" ]; then
    print_and_exit "This should never happen. Check your helpers definitions!"
  fi

  cd $ROS_WS

  CMD="$1"
  if [ -z "$2" ]; then
    $CMD
  else
    $CMD --packages-select $2
  fi

  cd -
}

function colcon_helper_ros2_up_to {
  if [ -z "$1" ]; then
    print_and_exit "This should never happen. Check your helpers definitions!"
  fi

  cd $ROS_WS

  CMD="$1"
  if [ -z "$2" ]; then
    print_and_exit "You should provide package for this command!"
  else
    $CMD --packages-up-to $2
  fi

  cd -
}

function colcon_build {
  colcon_helper_ros2 "colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"  "$*"
}

function colcon_build_up_to {
  colcon_helper_ros2_up_to "colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo" "$*"
}

function colcon_build_debug {
  colcon_helper_ros2 "colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug" "$*"
}

function colcon_build_release {
  colcon_helper_ros2 "colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release" "$*"
}

function colcon_test {
  colcon_helper_ros2 "colcon test" "$*"
}

function colcon_test_up_to {
  colcon_helper_ros2_up_to "colcon test" "$*"
}

function colcon_test_results {
  cd $ROS_WS
  if [ -z "$1" ]; then
    colcon test-result --all
  else
    colcon test-result --all | grep "$*"
  fi
  cd -
}

function colcon_all {
  colcon_build "$*"
  colcon_test "$*"
  colcon_test_results "$*"
}

function colcon_all_up_to {
  colcon_build_up_to "$*"
  colcon_test_up_to "$*"
  colcon_test_results "$*"
}

function colcon_remove {
  cd $ROS_WS
  if [ -z "$1" ]; then
    /bin/rm -rf build install log
  else
    for package in "$*"; do
      /bin/rm -rf build/${package} install/${package}
    done
  fi
  cd -
}

## END: Default Framework Definitions


## BEGIN: Framework functions
# Parameters:
# *message* - message to display
# *usage* - command usage description
function print_and_exit {
  # Get color definitions
  RosTeamWS_setup_exports

  message=$1
  echo ""
  echo -e "${RTW_COLOR_ERROR}$message!!! Exiting...${TERMINAL_COLOR_NC}"
  if [ ! -z "$2" ]; then
    echo ""
    echo -e "${TERMINAL_COLOR_USER_NOTICE}Usage: '$2'${TERMINAL_COLOR_NC}"
  fi
  echo -e "${TERMINAL_COLOR_BLUE}Error has happened. Press <CTRL> + C two times...${TERMINAL_COLOR_NC}"
  read -p ""
  exit 1
}

function framework_default_paths {
  ros_distro=$1

  FRAMEWORK_NAME="ros_team_workspace"

  # if we suppose a structure like
  # /opt/RosTeamWS/ros_ws_<ros_distro>/src/ros_team_ws/...
  # then FRAMEWORK_BASE_PATH should be /opt/RosTeamWS/
  FRAMEWORK_BASE_PATH="$(RosTeamWS_script_own_dir)/../../../.."
  FRAMEWORK_PACKAGE_PATH="$FRAMEWORK_BASE_PATH/ros_ws_$ros_distro/src/$FRAMEWORK_NAME"

  if [ ! -d "$FRAMEWORK_PACKAGE_PATH" ]; then
    FRAMEWORK_PACKAGE_PATH=$FRAMEWORK_MAIN_PATH
  fi
  RosTeamWS_FRAMEWORK_SCRIPTS_PATH="$FRAMEWORK_PACKAGE_PATH/scripts/"

  # Script-specific variables
  PACKAGE_TEMPLATES="$FRAMEWORK_PACKAGE_PATH/templates/package"
  ROBOT_DESCRIPTION_TEMPLATES="$FRAMEWORK_PACKAGE_PATH/templates/robot_description"
  ROS2_CONTROL_TEMPLATES="$FRAMEWORK_PACKAGE_PATH/templates/ros2_control"
  ROS2_CONTROL_HW_ITF_TEMPLATES="$ROS2_CONTROL_TEMPLATES/hardware"
  ROS2_CONTROL_CONTROLLER_TEMPLATES="$ROS2_CONTROL_TEMPLATES/controller"
  LICENSE_TEMPLATES="$FRAMEWORK_PACKAGE_PATH/templates/licenses"
  DOCKER_TEMPLATES="$FRAMEWORK_PACKAGE_PATH/templates/docker"
}

function check_ros_distro {
  ros_distro=$1
  if [ -z "$1" ]; then
    if [ -n "$DEFAULT_ROS_DISTRO" ]; then
      ros_distro=$DEFAULT_ROS_DISTRO
      echo "No ros_distro defined. Using default: '$ros_distro'"
      if [ ! -d "/opt/ros/$ros_distro" ]; then
        print_and_exit "FATAL: ROS '$ros_distro' not installed on this computer! Exiting..."
  #             echo "FATAL: ROS '$ros_distro' not installed on this computer! Exiting..."
  #             exit
      fi
      read -p "Press <ENTER> to continue or <CTRL>+C to exit."
    else
      return 2>/dev/null
    fi
  fi

  if [ ! -d "/opt/ros/$ros_distro" ]; then
    print_and_exit "FATAL: ROS '$ros_distro' not installed on this computer! Exiting..."
#         echo "FATAL: ROS '$ros_distro' not installed on this computer! Exiting..."
#         exit
  fi

  ros_version=$DEFAULT_ROS_VERSION
  if [[ $ros_distro == "foxy" ]]; then
    ros_version=2
  elif [[ $ros_distro == "galactic" ]]; then
    ros_version=2
  elif [[ $ros_distro == "humble" ]]; then
    ros_version=2
  elif [[ $ros_distro == "rolling" ]]; then
    ros_version=2
  elif [[ $ros_distro == "noetic" ]]; then
    ros_version=1
  fi

  framework_default_paths $ros_distro
}

# first param is package name, second (yes/no) for executing tests
function compile_and_source_package {
  pkg_name=$1
  if [ -z "$1" ]; then
    print_and_exit "No package to compile provided. Exiting..."
  fi
  test=$2
  if [ -z "$2" ]; then
    test="no"
  fi

  bn=`basename "$PWD"`
  path=$bn
  cd $ROS_WS

  colcon_build_up_to $pkg_name
  source install/setup.bash
  if [[ "$test" == "yes" ]]; then
    colcon_test_up_to $pkg_name
    colcon_test_results | grep $pkg_name
  fi
#   cd $path
}

# END: Framework functions

FRAMEWORK_MAIN_PATH="$(RosTeamWS_script_own_dir)/.."
