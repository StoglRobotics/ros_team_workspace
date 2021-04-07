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

# TODO(denis): add this into setup.bash
function RosTeamWS_setup_aliases {

# ROS
  alias rosd="cd \$ROS_WS"
  alias rosds="cd \$ROS_WS/src"
  alias rosdb="cd \$ROS_WS/build"
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
  alias cbr="colcon_build_release"
  alias cbup="colcon_build_up_to"

  alias ct="colcon_test"
  alias ctup="colcon_test_up_to"

  alias ctres="colcon_test_results"

  alias ca="colcon_all"
  alias caup="colcon_all_up_to"
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
  colcon_helper_ros2 "colcon build --symlink-install" "$*"
}

function colcon_build_up_to {
  colcon_helper_ros2_up_to "colcon build --symlink-install" "$*"
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
    colcon test-result
  else
    colcon test-result | grep "$*"
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

## END: Default Framework Definitions


## BEGIN: Framework functions
function print_and_exit {
  message=$1
  echo ""
  echo "$message  !!Exiting..."
  if [ ! -z "$2" ]; then
    echo ""
    echo "Usage: $2"
  fi
  exit
}

function framework_default_paths {
  ros_distro=$1

  FRAMEWORK_NAME="ros_team_workspace"
  FRAMEWORK_BASE_PATH=${FRAMEWORK_BASE_PATH:=/opt/RosTeamWS}
  FRAMEWORK_PACKAGE_PATH="$FRAMEWORK_BASE_PATH/ros_ws_$ros_distro/src/$FRAMEWORK_NAME"

  if [ ! -d "$FRAMEWORK_PACKAGE_PATH" ]; then
    FRAMEWORK_PACKAGE_PATH=$FRAMEWORK_MAIN_PATH
  fi
  RosTeamWS_FRAMEWORK_SCRIPTS_PATH="$FRAMEWORK_PACKAGE_PATH/scripts/"

  # Script-specific variables
  PACKAGE_TEMPLATES="$FRAMEWORK_PACKAGE_PATH/templates/package"
  ROBOT_DESCRIPTION_TEMPLATES="$FRAMEWORK_PACKAGE_PATH/templates/robot_description"
  ROS2_CONTROL_TEMPLATES="$FRAMEWORK_PACKAGE_PATH/templates/ros2_control"
  ROS2_CONTROL_HW_ITF_TEMPLATES="$FRAMEWORK_PACKAGE_PATH/templates/ros2_control/hardware"
  ROS2_CONTROL_CONTROLLER_TEMPLATES="$FRAMEWORK_PACKAGE_PATH/templates/ros2_control/controller"
  LICENSE_TEMPLATES="$FRAMEWORK_PACKAGE_PATH/templates/licenses"
}

function check_ros_distro {
  ros_distro=$1
  if [ -z "$1" ]; then
    ros_distro=$DEFAULT_ROS_DISTRO
    echo "No ros_distro defined. Using default: '$ros_distro'"
    if [ ! -d "/opt/ros/$ros_distro" ]; then
      print_and_exit "FATAL: ROS '$ros_distro' not installed on this computer! Exiting..."
#             echo "FATAL: ROS '$ros_distro' not installed on this computer! Exiting..."
#             exit
    fi
    read -p "Press <ENTER> to continue or <CTRL>+C to exit."
  fi

  if [ ! -d "/opt/ros/$ros_distro" ]; then
    print_and_exit "FATAL: ROS '$ros_distro' not installed on this computer! Exiting..."
#         echo "FATAL: ROS '$ros_distro' not installed on this computer! Exiting..."
#         exit
  fi

  ros_version=$DEFAULT_ROS_VERSION
  if [[ $ros_distro == "foxy" ]]; then
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
#   while [[ "$bn" != "src" ]]; do
#     cd ..
#     bn=`basename "$PWD"`
#     path="$bn/$path"
#   done
#   cd ..
  cd $ROS_WS
  colcon build --symlink-install --packages-up-to $pkg_name
  source install/setup.bash
  if [[ "$test" == "yes" ]]; then
    colcon test --packages-select $pkg_name
    colcon test-result | grep $pkg_name
  fi
  cd $path
}

# END: Framework functions

FRAMEWORK_MAIN_PATH="$(RosTeamWS_script_own_dir)/../"
