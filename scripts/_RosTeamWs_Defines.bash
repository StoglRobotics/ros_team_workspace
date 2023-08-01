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

# used for user decisions
if [ -z "$positive_answers" ]; then
  readonly positive_answers=("yes")
fi
if [ -z "$negative_answers" ]; then
  readonly negative_answers=("no")
fi
if [ -z "$rtw_accepted_answers" ]; then
  readonly rtw_accepted_answers=("${positive_answers[@]}" "${negative_answers[@]}")
fi

# All the possible supported ros distributions supported by rtw
if [ -z "$rtw_supported_ros_distributions" ]; then
  readonly rtw_supported_ros_distributions=("noetic" "foxy" "galactic" "humble" "iron" "rolling")
fi

# This needs to be set for every branch
# Check the set_supported_versions functions below and update as fit.
RTW_BRANCH_ROS_DISTRO="rolling"

# Based on the RTW_BRANCH_ROS_DISTRO the supported_ros_distributions and ros versions are set.
# This limits the templates you can generate with this branch of the RosTeamWs framework.
function set_supported_versions {
  case $RTW_BRANCH_ROS_DISTRO in
  noetic)
    supported_ros_distributions=("noetic")
    ros_version=1
    ;;
  foxy)
    supported_ros_distributions=("foxy" "galactic")
    ros_version=2
    ;;
  galactic)
    supported_ros_distributions=("foxy" "galactic")
    ros_version=2
    ;;
  humble)
    supported_ros_distributions=("humble")
    ros_version=2
    ;;
  rolling)
    supported_ros_distributions=("iron" "rolling")
    ros_version=2
    ;;
  *)
    print_and_exit "FATAL: the RTW_BRANCH_ROS_DISTRO in the _RosTeamWs_Defines.bash is set to a not supported ros distribution. This should never happen. Please set to the correct branch name which can be either of:${rtw_supported_ros_distributions}."
    ;;
esac

}
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

# function which prints a notification in predefined color scheme
# $1 - notification = The message which gets print to the commandline
function notify_user {
  notification=$1

  echo -e "${RTW_COLOR_NOTIFY_USER}${notification}${TERMINAL_COLOR_NC}"
}

## BEGIN: Framework functions
# Parameters:
# *message* - message to display
# *usage* - command usage description
function print_and_exit {
  message=$1

  echo ""
  echo -e "${RTW_COLOR_ERROR}$message!!! Exiting...${TERMINAL_COLOR_NC}"
  if [ ! -z "$2" ]; then
    echo ""
    echo -e "${TERMINAL_COLOR_USER_NOTICE}Usage: '$2'${TERMINAL_COLOR_NC}"
  fi
  echo -e "${TERMINAL_COLOR_USER_CONFIRMATION}Error has happened. Press <CTRL> + C two times...${TERMINAL_COLOR_NC}"
  read -p ""
  exit 1
}

# If the answer selected by user is in the ${rtw_accepted_answers} array
# true is returned else false.
#
# $1 - user_answer = The ansewer given by the user.
function is_accepted_user_answer {
  local user_answer=$1

  if [[ " ${rtw_accepted_answers[*]} " =~ " ${user_answer} " ]]; then
    return
  fi
  false
}

# Let's the user decide what he wants to do. The decision (question) is repeated
# till the user gives an accepted answer. Accepted answers are defined in ${rtw_accepted_answers}.
# Answers can either be positive or negative.
#
# $1 - decision = The decision (question) the user has to decide about.
# $2 - user_answer = The answer given by the user.
function user_decision {
  local decision=$1
  user_answer=$2

  echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}${decision}${TERMINAL_COLOR_NC} [${rtw_accepted_answers[*]}]"
  read user_answer

  while ! $(is_accepted_user_answer "$user_answer");
  do
    echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}${decision}${TERMINAL_COLOR_NC} ${TERMINAL_COLOR_USER_NOTICE}Please type one of the following:${TERMINAL_COLOR_NC} [${rtw_accepted_answers[*]}]"
    read user_answer
  done
}

# function which prints a notification in predefined color scheme and wait for user confirmation
# $1 - notification = The message which gets print to the commandline
function user_confirmation {
  notification=$1

  echo -e "${TERMINAL_COLOR_USER_CONFIRMATION}${notification}${TERMINAL_COLOR_NC}"
  read
}

function set_framework_default_paths {
  FRAMEWORK_NAME="ros_team_workspace"
  # readlink prints resolved symbolic links or canonical file names -> the "dir/dir_2/.." becomes "dir"
  FRAMEWORK_BASE_PATH="$(readlink -f "$(RosTeamWS_script_own_dir)"/..)"

  RosTeamWS_FRAMEWORK_SCRIPTS_PATH="$FRAMEWORK_BASE_PATH/scripts"
  RosTeamWS_FRAMEWORK_OS_CONFIGURE_PATH="$RosTeamWS_FRAMEWORK_SCRIPTS_PATH/os_configure"
  # Script-specific variables
  PACKAGE_TEMPLATES="$FRAMEWORK_BASE_PATH/templates/package"
  ROBOT_DESCRIPTION_TEMPLATES="$FRAMEWORK_BASE_PATH/templates/robot_description"
  ROS2_CONTROL_TEMPLATES="$FRAMEWORK_BASE_PATH/templates/ros2_control"
  ROS2_CONTROL_HW_ITF_TEMPLATES="$ROS2_CONTROL_TEMPLATES/hardware"
  ROS2_CONTROL_CONTROLLER_TEMPLATES="$ROS2_CONTROL_TEMPLATES/controller"
  LICENSE_TEMPLATES="$FRAMEWORK_BASE_PATH/templates/licenses"
  DOCKER_TEMPLATES="$FRAMEWORK_BASE_PATH/templates/docker"
}

function is_valid_ros_distribution {
    local ros_distribution=$1
    declare -a supported_ros_versions=("${!2}")

  if [[ " ${supported_ros_versions[*]} " =~ " ${ros_distribution} " ]]; then
    return
  fi

  false
}

function check_ros_distro {
  ros_distro=$1
  if [ -z "$2" ]; then
    use_docker="false"
  else
    use_docker=$2
  fi

  # check if the given distribution is a distribution supported by rtw
  while ! is_valid_ros_distribution "$ros_distro" rtw_supported_ros_distributions[@];
  do
    if [ -z "$ros_distro" ]; then
      echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}No ROS distribution provided. Please choose either of the following:${rtw_supported_ros_distributions[*]}"
    else
      echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}The ROS distribution {${ros_distro}} you chose is not supported by RosTeamWS. Please choose either of the following:${rtw_supported_ros_distributions[*]}"
    fi
      read ros_distro
  done

  # inside docker container we don't need to check if ros distro is present on system
  if [ "${use_docker}" != "true" ]; then
    if [ ! -d "/opt/ros/$ros_distro" ]; then
      local upper_case=$(echo $ros_distro | tr '[:lower:]' '[:upper:]')
      local alternative_ros_location=ALTERNATIVE_ROS_${upper_case}_LOCATION
      if [ ! -f "${!alternative_ros_location}/setup.bash" ]; then
        notify_user "You are possibly trying to run unsupported ROS distro ('$ros_distro') for your version of Ubuntu. Please set ${alternative_ros_location} variable, e.g., 'export ${alternative_ros_location}=/opt/ros/rolling'. The best is to add that line somewhere at the beginning of the '~/.ros_team_ws_rc' file."

        print_and_exit "FATAL: ROS '$ros_distro' not installed on this computer! Exiting..."
      else
        user_decision "Using ${!alternative_ros_location} for ${ros_distro}." user_answer
        # check if the chosen ros-distro location is correct.
        if [[ " ${negative_answers[*]} " =~ " ${user_answer} " ]]; then
          print_and_exit "Please set ${alternative_ros_location} to the correct location. Exiting..."
        else
          source "${!alternative_ros_location}/setup.bash"
        fi
      fi
    else
      # source ros to have access to ros cli commands for other functions
      source "/opt/ros/${ros_distro}/setup.bash"
    fi
  fi
}

function set_ros_version_for_distro {
  local ros_distribution=$1

  case $ros_distribution in
    noetic)
      ros_version=1
      ;;
    foxy)
      ros_version=2
      ;;
    galactic)
      ros_version=2
      ;;
    humble)
      ros_version=2
      ;;
    iron)
      ros_version=2
      ;;
    rolling)
      ros_version=2
      ;;
    *)
      print_and_exit "FATAL: For the chosen ros distribution ${ros_distribution} there is no ros version."
      ;;
  esac
}

function check_and_set_ros_distro_and_version {
  ros_distro=$1
  if [ -z "$2" ]; then
    use_docker="false"
  else
    use_docker=$2
  fi

  check_ros_distro "${ros_distro}" "${use_docker}"
  set_ros_version_for_distro "${ros_distro}"
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

  if [ -z "$ROS_WS" ]; then
    notify_user "Can not compile: No ROS_WS variable set. Trying to guess by sourced workspace."
    sourced_ws_dirname=$(dirname "$COLCON_PREFIX_PATH")
    if [ -z "$sourced_ws_dirname" ]; then
      print_and_exit "Error no workspace sourced. Please source the workspace folder and compile manually."
    fi

    user_decision "Is \"${sourced_ws_dirname} the correct sourced workspace?"
    if [[ " ${negative_answers[*]} " =~ " ${user_answer} " ]]; then
      print_and_exit "Aborting. Not the correct workspace sourced. Please source the correct workspace folder and compile manually."
    fi
  # set ROS_WS ourselves, as other functions need it later and will fail/use wrong path otherwise
  export ROS_WS="$sourced_ws_dirname"
  fi

  cd "$ROS_WS" || { print_and_exit "Could not change directory to workspace:\"$ROS_WS\". Check your workspace names in .ros_team_ws_rc and try again."; return 1; }

  colcon_build_up_to $pkg_name
  source install/setup.bash
  if [[ "$test" == "yes" ]]; then
    colcon_test_up_to $pkg_name
    colcon_test_results | grep $pkg_name
  fi
}

# END: Framework functions
