usage='setup.bash "ros_distro" "workspace_folder" "ros_ws_prefix" "ros_ws_suffix"'

# set -e

# # # keep track of the last executed command
# trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
# # # echo an error message before exiting
# trap 'echo "\"${last_command}\" command filed with exit code $?."' EXIT

# read -p "Starting..."

# Load Framework defines
script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $script_own_dir/../_RosTeamWs_Defines.bash
source $script_own_dir/../_RosTeamWs_Docker_Defines.bash
source $script_own_dir/../_Team_Defines.bash

# ros distribution name will be set in $ros_distro
# RosTeamWS_WS_DOCKER_SUPPORT is set via .ros_team_ws_rc
check_and_set_ros_distro_and_version $1 $RosTeamWS_WS_DOCKER_SUPPORT

ws_folder="$2"
if [ "$2" == "-" ]; then
  ws_folder="workspace"
fi

ws_prefix=$3
if [[ "$3" == "-" ]]; then
  ws_prefix=""
else
  ws_prefix="$3_"
fi

ws_suffix=$4
if [[ "$4" == "-" ]]; then
  ws_suffix=""
else
  ws_suffix="_$4"
fi

unset CMAKE_PREFIX_PATH
unset ROS_PACKAGE_PATH
unset ROS_WS

if [[ $ros_version == 1 ]]; then

  setup_exports
  setup_aliases
  setup_ros1_exports
  setup_ros1_aliases

  WS_FOLDER=""
  ## FIXME: 0-1 are deprecated
  WS_FOLDER_0="$HOME/$ws_folder/$ros_distro$ws_suffix"
  WS_FOLDER_1="$HOME/$ws_folder/ros_ws_$ros_distro$ws_suffix"

  WS_FOLDER_3="$HOME/$ws_folder/$ws_prefix$ros_distro$ws_suffix"

  if [ -d "$ws_folder" ]; then
    WS_FOLDER=$ws_folder
  elif [ -d "$WS_FOLDER_0" ]; then
    WS_FOLDER=$WS_FOLDER_0
  elif [ -d "$WS_FOLDER_1" ]; then
    WS_FOLDER=$WS_FOLDER_1
  elif [ -d "$WS_FOLDER_3" ]; then
    WS_FOLDER=$WS_FOLDER_3
  else
    print_and_exit "Neither '$WS_FOLDER_0', '$WS_FOLDER_1', nor '$WS_FOLDER_3' exist. Can not find ROS workspace!"
  fi

  export ROS_WS=$WS_FOLDER
  source "$WS_FOLDER/devel/setup.bash"

  echo ""
  echo "RosTeamWS: Sourced file: $WS_FOLDER/devel/setup.bash"


elif [[ $ros_version == 2 ]]; then

  setup_exports
  setup_aliases
  setup_ros2_exports
  setup_ros2_aliases

  #/opt/rti.com/rti_connext_dds-5.3.1/setenv_ros2rti.bash
  # export LANG=de_DE.UTF-8
  WS_FOLDER=""
  ## FIXME: 0-2 are deprecated
  WS_FOLDER_0="$HOME/$ws_folder/$ros_distro$ws_suffix"
  WS_FOLDER_1="$HOME/$ws_folder/ros_ws_$ros_distro$ws_suffix"
  WS_FOLDER_2="$HOME/$ws_folder/ros2_ws_$ros_distro$ws_suffix"

  WS_FOLDER_3="$HOME/$ws_folder/$ws_prefix$ros_distro$ws_suffix"

  if [ -d "$ws_folder" ]; then
    WS_FOLDER=$ws_folder
  elif [ -d "$WS_FOLDER_0" ]; then
    WS_FOLDER=$WS_FOLDER_0
  elif [ -d "$WS_FOLDER_1" ]; then
    WS_FOLDER=$WS_FOLDER_1
  elif [ -d "$WS_FOLDER_2" ]; then
    WS_FOLDER=$WS_FOLDER_2
  elif [ -d "$WS_FOLDER_3" ]; then
    WS_FOLDER=$WS_FOLDER_3
  else
    print_and_exit "Neither '$WS_FOLDER_0', '$WS_FOLDER_1', '$WS_FOLDER_2' nor '$WS_FOLDER_3' exist. Can not find ROS workspace!"
  fi

  if [ -n "$RosTeamWS_DOCKER_TAG" ] && [ "$RosTeamWS_DOCKER_TAG" != "-" ]; then
    echo -e "${TERMINAL_COLOR_BLUE}RosTeamWS: The workspace uses docker container '$RosTeamWS_DOCKER_TAG'."
  fi

  export ROS_WS=$WS_FOLDER
  # TODO: COLCON_WS is deprecated!!
  export COLCON_WS=$ROS_WS
  FILE_TO_SOURCE="$WS_FOLDER/install/setup.bash"
  if [ ! -f "$FILE_TO_SOURCE" ]; then
    echo -e "${TERMINAL_COLOR_YELLOW}'$FILE_TO_SOURCE' not found! Sourcing base workspace for '$ros_distro'."
    FILE_TO_SOURCE="/opt/ros/$ros_distro/setup.bash"
  fi
  source "$FILE_TO_SOURCE"

  echo ""
  echo -e "${TERMINAL_COLOR_BLUE}RosTeamWS: Sourced file: ${FILE_TO_SOURCE}${TERMINAL_COLOR_NC}"
fi
