usage='setup.bash "ros_distro" "ros_ws_suffix" "workspace_folder"'

# Load Framework defines
# script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
# source $script_own_dir/../_RosTeamWs_Defines.bash

# ros distribution name will be set in $ros_distro
check_ros_distro $1

ws_suffix=$2
if [[  "$2" == "-" ]]; then
  ws_suffix=""
else
  ws_suffix="_$2"
fi

ws_folder="$3"
if [ -z "$3" ]; then
  ws_folder="workspace"
fi

unset CMAKE_PREFIX_PATH
unset ROS_PACKAGE_PATH
unset COLCON_WS

if [[ $ros_version == 1 ]]; then

  setup_ros1_exports
  setup_ros1_aliases

  source ~/$ws_folder/ros_ws_$ros_distro$ws_suffix/devel/setup.bash
  source `rospack find intelligent_robotic_automation`/scripts/environment/ros_setup.bash

elif [[ $ros_version == 2 ]]; then

  setup_ros2_exports
  setup_ros2_aliases

  /opt/rti.com/rti_connext_dds-5.3.1/setenv_ros2rti.bash
  # export LANG=de_DE.UTF-8
  WS_FOLDER=""
  WS_FOLDER_1="$HOME/$ws_folder/ros_ws_$ros_distro$ws_suffix"
  WS_FOLDER_2="$HOME/$ws_folder/ros2_ws_$ros_distro$ws_suffix"
  if [ -d "$WS_FOLDER_1" ]; then
    WS_FOLDER=$WS_FOLDER_1
  elif [ -d "$WS_FOLDER_2" ]; then
    WS_FOLDER=$WS_FOLDER_2
  else
    print_and_exit "Neither '$WS_FOLDER_1' nor 'WS_FOLDER_2' exist. Can not find ROS workspace!"
  fi

  export COLCON_WS=$WS_FOLDER
  source "$WS_FOLDER/install/setup.bash"

  echo ""
  echo "RosTeamWS: Sourced file: $WS_FOLDER/install/setup.bash"
fi
