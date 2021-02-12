#!/bin/bash

# workspace folder is relative to your home

usage='Usage: setup-ros-workspace.bash ROS_DISTRO WS_SUFFIX WS_FOLDER'

# Load Framework defines
script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $script_own_dir/_RosTeamWs_Defines.bash

# ros distribution name will be set in $ros_distro
check_ros_distro $1

ros_ws_suffix="$2"
if [ -z "$2" ]; then
  ros_ws_suffix=""
  echo "No ros_ws_suffix used..."
fi

ws_folder="$3"
if [ -z "$3" ]; then
  ws_folder="workspace"
  echo "Using default '~/workspace' folder to setup ros workspace"
fi

# TODO: Write this automatically from the user's definitions
echo "Please choose which workspace should be basis for yours:"
echo "(0) <Use current sourced workspace>"
# echo "(1) Industrial"
# echo "(2) Mobile"
read choice

if [ -z "$choice" ]; then
  print_and_exit "No workspace is chosen!"
fi

case "$choice" in
# "1")
#    base_ws=Industrial
#    ;;
# "2")
#    base_ws=Mobile
#    ;;
"0")
   base_ws="<current>"
   ;;
*)
  echo "No workspace chosen! Exiting..."
  exit
esac

# TODO: Add here output of the <current> WS
echo ""
read -p "ATTENTION: Creating a new workspace in folder '$ws_folder' for ROS '$ros_distro' (ROS$ros_version) with suffix '$ros_ws_suffix' using '$base_ws' as base workspace. Press <ENTER> to continue..."

# Create and initalise ROS-Workspace
if [[ $base_ws != "<current>" ]]; then
  if [[ $ros_version == 1 ]]; then
    source $FRAMEWORK_BASE_PATH/${base_ws}_ros_ws_$ros_distro/devel/setup.bash
  else
    source $FRAMEWORK_BASE_PATH/${base_ws}_ros_ws_$ros_distro/install/setup.bash
  fi
fi

mkdir -p ~/$ws_folder/ros_ws_${ros_distro}_$ros_ws_suffix
cd ~/$ws_folder/ros_ws_${ros_distro}_$ros_ws_suffix

if [[ $ros_version == 1 ]]; then
  wstool init src
  catkin config -DCMAKE_BUILD_TYPE=RelwithDebInfo
  catkin build
elif [[ $ros_version == 2 ]]; then
  mkdir src
  colcon build --symlink-install
fi

cd

alias_name=_ros${ros_version}
if [ -z "$ros_ws_suffix" ]; then
  fun_name="RosTeamWS_setup_ros$ros_version"
  ros_ws_suffix="-"
else
  fun_name="RosTeamWS_setup_ros${ros_version}_$ros_ws_suffix"
  alias_name=${alias_name}_$ros_ws_suffix
fi

cp ~/.bashrc ~/.bashrc.bkp
# Comment out the old configuration is such exists - this is hard if using functions...
sed -i -e '/'"$fun_name"'/ s/^#*/OLD_/' ~/.bashrc
sed -i -e '/alias st_ros'"$ros_version=$fun_name"'/ s/^#*/#/' ~/.bashrc

echo "" >> ~/.bashrc
echo "$fun_name () {" >> ~/.bashrc
echo "  RosTeamWS_BASE_WS=\"$base_ws\"" >> ~/.bashrc
echo "  RosTeamWS_DISTRO=$ros_distro" >> ~/.bashrc
echo "  RosTeamWS_WS_FOLDER=$ws_folder" >> ~/.bashrc
echo "  RosTeamWS_WS_SUFFIX=$ros_ws_suffix" >> ~/.bashrc
echo "  source $FRAMEWORK_BASE_PATH/ros_ws_\$RosTeamWS_DISTRO/src/$FRAMEWORK_NAME/scripts/environment/setup.bash \$RosTeamWS_DISTRO \$RosTeamWS_WS_SUFFIX \$RosTeamWS_WS_FOLDER" >> ~/.bashrc
echo "}" >> ~/.bashrc
echo "alias $alias_name=$fun_name" >> ~/.bashrc

# Setup new workspace
source ~/.bashrc

# Update rosdep definitions
rosdep update

if [[ $ros_version == 1 ]]; then
  rospack profile
fi

echo "------------------------------------------------------"
echo "Fnished: Please open a new terminal and execute '$alias_name'"
