#!/bin/bash

# workspace folder is relative to your home

usage="setup-ros-workspace.bash ROS_DISTRO WS_FOLDER WS_PREFIX WS_SUFFIX"

# Load Framework defines
script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $script_own_dir/_RosTeamWs_Defines.bash

# TODO(destogl): can we somehow reuse this in each and every script: a function? or source?
if [ -z "$1" ]; then
  print_and_exit "No input parameters provided" "$usage"
  return 2>/dev/null || exit
elif [[ $1 == "--help" || $1 == "-h" ]]; then
  print_and_exit "Here is the usage help for this script:" "$usage"
  return 2>/dev/null || exit
fi


# ros distribution name will be set in ${ros_distro}
check_ros_distro $1

ws_folder="$2"
if [ -z "$2" ]; then
  ws_folder="workspace"
  echo "Using default '~/workspace' folder to setup ros workspace"
fi

ros_ws_prefix="$3"
if [ -z "$3" ]; then
  ros_ws_prefix="-"
  echo "No ros_ws_prefix used..."
fi

ros_ws_suffix="$4"
if [ -z "$4" ]; then
  ros_ws_suffix="-"
  echo "No ros_ws_suffix used..."
fi

# TODO: Write this automatically from the user's definitions
echo "Please choose which workspace should be basis for yours:"
echo "(0) <Use current sourced workspace>"
read choice

if [ -z "$choice" ]; then
  print_and_exit "No workspace is chosen!" "$usage"
  return 2>/dev/null || exit
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

# Form here the full name
ws_full_name=${ros_distro}

if [ "$ros_ws_prefix" != "-" ]; then
  ws_full_name=${ros_ws_prefix}_${ws_full_name}
fi
if [ "$ros_ws_suffix" != "-" ]; then
  ws_full_name=${ws_full_name}_${ros_ws_suffix}
fi

# TODO: Add here output of the <current> WS
echo ""
read -p "ATTENTION: Creating a new workspace in folder '${ws_folder}' for ROS '${ros_distro}' (ROS$ros_version) with suffix '${ros_ws_suffix}' (full path: '${ws_folder}/${ws_full_name}') using '${base_ws}' as base workspace. Press <ENTER> to continue..."

# Create and initialise ROS-Workspace
if [[ ${base_ws} != "<current>" ]]; then
  if [[ $ros_version == 1 ]]; then
    source $FRAMEWORK_BASE_PATH/${base_ws}_ros_ws_${ros_distro}/devel/setup.bash
  else
    source $FRAMEWORK_BASE_PATH/${base_ws}_ros_ws_${ros_distro}/install/setup.bash
  fi
fi

mkdir -p ~/${ws_folder}/${ws_full_name}
cd ~/${ws_folder}/${ws_full_name}

if [[ $ros_version == 1 ]]; then
  wstool init src
  catkin config -DCMAKE_BUILD_TYPE=RelwithDebInfo
  catkin build
elif [[ $ros_version == 2 ]]; then
  mkdir src
#   cb
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
fi

# Go to Home folder of the user
cd

# Create a function name
fun_name_suffix=${ros_distro}

if [ "$ros_ws_prefix" != "-" ]; then
  fun_name_suffix=${fun_name_suffix}_${ros_ws_prefix}
fi
if [ "$ros_ws_suffix" != "-" ]; then
  fun_name_suffix=${fun_name_suffix}_${ros_ws_suffix}
fi

alias_name=_${ws_full_name}
fun_name="RosTeamWS_setup_${fun_name_suffix}"

cp ~/.ros_team_ws_rc ~/.ros_team_ws_rc.bkp
# Comment out the old configuration is such exists - this is hard if using functions...
sed -i -e '/'"$fun_name"'/ s/^#*/OLD_/' ~/.ros_team_ws_rc
sed -i -e '/alias st_ros'"$ros_version=$fun_name"'/ s/^#*/#/' ~/.ros_team_ws_rc

echo "" >> ~/.ros_team_ws_rc
echo "$fun_name () {" >> ~/.ros_team_ws_rc
echo "  RosTeamWS_BASE_WS=\"${base_ws}\"" >> ~/.ros_team_ws_rc
echo "  RosTeamWS_DISTRO=\"${ros_distro}\"" >> ~/.ros_team_ws_rc
echo "  RosTeamWS_WS_FOLDER=\"${ws_folder}\"" >> ~/.ros_team_ws_rc
echo "  RosTeamWS_WS_PREFIX=\"${ros_ws_prefix}\"" >> ~/.ros_team_ws_rc
echo "  RosTeamWS_WS_SUFFIX=\"${ros_ws_suffix}\"" >> ~/.ros_team_ws_rc
echo "  source $FRAMEWORK_BASE_PATH/ros_ws_\"\$RosTeamWS_DISTRO\"/src/$FRAMEWORK_NAME/scripts/environment/setup.bash \"\$RosTeamWS_DISTRO\" \"\$RosTeamWS_WS_FOLDER"\" \"\$RosTeamWS_WS_PREFIX\" \"\$RosTeamWS_WS_SUFFIX\" >> ~/.ros_team_ws_rc
echo "}" >> ~/.ros_team_ws_rc
echo "alias $alias_name=$fun_name" >> ~/.ros_team_ws_rc

# Setup new workspace
source ~/.ros_team_ws_rc

# Update rosdep definitions
rosdep update

if [[ $ros_version == 1 ]]; then
  rospack profile
fi

echo "------------------------------------------------------"
echo "Finished: Please open a new terminal and execute '$alias_name'"
