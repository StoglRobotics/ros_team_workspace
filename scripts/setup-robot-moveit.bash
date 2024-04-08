#!/bin/bash
#
# Copyright 2021 Denis Stogl (Stogl Robotics Consulting)
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

usage="setup-robot-moveit ROBOT_NAME DESCRIPTION_PKG_NAME"

# Load Framework defines
script_own_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
source $script_own_dir/../setup.bash
check_and_set_ros_distro_and_version "${ROS_DISTRO}"

ROBOT_NAME=$1
if [ -z "$ROBOT_NAME" ]; then
  print_and_exit "ERROR: You should provide robot name! Nothing to do 😯" "$usage"
fi

DESCR_PKG_NAME=$2
if [ -z "$DESCR_PKG_NAME" ]; then
  print_and_exit "ERROR: You should provide description package name! Nothing to do 😯" "$usage"
fi

echo "Which launchfiles should be added? Choose from the following options:"
echo "1) xml"
echo "2) python"
echo "3) both"

read -p "Enter your choice:" choice

LAUNCH_FILE_TYPES=()

case $choice in
1)
  LAUNCH_FILE_TYPES+=(".xml")
  ;;
2)
  LAUNCH_FILE_TYPES+=(".py")
  ;;
3)
  LAUNCH_FILE_TYPES+=(".xml" ".py")
  ;;
*)
  print_and_exit "Invalid choice. Exiting."
  ;;
esac

if [ ! -f "package.xml" ]; then
  print_and_exit "ERROR: 'package.xml' not found. You should execute this script at the top level of your package folder. Nothing to do 😯" "$usage"
fi
PKG_NAME="$(grep -Po '(?<=<name>).*?(?=</name>)' package.xml | sed -e 's/[[:space:]]//g')"

echo ""
echo -e "${TERMINAL_COLOR_USER_NOTICE}ATTENTION: Setting up moveit package for robot '$ROBOT_NAME' in package '$PKG_NAME' in folder '$(pwd)' with robot description package '$DESCR_PKG_NAME'.${TERMINAL_COLOR_NC}"
echo -e "${TERMINAL_COLOR_USER_CONFIRMATION}If correct press <ENTER>, otherwise <CTRL>+C and start the script again from the package folder and/or with correct robot name.${TERMINAL_COLOR_NC}"
read

# Remove include and src folders - in this package should be no source
RM_FOLDERS=("src")

for FOLDER in "${RM_FOLDERS[@]}"; do
  if [[ -d $FOLDER && ! "$(ls -A $FOLDER)" ]]; then
    rm -r $FOLDER
  fi
done

# Create folders
mkdir -p config
mkdir -p launch
mkdir -p rviz
mkdir -p srdf

# Copy rviz files
mkdir -p rviz
ROBOT_RVIZ="rviz/moveit.rviz"
cp -n "$MOVEIT_TEMPLATES/moveit.rviz" $ROBOT_RVIZ

# Copy config files
MOVE_GROUP_CONFIG_YAML="config/move_group.yaml"
OMPL_PLANNING_CONFIG_YAML="config/ompl_planning.yaml"
cp -n $MOVEIT_TEMPLATES/move_group.yaml $MOVE_GROUP_CONFIG_YAML
cp -n $MOVEIT_TEMPLATES/ompl_planning.yaml $OMPL_PLANNING_CONFIG_YAML

# Copy SRDF/xacro files
ROBOT_SRDF="srdf/${ROBOT_NAME}.srdf.xacro"
ROBOT_SRDF_MACRO="srdf/${ROBOT_NAME}_macro.srdf.xacro"
cp -n "$MOVEIT_TEMPLATES/robot.srdf.xacro" $ROBOT_SRDF
cp -n "$MOVEIT_TEMPLATES/robot_macro.srdf.xacro" $ROBOT_SRDF_MACRO


# Copy launch files
for file_type in "${LAUNCH_FILE_TYPES[@]}"; do
  # Construct the file paths
  MOVEIT_LAUNCH="launch/moveit.launch${file_type}"

  # Copy the templates to the destination with the specified file type
  cp -n "$MOVEIT_TEMPLATES/moveit.launch${file_type}" "${MOVEIT_LAUNCH}"

  # sed all needed files
  FILES_TO_SED=($MOVEIT_LAUNCH $ROBOT_SRDF $ROBOT_SRDF_MACRO $MOVE_GROUP_CONFIG_YAML $OMPL_PLANNING_CONFIG_YAML)

  for SED_FILE in "${FILES_TO_SED[@]}"; do
    sed -i "s/\\\$PKG_NAME\\\$/${PKG_NAME}/g" $SED_FILE
    sed -i "s/\\\$ROBOT_NAME\\\$/${ROBOT_NAME}/g" $SED_FILE
    sed -i "s/\\\$DESCR_PKG_NAME\\\$/${DESCR_PKG_NAME}/g" $SED_FILE
  done
done

# package.xml: Add dependencies if they not exist
DEP_PKGS=(
  "xacro"
  "$DESCR_PKG_NAME"  
  "moveit_ros_move_group"
  "moveit_kinematics"
  "moveit_planners"
  "moveit_simple_controller_manager"
  )

for DEP_PKG in "${DEP_PKGS[@]}"; do
  if $(grep -q $DEP_PKG package.xml); then
    echo "'$DEP_PKG' is already listed in package.xml"
  else
    append_to_string="<buildtool_depend>ament_cmake<\/buildtool_depend>"
    sed -i "s/$append_to_string/$append_to_string\\n\\n  <exec_depend>${DEP_PKG}<\/exec_depend>/g" package.xml
  fi
done

# CMakeLists.txt: Add install paths of the files
prepend_to_string="if(BUILD_TESTING)"
sed -i "s/$prepend_to_string/install\(\\n  DIRECTORY config launch rviz srdf\\n  DESTINATION share\/\$\{PROJECT_NAME\}\\n\)\\n\\n$prepend_to_string/g" CMakeLists.txt

# TODO: add README with general instructions

# TODO: Add license checks

# skip compilation, let the user install MoveIt manually, as it can introduce
# breaking changes often.

echo ""
echo -e "${TERMINAL_COLOR_USER_NOTICE}FINISHED: You can test the configuration by first launching the ros2_control bringup, followed by 
'ros2 launch $PKG_NAME moveit.launch${LAUNCH_FILE_TYPES[*]}'${TERMINAL_COLOR_NC}"
