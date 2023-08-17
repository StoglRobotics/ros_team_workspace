#!/bin/bash
#
# Copyright 2021 Stogl Denis Stogl (Stogl Robotics Consulting)
# Copyright 2017-2020 Denis Stogl (Institute for Anthropomatics and Robotics (IAR) -
#  Intelligent Process Control and Robotics (IPR) of Karlsruhe Institute of Technology (KIT))
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

usage="setup-robot-description ROBOT_NAME"

# Load Framework defines
script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $script_own_dir/../setup.bash
check_and_set_ros_distro_and_version ${ROS_DISTRO}

ROBOT_NAME=$1
if [ -z "$ROBOT_NAME" ]; then
  print_and_exit "ERROR: You should provide robot name! Nothing to do 😯" "$usage"
fi

if [ ! -f "package.xml" ]; then
  print_and_exit "ERROR: 'package.xml' not found. You should execute this script at the top level of your package folder. Nothing to do 😯" "$usage"
fi
PKG_NAME="$(grep -Po '(?<=<name>).*?(?=</name>)' package.xml | sed -e 's/[[:space:]]//g')"

echo ""
echo -e "${TERMINAL_COLOR_USER_NOTICE}ATTENTION: Setting up description configuration for robot '$ROBOT_NAME' in package '$PKG_NAME' in folder '`pwd`'.${TERMINAL_COLOR_NC}"
echo -e "${TERMINAL_COLOR_USER_CONFIRMATION}If correct press <ENTER>, otherwise <CTRL>+C and start the script again from the package folder and/or with correct robot name.${TERMINAL_COLOR_NC}"
read

# Create folders for meshes
F_NAME="meshes/${ROBOT_NAME}/collision"
mkdir -p $F_NAME
touch $F_NAME/.gitkeep

F_NAME="meshes/${ROBOT_NAME}/visual"
mkdir -p $F_NAME
touch $F_NAME/.gitkeep

# Create folder for URDF/xacro files
F_NAME="urdf/${ROBOT_NAME}"
mkdir -p $F_NAME

# Copy URDF/xacro files
ROBOT_URDF_XACRO="urdf/${ROBOT_NAME}.urdf.xacro"
ROBOT_MACRO="urdf/${ROBOT_NAME}/${ROBOT_NAME}_macro.xacro"
ROBOT_MACRO_ROS2_CONTROL="urdf/${ROBOT_NAME}/${ROBOT_NAME}_macro.ros2_control.xacro"
cp -n "$ROBOT_DESCRIPTION_TEMPLATES/common.xacro" urdf/common.xacro
cp -n "$ROBOT_DESCRIPTION_TEMPLATES/robot.urdf.xacro" $ROBOT_URDF_XACRO
cp -n "$ROBOT_DESCRIPTION_TEMPLATES/robot_macro.xacro" $ROBOT_MACRO
cp -n "$ROBOT_DESCRIPTION_TEMPLATES/robot_macro.ros2_control.xacro" $ROBOT_MACRO_ROS2_CONTROL

# Copy launch.py file for testing the description
mkdir -p launch
VIEW_ROBOT_LAUNCH="launch/view_${ROBOT_NAME}.launch.py"
cp -n "$ROBOT_DESCRIPTION_TEMPLATES/view_robot.launch.py" $VIEW_ROBOT_LAUNCH

# Copy YAML files
mkdir -p config
touch config/.gitkeep

# Copy rviz files
mkdir -p rviz
ROBOT_RVIZ="rviz/${ROBOT_NAME}.rviz"
cp -n "$ROBOT_DESCRIPTION_TEMPLATES/robot.rviz" $ROBOT_RVIZ

# sed all needed files
FILES_TO_SED=($ROBOT_URDF_XACRO $ROBOT_MACRO $ROBOT_MACRO_ROS2_CONTROL $VIEW_ROBOT_LAUNCH)

for SED_FILE in "${FILES_TO_SED[@]}"; do
  sed -i "s/\\\$PKG_NAME\\\$/${PKG_NAME}/g" $SED_FILE
  sed -i "s/\\\$ROBOT_NAME\\\$/${ROBOT_NAME}/g" $SED_FILE
done

# Add dependencies if they not exist
DEP_PKGS=("xacro" "rviz2" "robot_state_publisher" "joint_state_publisher_gui")

for DEP_PKG in "${DEP_PKGS[@]}"; do
  if `grep -q $DEP_PKG package.xml`; then
    echo "'$DEP_PKG' is already listed in package.xml"
  else
    append_to_string="<buildtool_depend>ament_cmake<\/buildtool_depend>"
    sed -i "s/$append_to_string/$append_to_string\\n\\n  <exec_depend>${DEP_PKG}<\/exec_depend>/g" package.xml
  fi
done

# Add install paths of the files
preppend_to_string="if(BUILD_TESTING)"
sed -i "s/$preppend_to_string/install\(\\n  DIRECTORY config launch meshes rviz urdf\\n  DESTINATION share\/\$\{PROJECT_NAME\}\\n\)\\n\\n$preppend_to_string/g" CMakeLists.txt

# extend README with general instructions
if [ -f README.md ]; then
  cat $ROBOT_DESCRIPTION_TEMPLATES/append_to_README.md >> README.md
  sed -i "s/\\\$PKG_NAME\\\$/${PKG_NAME}/g" README.md
  sed -i "s/\\\$ROBOT_NAME\\\$/${ROBOT_NAME}/g" README.md
fi

#TODO: Set license

git add .
git commit -m "RosTeamWS: Description files for $ROBOT_NAME generated."

# Compile and add new package the to the path
compile_and_source_package $PKG_NAME

echo ""
echo -e "${TERMINAL_COLOR_USER_NOTICE}FINISHED: You can test the configuration by executing 'ros2 launch $PKG_NAME view_${ROBOT_NAME}.launch.py'${TERMINAL_COLOR_NC}"
