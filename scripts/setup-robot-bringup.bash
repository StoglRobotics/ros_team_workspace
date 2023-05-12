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

usage="setup-robot-bringup ROBOT_NAME DESCRIPTION_PKG_NAME"

# Load Framework defines
script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
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

if [ ! -f "package.xml" ]; then
  print_and_exit "ERROR: 'package.xml' not found. You should execute this script at the top level of your package folder. Nothing to do 😯" "$usage"
fi
PKG_NAME="$(grep -Po '(?<=<name>).*?(?=</name>)' package.xml | sed -e 's/[[:space:]]//g')"


echo ""
echo -e "${TERMINAL_COLOR_USER_NOTICE}ATTENTION: Setting up bringup configuration for robot '$ROBOT_NAME' in package '$PKG_NAME' in folder '`pwd`' with robot description package '$DESCR_PKG_NAME'.${TERMINAL_COLOR_NC}"
echo -e "${TERMINAL_COLOR_USER_CONFIRMATION}If correct press <ENTER>, otherwise <CTRL>+C and start the script again from the package folder and/or with correct robot name.${TERMINAL_COLOR_NC}"
read

# Remove include and src folders - in this package should be no source
RM_FOLDERS=("include" "src")

for FOLDER in "${RM_FOLDERS[@]}"; do
  if [[ -d $FOLDER && ! "$(ls -A $FOLDER )" ]]; then
    rm -r $FOLDER
  fi
done

# Create folders
mkdir -p config
mkdir -p launch

# Copy config files
ROBOT_CONTROLLERS_YAML="config/${ROBOT_NAME}_controllers.yaml"
ROBOT_FPC_PUB_YAML="config/test_goal_publishers_config.yaml"
cp -n $ROS2_CONTROL_TEMPLATES/robot_controllers.yaml $ROBOT_CONTROLLERS_YAML
cp -n $ROS2_CONTROL_TEMPLATES/test_goal_publishers_config.yaml $ROBOT_FPC_PUB_YAML

# Copy launch files
ROBOT_LAUNCH="launch/${ROBOT_NAME}.launch.py"
TEST_FWD_POS_CTRL_LAUNCH="launch/test_forward_position_controller.launch.py"
TEST_JTC_LAUNCH="launch/test_joint_trajectory_controller.launch.py"
cp -n $ROS2_CONTROL_TEMPLATES/robot_ros2_control.launch.py ${ROBOT_LAUNCH}
cp -n $ROS2_CONTROL_TEMPLATES/test_forward_position_controller.launch.py $TEST_FWD_POS_CTRL_LAUNCH
cp -n $ROS2_CONTROL_TEMPLATES/test_joint_trajectory_controller.launch.py $TEST_JTC_LAUNCH


# sed all needed files
FILES_TO_SED=($ROBOT_LAUNCH $TEST_FWD_POS_CTRL_LAUNCH $TEST_JTC_LAUNCH)

for SED_FILE in "${FILES_TO_SED[@]}"; do
  sed -i "s/\\\$PKG_NAME\\\$/${PKG_NAME}/g" $SED_FILE
  sed -i "s/\\\$RUNTIME_CONFIG_PKG_NAME\\\$/${PKG_NAME}/g" $SED_FILE
  sed -i "s/\\\$ROBOT_NAME\\\$/${ROBOT_NAME}/g" $SED_FILE
  sed -i "s/\\\$DESCR_PKG_NAME\\\$/${DESCR_PKG_NAME}/g" $SED_FILE
done

# package.xml: Add dependencies if they not exist
DEP_PKGS=("xacro" "rviz2" "ros2_controllers_test_nodes" "robot_state_publisher" "joint_trajectory_controller" "joint_state_broadcaster" "forward_command_controller" "controller_manager" "$DESCR_PKG_NAME")

for DEP_PKG in "${DEP_PKGS[@]}"; do
  if `grep -q $DEP_PKG package.xml`; then
    echo "'$DEP_PKG' is already listed in package.xml"
  else
    append_to_string="<buildtool_depend>ament_cmake<\/buildtool_depend>"
    sed -i "s/$append_to_string/$append_to_string\\n\\n  <exec_depend>${DEP_PKG}<\/exec_depend>/g" package.xml
  fi
done

# CMakeLists.txt: Add install paths of the files
prepend_to_string="if(BUILD_TESTING)"
sed -i "s/$prepend_to_string/install\(\\n  DIRECTORY config launch\\n  DESTINATION share\/\$\{PROJECT_NAME\}\\n\)\\n\\n$prepend_to_string/g" CMakeLists.txt

# extend README with general instructions
if [ -f README.md ]; then
  cat $ROS2_CONTROL_TEMPLATES/append_to_README.md >> README.md
  sed -i "s/\\\$PKG_NAME\\\$/${PKG_NAME}/g" README.md
  sed -i "s/\\\$ROBOT_NAME\\\$/${ROBOT_NAME}/g" README.md
  sed -i "s/\\\$DESCR_PKG_NAME\\\$/${DESCR_PKG_NAME}/g" $SED_FILE
fi

# TODO: Add license checks

git add .
# git commit -m "RosTeamWS: Bringup files for $ROBOT_NAME generated."

# Compile and add new package the to the path
compile_and_source_package $PKG_NAME

echo ""
echo -e "${TERMINAL_COLOR_USER_NOTICE}FINISHED: You can test the configuration by executing 'ros2 launch $PKG_NAME ${ROBOT_NAME}.launch.py'${TERMINAL_COLOR_NC}"
