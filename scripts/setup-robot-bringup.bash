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

usage="setup-robot-bringup.bash ROBOT_NAME DESCRIPTION_PKG [PKG_NAME]"

# echo ""
# echo "Your path is `pwd`. Is this your package folder where to setup robot's bringup?"
# read -p "If so press <ENTER> otherwise <CTRL>+C and start the script again from the bringup folder."

# Load Framework defines
script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $script_own_dir/_RosTeamWs_Defines.bash
check_ros_distro ${ROS_DISTRO}

ROBOT_NAME=$1
if [ -z "$1" ]; then
  echo "You should provide robot name!"
  exit 0
fi

DESCR_PKG_NAME=$2
if [ -z "$2" ]; then
  echo "You should provide description package name!"
  exit 0
fi

PKG_NAME=$3
if [ -z "$3" ]; then
  current=`pwd`
  PKG_NAME=$(basename "$current")
  echo "Package name guessed from the current path is '$PKG_NAME' is this correct? If not provide it as second parameter."
fi

echo ""
echo "ATTENTION: Setting up bringup configuration for robot '$ROBOT_NAME' in package '$PKG_NAME' in folder '`pwd`'."
echo ""
read -p "If correct press <ENTER>, otherwise <CTRL>+C and start the script again from the package folder and/or with correct robot name."

# Remove include and src folders - in this package should be no source
RM_FOLDERS=("include" "src")

for FOLDER in "${RM_FOLDERS[@]}"; do
  if [ -d $FOLDER && ! "$(ls -A $FOLDER )" ]; then
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
TEST_PUB_LAUNCH="launch/test_forward_position_controller.launch.py"
TEST_JTC_PUB_LAUNCH="launch/test_joint_trajectory_controller.launch.py"
cp -n $ROS2_CONTROL_TEMPLATES/robot_ros2_control.launch.py ${ROBOT_LAUNCH}
cp -n $ROS2_CONTROL_TEMPLATES/test_forward_position_controller.launch.py $TEST_PUB_LAUNCH


# sed all needed files
FILES_TO_SED=($ROBOT_LAUNCH $TEST_PUB_LAUNCH)

for SED_FILE in "${FILES_TO_SED[@]}"; do
  sed -i "s/\\\$PKG_NAME\\\$/${PKG_NAME}/g" $SED_FILE
  sed -i "s/\\\$RUNTIME_CONFIG_PKG_NAME\\\$/${PKG_NAME}/g" $SED_FILE
  sed -i "s/\\\$ROBOT_NAME\\\$/${ROBOT_NAME}/g" $SED_FILE
  sed -i "s/\\\$DESCR_PKG_NAME\\\$/${DESCR_PKG_NAME}/g" $SED_FILE
done

# package.xml: Add dependencies if they not exist
DEP_PKGS=("xacro" "rviz2" "robot_state_publisher" "joint_trajectory_controller" "joint_state_controller" "forward_command_controller" "controller_manager" "$DESCR_PKG_NAME")

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
sed -i "s/$prepend_to_string/install\(\\n  DIRECTORY config launch\/\\n  DESTINATION share\/\$\{PROJECT_NAME\}\\n\)\\n\\n$prepend_to_string/g" CMakeLists.txt

# extend README with general instructions
if [ -f README.md ]; then
  cat $ROS2_CONTROL_TEMPLATES/append_to_README.md >> README.md
  sed -i "s/\\\$PKG_NAME\\\$/${PKG_NAME}/g" README.md
  sed -i "s/\\\$ROBOT_NAME\\\$/${ROBOT_NAME}/g" README.md
  sed -i "s/\\\$DESCR_PKG_NAME\\\$/${DESCR_PKG_NAME}/g" $SED_FILE
fi

# TODO: Add license checks

git add .
git commit -m "RosTeamWS: Bringup files for $ROBOT_NAME generated."

# Compile and add new package the to the path
compile_and_source_package $PKG_NAME

echo ""
echo "FINISHED: You can test the configuration by executing 'ros2 launch $PKG_NAME test_${ROBOT_NAME}_bringup.launch.py'"
