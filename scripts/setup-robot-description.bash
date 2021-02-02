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

usage="setup-robot-description.bash ROBOT_NAME [PKG_NAME]"

echo ""
echo "Your path is `pwd`. Is this your package folder where to setup robot's description?"
read -p "If so press <ENTER> otherise <CTRL>+C and start the script again from the description folder."

# Load Framework defines
script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $script_own_dir/_RosTeamWs_Defines.bash
check_ros_distro ${ROS_DISTRO}

ROBOT_NAME=$1
if [ -z "$1" ]; then
  echo "You should provide package description!"
  exit 0
fi

PKG_NAME=$2
if [ -z "$2" ]; then
  current=`pwd`
  PKG_NAME=$(basename "$current")
  echo "Package name guessed from the current path is '$PKG_NAME' is this correct?"
  echo "If so press <ENTER> otherise <CTRL>+C and start the script with the package name as second parameter."
  read
fi

# Create folders for meshes
F_NAME="meshes/${ROBOT_NAME}/visual"
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

cp -n $ROBOT_DESCRIPTION_TEMPLATES/common.xacro urdf/common.xacro
cp -n $ROBOT_DESCRIPTION_TEMPLATES/robot.urdf.xacro $ROBOT_URDF_XACRO
cp -n $ROBOT_DESCRIPTION_TEMPLATES/robot_macro.xacro $ROBOT_MACRO
cp -n $ROBOT_DESCRIPTION_TEMPLATES/robot_macro.ros2_control.xacro $ROBOT_MACRO_ROS2_CONTROL

# Copy launch.py file for testing the description
# TODO: Add here ros2_control launch file
mkdir -p launch
TEST_ROBOT_DESCRIPTION_LAUNCH="launch/test_${ROBOT_NAME}_description.launch.py"
cp -n $ROBOT_DESCRIPTION_TEMPLATES/test_robot_description.launch.py $TEST_ROBOT_DESCRIPTION_LAUNCH

# Copy YAML files
mkdir -p config
ROBOT_CONTROLLERS_YAML="config/${ROBOT_NAME}_controllers.yaml"
cp -n $ROBOT_DESCRIPTION_TEMPLATES/robot_controllers.yaml $ROBOT_CONTROLLERS_YAML

# Copy rviz files
mkdir -p rviz
ROBOT_RVIZ="rviz/${ROBOT_NAME}.rviz"
cp -n $ROBOT_DESCRIPTION_TEMPLATES/robot.rviz $ROBOT_RVIZ

# sed all needed files
FILES_TO_SED=($ROBOT_URDF_XACRO $ROBOT_MACRO $ROBOT_MACRO_ROS2_CONTROL $TEST_ROBOT_DESCRIPTION_LAUNCH)

for SED_FILE in "${FILES_TO_SED[@]}"; do
  sed -i "s/\\\$PKG_NAME\\\$/${PKG_NAME}/g" $SED_FILE
  sed -i "s/\\\$ROBOT_NAME\\\$/${ROBOT_NAME}/g" $SED_FILE
done

# Add dependencies if they not exist
DEP_PKGS=("joint_state_publisher_gui" "robot_state_publisher" "rviz2")

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
sed -i "s/$preppend_to_string/install\(\\n  DIRECTORY config launch\/ meshes rviz urdf\\n  DESTINATION share\/\$\{PROJECT_NAME\}\\n\)\\n\\n$preppend_to_string/g" CMakeLists.txt

git add .
git commit -m "RosTeamWS: Description files for $ROBOT_NAME generated."

# TODO: move this into separate, helper function
# Compile and add new package the to the path
bn=`basename "$PWD"`
path=$bn
while [[ "$bn" != "src" ]]; do
  cd ..
  bn=`basename "$PWD"`
  path="$bn/$path"
done
cd ..
colcon build --symlink-install --packages-select $PKG_NAME
source install/setup.bash
cd $path

echo ""
echo "FINISHED: You can test the configuration by executing 'ros2 launch $PKG_NAME test_${ROBOT_NAME}_description.launch.py'"






























