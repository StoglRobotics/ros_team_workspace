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

usage="setup-robot-description ROBOT_NAME LAUNCH_FILE_TYPE"

# Load Framework defines
script_own_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
source $script_own_dir/../setup.bash

check_and_set_ros_distro_and_version ${ROS_DISTRO}

ROBOT_NAME=$1
if [ -z "$ROBOT_NAME" ]; then
  print_and_exit "ERROR: You should provide robot name! Nothing to do 😯" "$usage"
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
echo -e "${TERMINAL_COLOR_USER_NOTICE}ATTENTION: Setting up description configuration for robot '$ROBOT_NAME' in package '$PKG_NAME' in folder '$(pwd)'.${TERMINAL_COLOR_NC}"
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
cp -n "$ROBOT_DESCRIPTION_TEMPLATES/robot.urdf.xacro" $ROBOT_URDF_XACRO
cp -n "$ROBOT_DESCRIPTION_TEMPLATES/robot_macro.xacro" $ROBOT_MACRO
cp -n "$ROBOT_DESCRIPTION_TEMPLATES/robot_macro.ros2_control.xacro" $ROBOT_MACRO_ROS2_CONTROL
mkdir -p urdf/common
cp -n "$ROBOT_DESCRIPTION_TEMPLATES/inertials.xacro" urdf/common/inertials.xacro
cp -n "$ROBOT_DESCRIPTION_TEMPLATES/materials.xacro" urdf/common/materials.xacro

# Copy launch files for testing the description
for file_type in "${LAUNCH_FILE_TYPES[@]}"; do
  mkdir -p launch

  ROBOT_DESCRIPTION_LAUNCH="launch/${ROBOT_NAME}_description.launch${file_type}"
  cp -n "$ROBOT_DESCRIPTION_TEMPLATES/robot_description.launch${file_type}" $ROBOT_DESCRIPTION_LAUNCH

  VIEW_ROBOT_LAUNCH="launch/view_${ROBOT_NAME}.launch${file_type}"
  cp -n "$ROBOT_DESCRIPTION_TEMPLATES/view_robot.launch${file_type}" $VIEW_ROBOT_LAUNCH

  # sed all needed files
  FILES_TO_SED=($ROBOT_URDF_XACRO $ROBOT_MACRO $ROBOT_MACRO_ROS2_CONTROL $ROBOT_DESCRIPTION_LAUNCH $VIEW_ROBOT_LAUNCH)

  for SED_FILE in "${FILES_TO_SED[@]}"; do
    sed -i "s/\\\$PKG_NAME\\\$/${PKG_NAME}/g" $SED_FILE
    sed -i "s/\\\$ROBOT_NAME\\\$/${ROBOT_NAME}/g" $SED_FILE
  done
done

# Copy YAML files
mkdir -p config
touch config/.gitkeep

# Copy rviz files
mkdir -p rviz
ROBOT_RVIZ="rviz/${ROBOT_NAME}.rviz"
cp -n "$ROBOT_DESCRIPTION_TEMPLATES/robot.rviz" $ROBOT_RVIZ

# copy test files
mkdir -p test
ROBOT_TEST_FILE="test/${ROBOT_NAME}_test_urdf_xacro.py"
cp -n "${ROBOT_DESCRIPTION_TEMPLATES}/test_urdf_xacro.py" $ROBOT_TEST_FILE

# sed all needed files
FILES_TO_SED=($ROBOT_URDF_XACRO $ROBOT_MACRO $ROBOT_MACRO_ROS2_CONTROL $VIEW_ROBOT_LAUNCH_XML $ROBOT_TEST_FILE)
for SED_FILE in "${FILES_TO_SED[@]}"; do
  sed -i "s/\\\$PKG_NAME\\\$/${PKG_NAME}/g" $SED_FILE
  sed -i "s/\\\$ROBOT_NAME\\\$/${ROBOT_NAME}/g" $SED_FILE
done

# Add all the exec depend packages
DEPENDENCIES=("joint_state_publisher_gui" "robot_state_publisher" "rviz2" "xacro")
# append to last exec_depend or if non is found to buildtool_depend
PREVIOUS_STRING=$(grep -E "^\s*<exec_depend>" package.xml | tail -n 1)
if [ -z "$PREVIOUS_STRING" ]; then
  PREVIOUS_STRING="<buildtool_depend>ament_cmake<\/buildtool_depend>"
fi
DEPEND_TAG="exec_depend"
for DEP_PKG in "${DEPENDENCIES[@]}"; do
  # Check if the current package is already listed in the package.xml file
  if grep -q "<$DEPEND_TAG>${DEP_PKG}<\/$DEPEND_TAG>" package.xml; then
    echo "'$DEP_PKG' is already listed in package.xml"
  else
    # Check if the previous string starts with <buildtool_depend>
    if [[ ! "$PREVIOUS_STRING" =~ ^\s*\<$DEPEND_TAG.* ]]; then
      # If the previous string starts with different tag, add 2 newline characters after it to start new block
      newline="\n\n"
    else
      # otherwise add only one newline character so it's in the same block
      newline="\n"
    fi
    sed -i "s#$PREVIOUS_STRING#$PREVIOUS_STRING${newline}  <$DEPEND_TAG>${DEP_PKG}<\/$DEPEND_TAG>#g" package.xml
    PREVIOUS_STRING="<$DEPEND_TAG>${DEP_PKG}<\/$DEPEND_TAG>"
  fi
done

# Add all the test depend packages
DEPENDENCIES=("ament_cmake_pytest" "launch_testing_ament_cmake" "launch_testing_ros" "liburdfdom-tools" "xacro")
# append to last exec_depend or if non is found to buildtool_depend
PREVIOUS_STRING=$(grep -E "^\s*<test_depend>" package.xml | tail -n 1)
if [ -z "$PREVIOUS_STRING" ]; then
  PREVIOUS_STRING="<exec_depend>xacro</exec_depend>"
fi
DEPEND_TAG="test_depend"
for DEP_PKG in "${DEPENDENCIES[@]}"; do
  # Check if the current package is already listed in the package.xml file
  if grep -q "<$DEPEND_TAG>${DEP_PKG}<\/$DEPEND_TAG>" package.xml; then
    echo "'$DEP_PKG' is already listed in package.xml"
  else
    # Check if the previous string starts with <buildtool_depend>
    if [[ ! "$PREVIOUS_STRING" =~ ^\s*\<$DEPEND_TAG.* ]]; then
      # If the previous string starts with different tag, add 2 newline characters after it to start new block
      newline="\n\n"
    else
      # otherwise add only one newline character so it's in the same block
      newline="\n"
    fi
    sed -i "s#$PREVIOUS_STRING#$PREVIOUS_STRING${newline}  <$DEPEND_TAG>${DEP_PKG}<\/$DEPEND_TAG>#g" package.xml
    PREVIOUS_STRING="<$DEPEND_TAG>${DEP_PKG}<\/$DEPEND_TAG>"
  fi
done

# Update the CMakeLists.txt
# Add install paths of the files
preppend_to_string="if(BUILD_TESTING)"
sed -i "s/$preppend_to_string/install\(\\n  DIRECTORY config launch meshes rviz urdf test\\n  DESTINATION share\/\$\{PROJECT_NAME\}\\n\)\\n\\n$preppend_to_string/g" CMakeLists.txt
# Add the test
lines_to_append="  find_package(ament_cmake_pytest REQUIRED)\n\n  ament_add_pytest_test(test_${ROBOT_NAME}_urdf_xacro ${ROBOT_TEST_FILE})"
# Define the search pattern
pattern='if(BUILD_TESTING)'
# Use sed to find the pattern and append the lines after it in CMakeLists.txt
sed -i "/$pattern/a$lines_to_append" CMakeLists.txt

# extend README with general instructions
if [ ! -f README.md ]; then
  echo "#${PKG_NAME}\n\n" > README.md
fi

cat $ROBOT_DESCRIPTION_TEMPLATES/append_to_README.md >> README.md
sed -i "s/\\\$PKG_NAME\\\$/${PKG_NAME}/g" README.md
sed -i "s/\\\$ROBOT_NAME\\\$/${ROBOT_NAME}/g" README.md

# Compile and add new package the to the path
compile_and_source_package $PKG_NAME

echo ""
echo -e "${TERMINAL_COLOR_USER_NOTICE}FINISHED: You can test the configuration by executing 'ros2 launch $PKG_NAME view_${ROBOT_NAME}.launch${LAUNCH_FILE_TYPES[*]}'${TERMINAL_COLOR_NC}"
