#!/bin/bash
#
# Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

usage="ros2_control_setup-hardware-interface-package FILE_NAME [CLASS_NAME]"

# Load Framework defines
script_own_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
source "$script_own_dir/../../setup.bash"

check_and_set_ros_distro_and_version "${ROS_DISTRO}"

FILE_NAME=$1
if [ -z "$1" ]; then
  print_and_exit "You should provide the file name! Nothing to do 😯" "$usage"
fi
if [ -f "src/$FILE_NAME.cpp" ]; then
  print_and_exit "ERROR:The file '$FILE_NAME' already exist! 😱!" "$usage"
fi

if [ ! -f "package.xml" ]; then
  print_and_exit "ERROR: 'package.xml' not found. \
You should execute this script at the top level of your package folder. \
Nothing to do 😯" "$usage"
fi
PKG_NAME="$(grep -Po '(?<=<name>).*?(?=</name>)' package.xml | sed -e 's/[[:space:]]//g')"

echo "" # Adds empty line

CLASS_NAME=$2
if [ -z "$2" ]; then
  delimiter='_'
  s="$FILE_NAME$delimiter"
  CLASS_NAME=""
  while [[ $s ]]; do
    part="${s%%"$delimiter"*}"
    s=${s#*"$delimiter"}
    CLASS_NAME="$CLASS_NAME${part^}"
  done
  echo -e "${TERMINAL_COLOR_USER_CONFIRMATION}ClassName guessed from the \
'$FILE_NAME': '$CLASS_NAME'. Is this correct? If not provide it as the second \
parameter.${TERMINAL_COLOR_NC}"
fi

echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}Which license-header do you want to use? [1]"
echo "(0) None"
echo "(1) Apache 2.0 License"
echo "(2) Proprietary"
echo -n -e "${TERMINAL_COLOR_NC}"
read -r choice
choice=${choice:="1"}

if [ "$choice" != 0 ]; then
  echo -n -e "${TERMINAL_COLOR_USER_INPUT_DECISION}Insert your company or personal name \
(copyright): ${TERMINAL_COLOR_NC}"
  read -r NAME_ON_LICENSE
  NAME_ON_LICENSE=${NAME_ON_LICENSE=""}
  YEAR_ON_LICENSE=$(date +%Y)
fi

LICENSE_HEADER=""
case "$choice" in
"1")
  LICENSE_HEADER="$LICENSE_TEMPLATES/default_cpp.txt"
  ;;
"2")
  LICENSE_HEADER="$LICENSE_TEMPLATES/propriatery_company_cpp.txt"
  ;;
esac

echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}Which type of ros2_control hardware interface \
you want to extend? [0]"
echo "(0) system"
echo "(1) sensor"
echo "(2) actuator"
echo -n -e "${TERMINAL_COLOR_NC}"
read -r choice
choice=${choice="0"}

INTERFACE_TYPE="system"
case "$choice" in
"1")
  INTERFACE_TYPE="sensor"
  ;;
"2")
  INTERFACE_TYPE="actuator"
  ;;
esac

echo ""
echo -e "${TERMINAL_COLOR_USER_NOTICE}ATTENTION: Setting up ros2_control hardware interface \
files with following parameters: file name '$FILE_NAME', class '$CLASS_NAME', package/namespace \
'$PKG_NAME' for interface type '$INTERFACE_TYPE'. Those will be placed in folder \
'$(pwd)'.${TERMINAL_COLOR_NC}"
echo ""
echo -e "${TERMINAL_COLOR_USER_CONFIRMATION}If correct press <ENTER>, otherwise <CTRL>+C and \
start the script again from the package folder and/or with correct robot name.${TERMINAL_COLOR_NC}"
read -r

# Add folders if deleted
ADD_FOLDERS=("include/$PKG_NAME" "src" "test")

for FOLDER in "${ADD_FOLDERS[@]}"; do
  mkdir -p "$FOLDER"
done

# Set file constants
VC_H="include/$PKG_NAME/visibility_control.h"
HW_ITF_HPP="include/$PKG_NAME/$FILE_NAME.hpp"
HW_ITF_CPP="src/$FILE_NAME.cpp"
PLUGIN_XML="$PKG_NAME.xml"
TEST_CPP="test/test_$FILE_NAME.cpp"

# Copy files
if [[ ! -f "$VC_H" ]]; then
  cp -n "$ROS2_CONTROL_HW_ITF_TEMPLATES"/dummy_package_namespace/visibility_control.h "$VC_H"
fi
cp -n "$ROS2_CONTROL_HW_ITF_TEMPLATES"/dummy_package_namespace/robot_hardware_interface.hpp "$HW_ITF_HPP"
cp -n "$ROS2_CONTROL_HW_ITF_TEMPLATES"/robot_hardware_interface.cpp "$HW_ITF_CPP"
cp -n "$ROS2_CONTROL_HW_ITF_TEMPLATES"/robot_pluginlib.xml "$PLUGIN_XML"
cp -n "$ROS2_CONTROL_HW_ITF_TEMPLATES"/test_robot_hardware_interface.cpp "$TEST_CPP"

echo -e "${TERMINAL_COLOR_USER_NOTICE}Template files copied.${TERMINAL_COLOR_NC}"

# Add license header to the files
# TODO: When Proprietary then add the following before ament_lint_auto_find_test_dependencies()
# list(APPEND AMENT_LINT_AUTO_EXCLUDE
#    ament_cmake_copyright
#  )
FILES_TO_LICENSE=("$VC_H" "$HW_ITF_HPP" "$HW_ITF_CPP" "$TEST_CPP")
TMP_FILE=".f_tmp"
if [[ "$LICENSE_HEADER" != "" ]]; then
  touch $TMP_FILE
  for FILE_TO_LIC in "${FILES_TO_LICENSE[@]}"; do
    cat "$LICENSE_HEADER" >$TMP_FILE
    sed "1,13d" "$FILE_TO_LIC" >>$TMP_FILE # delete first 13 lines which correspond to fake license
    mv $TMP_FILE "$FILE_TO_LIC"
    sed -i "s/\\\$YEAR\\\$/${YEAR_ON_LICENSE}/g" "$FILE_TO_LIC"
    sed -i "s/\\\$NAME_ON_LICENSE\\\$/${NAME_ON_LICENSE}/g" "$FILE_TO_LIC"
  done
#   echo "Licence header added to files: ("`declare -p FILES_TO_LICENSE`")"
else
  for FILE_TO_LIC in "${FILES_TO_LICENSE[@]}"; do
    sed -i "/\\\$LICENSE\\\$/d" "$FILE_TO_LIC"
  done
fi

FILES_TO_SED=("${FILES_TO_LICENSE[@]}")
# sed all needed files
FILES_TO_SED+=("$PLUGIN_XML")
# declare -p FILES_TO_SED

for SED_FILE in "${FILES_TO_SED[@]}"; do
  # package name for include guard
  sed -i "s/TEMPLATES__ROS2_CONTROL__HARDWARE__DUMMY_PACKAGE_NAMESPACE/${PKG_NAME^^}/g" "$SED_FILE"
  sed -i "s/dummy_package_namespace/${PKG_NAME}/g" "$SED_FILE"     # package name for includes
  sed -i "s/dummy_file_name/${FILE_NAME}/g" "$SED_FILE"            # file name
  sed -i "s/ROBOT_HARDWARE_INTERFACE/${FILE_NAME^^}/g" "$SED_FILE" # file name for include guard
  sed -i "s/DummyClassName/${CLASS_NAME}/g" "$SED_FILE"            # class name
  sed -i "s/dummy_interface_type/${INTERFACE_TYPE}/g" "$SED_FILE"  # interface type for includes
  sed -i "s/Dummy_Interface_Type/${INTERFACE_TYPE^}/g" "$SED_FILE" # Interface type in namespace resolution
done

# If type is "sensor" remove write and command_interfaces methods
if [[ "$INTERFACE_TYPE" == "sensor" ]]; then
  METHODS_TO_DELETE=("write(" "export_command_interfaces()")
  # Clean HPP
  for DEL_METHOD in "${METHODS_TO_DELETE[@]}"; do
    line_nr=$(grep -n "${DEL_METHOD}" "$HW_ITF_HPP" | awk -F ":" '{print $1;}')
    ((start_line = line_nr - 1))
    ((end_line = line_nr + 2))
    sed -i "${start_line},${end_line}d" "$HW_ITF_HPP"
  done
  # Clean CPP
  line_nr=$(grep -n "::write(" "$HW_ITF_CPP" | awk -F ":" '{print $1;}')
  ((start_line = line_nr - 1))
  ((end_line = line_nr + 6))
  sed -i "${start_line},${end_line}d" "$HW_ITF_CPP"

  line_nr=$(grep -n "export_command_interfaces()" "$HW_ITF_CPP" | awk -F ":" '{print $1;}')
  ((start_line = line_nr - 1))
  ((end_line = line_nr + 10))
  sed -i "${start_line},${end_line}d" "$HW_ITF_CPP"

  # Remove command interfaces from test URDF
  sed -i '/command_interface/d' "$TEST_CPP"

  # Change joint to sensor in test URDF
  sed -i 's/joint/sensor/g' "$TEST_CPP"
fi

# CMakeLists.txt: Remove comments if there any and add library
DEL_STRINGS=("# uncomment the following" "# further" "# find_package(<dependency>")

for DEL_STR in "${DEL_STRINGS[@]}"; do
  sed -i "/$DEL_STR/d" CMakeLists.txt
done

TMP_FILE=".f_tmp"
touch $TMP_FILE

# Get line with if(BUILD_TESTING)
TEST_LINE=$(awk '$1 == "if(BUILD_TESTING)" { print NR }' CMakeLists.txt)
((CUT_LINE = TEST_LINE - 1))
{
  head -$CUT_LINE CMakeLists.txt

  # Add Plugin library stuff inside
  echo "add_library("
  echo "  $PKG_NAME"
  echo "  SHARED"
  echo "  $HW_ITF_CPP"
  echo ")"

  echo "target_include_directories("
  echo "  $PKG_NAME"
  echo "  PUBLIC"
  echo "  include"
  echo ")"

  echo "ament_target_dependencies("
  echo "  $PKG_NAME"
  echo "  hardware_interface"
  echo "  rclcpp"
  echo "  rclcpp_lifecycle"
  echo ")"

  echo ""
  echo "pluginlib_export_plugin_description_file("
  echo "  hardware_interface $PLUGIN_XML)"

  ## Add install directives
  echo ""
  echo "install("
  echo "  TARGETS"
  echo "  $PKG_NAME"
  echo "  RUNTIME DESTINATION bin"
  echo "  ARCHIVE DESTINATION lib"
  echo "  LIBRARY DESTINATION lib"
  echo ")"

  if ! grep -q "DIRECTORY include/" "$TMP_FILE"; then
    echo ""
    echo "install("
    echo "  DIRECTORY include/"
    echo "  DESTINATION include"
    echo ")"
  fi

  echo ""
} >>$TMP_FILE

END_TEST_LINE=$(tail -n +"$TEST_LINE" CMakeLists.txt | awk '$1 == "endif()" { print NR }')
((CUT_LINE = END_TEST_LINE - 1))
tail -n +"$TEST_LINE" CMakeLists.txt | head -"$CUT_LINE" >>$TMP_FILE

{
  echo ""
  echo "  find_package(ament_cmake_gmock REQUIRED)"
  echo "  find_package(ros2_control_test_assets REQUIRED)"
  echo ""
  echo "  ament_add_gmock(test_$FILE_NAME $TEST_CPP)"
  echo "  target_include_directories(test_$FILE_NAME PRIVATE include)"
  echo "  ament_target_dependencies("
  echo "    test_$FILE_NAME"
  echo "    hardware_interface"
  echo "    pluginlib"
  echo "    ros2_control_test_assets"
  echo "  )"
  echo ""
} >>$TMP_FILE

# Add export definitions
tail -n +"$TEST_LINE" CMakeLists.txt | head -"$END_TEST_LINE" | tail -1 >>$TMP_FILE

{
  echo ""
  echo "ament_export_include_directories("
  echo "  include"
  echo ")"

  echo "ament_export_libraries("
  echo "  $PKG_NAME"
  echo ")"

  echo "ament_export_dependencies("
  echo "  hardware_interface"
  echo "  pluginlib"
  echo "  rclcpp"
  echo "  rclcpp_lifecycle"
  echo ")"
} >>$TMP_FILE

# Add last part
((CUT_LINE = END_TEST_LINE + 1))
tail -n +"$TEST_LINE" CMakeLists.txt | tail -n +$CUT_LINE >>$TMP_FILE

mv $TMP_FILE CMakeLists.txt

# CMakeLists.txt & package.xml: Add dependencies if they not exist
DEP_PKGS=("rclcpp_lifecycle" "rclcpp" "pluginlib" "hardware_interface")

for DEP_PKG in "${DEP_PKGS[@]}"; do

  # CMakeLists.txt
  if grep -q "find_package(${DEP_PKG} REQUIRED)" CMakeLists.txt; then
    echo "'$DEP_PKG' is already dependency in CMakeLists.txt"
  else
    append_to_string="find_package(ament_cmake REQUIRED)"
    sed -i "s/$append_to_string/$append_to_string\\nfind_package(${DEP_PKG} REQUIRED)/g" CMakeLists.txt
  fi

  # package.xml
  if grep -q "<depend>${DEP_PKG}</depend>" package.xml; then
    echo "'$DEP_PKG' is already listed in package.xml"
  else
    append_to_string="<buildtool_depend>ament_cmake<\/buildtool_depend>"
    sed -i "s/$append_to_string/$append_to_string\\n\\n  <depend>${DEP_PKG}<\/depend>/g" package.xml
  fi

done

# CMakeLists.txt & package.xml: Add test dependencies if they not exist
TEST_DEP_PKGS=("ros2_control_test_assets" "ament_cmake_gmock")

for DEP_PKG in "${TEST_DEP_PKGS[@]}"; do

  # CMakeLists.txt
  if grep -q "  find_package(${DEP_PKG} REQUIRED)" CMakeLists.txt; then
    echo "'$DEP_PKG' is already listed in CMakeLists.txt"
  else
    append_to_string="if(BUILD_TESTING)"
    sed -i "s/$append_to_string/$append_to_string\\n  find_package(${DEP_PKG} REQUIRED)/g" CMakeLists.txt
  fi

  # package.xml
  if grep -q "<test_depend>${DEP_PKG}</test_depend>" package.xml; then
    echo "'$DEP_PKG' is already listed in package.xml"
  else
    prepend_to_string="  <export>"
    sed -i "s/$prepend_to_string/  <test_depend>${DEP_PKG}<\/test_depend>\\n\\n$prepend_to_string/g" package.xml
  fi
done

# extend README with general instructions
if [ -f README.md ]; then
  {
    echo ""
    echo "Pluginlib-Library: $PKG_NAME"
    echo "Plugin: $PKG_NAME/${CLASS_NAME} (hardware_interface::${INTERFACE_TYPE^}Interface)"
  } >>README.md
fi

echo -e "${TERMINAL_COLOR_USER_NOTICE}Template files were adjusted.${TERMINAL_COLOR_NC}"

git add .
# git commit -m "RosTeamWS: ros2_control skeleton files for $ROBOT_NAME generated."

# Compile and add new package the to the path
compile_and_source_package "$PKG_NAME" "yes"

echo ""
echo -e "${TERMINAL_COLOR_USER_NOTICE}FINISHED: Your package is set and the tests should be \
finished without any errors. (linter errors possible!)${TERMINAL_COLOR_NC}"
