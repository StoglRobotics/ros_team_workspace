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

usage="setup-controller-package.bash FILE_NAME [CLASS_NAME] [PKG_NAME]"

# echo ""
# echo "Your path is `pwd`. Is this your package folder where to setup robot's bringup?"
# read -p "If so press <ENTER> otherwise <CTRL>+C and start the script again from the bringup folder."

# Load Framework defines
script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $script_own_dir/../../setup.bash
check_ros_distro ${ROS_DISTRO}

FILE_NAME=$1
if [ -z "$1" ]; then
  print_and_exit "ERROR: You should provide the file name!" "$usage"
fi
if [ -f src/$FILE_NAME.cpp ]; then
  print_and_exit "ERROR:The file '$FILE_NAME' already exist!" "$usage"
fi

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
  echo "ClassName guessed from the '$FILE_NAME': '$CLASS_NAME'. Is this correct? If not, provide it as the second parameter."
fi

PKG_NAME=$3
if [ -z "$3" ]; then
  current=`pwd`
  PKG_NAME=$(basename "$current")
  echo "Package name guessed from the current path is '$PKG_NAME'. Is this correct? If not provide it as the third parameter."
fi

echo "Which license-header do you want to use? [1]"
echo "(0) None"
echo "(1) Apache 2.0 License"
echo "(2) Propiatery"
read choice
choice=${choice:="1"}

if [ "$choice" != 0 ]; then
  read -p "Insert your company or personal name (copyright): " NAME_ON_LICENSE
  NAME_ON_LICENSE=${NAME_ON_LICENSE=""}
  YEAR_ON_LICENSE=`date +%Y`
fi

LICENSE_HEADER=""
case "$choice" in
"1")
  LICENSE_HEADER="$LICENSE_TEMPLATES/default_cpp.txt"
  ;;
"2")
  LICENSE_HEADER="$LICENSE_TEMPLATES/propriatery_company_cpp.txt"
esac

read -p "Is package already configured (is in there a working controller already)? (yes/no) [no] " package_configured
package_configured=${package_configured:="no"}

echo ""
echo "ATTENTION: Setting up ros2_control controller files with following parameters: file name '$FILE_NAME', class '$CLASS_NAME', package/namespace '$PKG_NAME'. Those will be placed in folder '`pwd`'."
echo ""
read -p "If correct press <ENTER>, otherwise <CTRL>+C and start the script again from the package folder and/or with correct controller name."

# Add folders if deleted
ADD_FOLDERS=("include/$PKG_NAME" "src" "test")

for FOLDER in "${ADD_FOLDERS[@]}"; do
    mkdir -p $FOLDER
done

# Set file constants
VC_H="include/$PKG_NAME/visibility_control.h"
CTRL_HPP="include/$PKG_NAME/$FILE_NAME.hpp"
CTRL_CPP="src/$FILE_NAME.cpp"
PLUGIN_XML="$PKG_NAME.xml"
LOAD_TEST_CPP="test/test_load_$FILE_NAME.cpp"
TEST_CPP="test/test_$FILE_NAME.cpp"
TEST_HPP="test/test_$FILE_NAME.hpp"

# Copy files
if [[ ! -f "$VC_H" ]]; then
  cp -n $ROS2_CONTROL_HW_ITF_TEMPLATES/visibility_control.h $VC_H
fi
cat $ROS2_CONTROL_CONTROLLER_TEMPLATES/controller_pluginlib.xml >> $PLUGIN_XML
cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/controller.hpp $CTRL_HPP
cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/controller.cpp $CTRL_CPP
cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/test_load_controller.cpp $LOAD_TEST_CPP
cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/test_controller.cpp $TEST_CPP
cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/test_controller.hpp $TEST_HPP

echo "Template files copied."

# TODO(anyone): fuse this with hardware interface package creating.

# Add license header to the files
FILES_TO_LICENSE=("$CTRL_HPP" "$CTRL_CPP" "$LOAD_TEST_CPP" "$TEST_CPP" "$TEST_HPP")
if [[ "$package_configured" == "no" ]]; then
  FILES_TO_LICENSE+=("$VC_H")
fi
TMP_FILE=".f_tmp"
if [[ "$LICENSE_HEADER" != "" ]]; then
  touch $TMP_FILE
  for FILE_TO_LIC in "${FILES_TO_LICENSE[@]}"; do
    cat $LICENSE_HEADER > $TMP_FILE
    cat $FILE_TO_LIC >> $TMP_FILE
    sed -i "/\\\$LICENSE\\\$/d" $TMP_FILE
    mv $TMP_FILE $FILE_TO_LIC
    sed -i "s/\\\$YEAR\\\$/${YEAR_ON_LICENSE}/g" $FILE_TO_LIC
    sed -i "s/\\\$NAME_ON_LICENSE\\\$/${NAME_ON_LICENSE}/g" $FILE_TO_LIC
  done
#   echo "Licence header added to files: ("`declare -p FILES_TO_LICENSE`")"
fi

# sed all needed files
FILES_TO_SED=("${FILES_TO_LICENSE[@]}")
FILES_TO_SED+=("$PLUGIN_XML")
# declare -p FILES_TO_SED

for SED_FILE in "${FILES_TO_SED[@]}"; do
  sed -i "s/\\\$PACKAGE_NAME\\\$/${PKG_NAME^^}/g" $SED_FILE
  sed -i "s/\\\$package_name\\\$/${PKG_NAME}/g" $SED_FILE
  sed -i "s/\\\$file_name\\\$/${FILE_NAME}/g" $SED_FILE
  sed -i "s/\\\$FILE_NAME\\\$/${FILE_NAME^^}/g" $SED_FILE
  sed -i "s/\\\$ClassName\\\$/${CLASS_NAME}/g" $SED_FILE
  sed -i "s/\\\$interface_type\\\$/${INTERFACE_TYPE}/g" $SED_FILE
  sed -i "s/\\\$Interface_Type\\\$/${INTERFACE_TYPE^}/g" $SED_FILE
done


# CMakeLists.txt: Remove comments if there any and add library
DEL_STRINGS=("# uncomment the following" "# further" "# find_package(<dependency>")

for DEL_STR in "${DEL_STRINGS[@]}"; do
  sed -i "/$DEL_STR/d" CMakeLists.txt
done

TMP_FILE=".f_tmp"
touch $TMP_FILE

# Get line with if(BUILD_TESTING)
TEST_LINE=`awk '$1 == "if(BUILD_TESTING)" { print NR }' CMakeLists.txt`
let CUT_LINE=$TEST_LINE-1
head -$CUT_LINE CMakeLists.txt >> $TMP_FILE

# Add Plugin library stuff inside
echo "add_library(" >> $TMP_FILE
echo "  $FILE_NAME" >> $TMP_FILE
echo "  SHARED" >> $TMP_FILE
echo "  $CTRL_CPP" >> $TMP_FILE
echo ")" >> $TMP_FILE

echo "target_include_directories(" >> $TMP_FILE
echo "  $FILE_NAME" >> $TMP_FILE
echo "  PUBLIC" >> $TMP_FILE
echo "  $<BUILD_INTERFACE:\${CMAKE_CURRENT_SOURCE_DIR}/include>" >> $TMP_FILE
echo "  $<INSTALL_INTERFACE:include>" >> $TMP_FILE
echo ")" >> $TMP_FILE

# TODO(anyone): Add this dependencies in a loop
echo "ament_target_dependencies(" >> $TMP_FILE
echo "  $FILE_NAME" >> $TMP_FILE
echo "  control_msgs" >> $TMP_FILE
echo "  controller_interface" >> $TMP_FILE
echo "  hardware_interface" >> $TMP_FILE
echo "  pluginlib" >> $TMP_FILE
echo "  rclcpp" >> $TMP_FILE
echo "  rclcpp_lifecycle" >> $TMP_FILE
echo "  realtime_tools" >> $TMP_FILE
echo ")" >> $TMP_FILE

# TODO(anyone): Delete after Foxy!!!
echo "# prevent pluginlib from using boost" >> $TMP_FILE
echo "target_compile_definitions($FILE_NAME PUBLIC \"PLUGINLIB__DISABLE_BOOST_FUNCTIONS\")" >> $TMP_FILE

if [[ "$package_configured" == "no" ]]; then

  echo "" >> $TMP_FILE
  echo "pluginlib_export_plugin_description_file(" >> $TMP_FILE
  echo "  controller_interface $PLUGIN_XML)" >> $TMP_FILE

  ## Add install directives
  echo "" >> $TMP_FILE
  echo "install(" >> $TMP_FILE
  echo "  TARGETS" >> $TMP_FILE
  echo "  $FILE_NAME" >> $TMP_FILE
  echo "  RUNTIME DESTINATION bin" >> $TMP_FILE
  echo "  ARCHIVE DESTINATION lib" >> $TMP_FILE
  echo "  LIBRARY DESTINATION lib" >> $TMP_FILE
  echo ")" >> $TMP_FILE

  if [[ ! `grep -q "DIRECTORY include/" $TMP_FILE` ]]; then
    echo "" >> $TMP_FILE
    echo "install(" >> $TMP_FILE
    echo "  DIRECTORY include/" >> $TMP_FILE
    echo "  DESTINATION include" >> $TMP_FILE
    echo ")" >> $TMP_FILE
  fi

fi

echo ""  >> $TMP_FILE

END_TEST_LINE=`tail -n +$TEST_LINE CMakeLists.txt | awk '$1 == "endif()" { print NR }'`
let CUT_LINE=$END_TEST_LINE-1
tail -n +$TEST_LINE CMakeLists.txt | head -$CUT_LINE >> $TMP_FILE

echo "" >> $TMP_FILE
echo "  ament_add_gmock(test_load_$FILE_NAME $LOAD_TEST_CPP)" >> $TMP_FILE
echo "  target_include_directories(test_load_$FILE_NAME PRIVATE include)" >> $TMP_FILE
echo "  ament_target_dependencies(" >> $TMP_FILE
echo "    test_load_$FILE_NAME" >> $TMP_FILE
echo "    controller_manager" >> $TMP_FILE
echo "    hardware_interface" >> $TMP_FILE
echo "    ros2_control_test_assets" >> $TMP_FILE
echo "  )" >> $TMP_FILE
echo ""

echo "" >> $TMP_FILE
echo "  ament_add_gmock(test_$FILE_NAME $TEST_CPP)" >> $TMP_FILE
echo "  target_include_directories(test_$FILE_NAME PRIVATE include)" >> $TMP_FILE
echo "  target_link_libraries(test_$FILE_NAME $FILE_NAME)" >> $TMP_FILE
echo "  ament_target_dependencies(" >> $TMP_FILE
echo "    test_$FILE_NAME" >> $TMP_FILE
echo "    controller_interface" >> $TMP_FILE
echo "    hardware_interface" >> $TMP_FILE
echo "  )" >> $TMP_FILE
echo ""

# Add export definitions
tail -n +$TEST_LINE CMakeLists.txt | head -$END_TEST_LINE | tail -1 >> $TMP_FILE

if [[ "$package_configured" == "no" ]]; then

  echo "" >> $TMP_FILE
  echo "ament_export_include_directories(" >> $TMP_FILE
  echo "  include" >> $TMP_FILE
  echo ")" >> $TMP_FILE

fi

echo "ament_export_libraries(" >> $TMP_FILE
echo "  $FILE_NAME" >> $TMP_FILE
echo ")" >> $TMP_FILE

if [[ "$package_configured" == "no" ]]; then
  # TODO(anyone): use this from a list so its the same as above
  echo "ament_export_dependencies(" >> $TMP_FILE
  echo "  control_msgs" >> $TMP_FILE
  echo "  controller_interface" >> $TMP_FILE
  echo "  hardware_interface" >> $TMP_FILE
  echo "  pluginlib" >> $TMP_FILE
  echo "  rclcpp" >> $TMP_FILE
  echo "  rclcpp_lifecycle" >> $TMP_FILE
  echo "  realtime_tools" >> $TMP_FILE
  echo ")" >> $TMP_FILE

fi

# Add last part
let CUT_LINE=$END_TEST_LINE+1
tail -n +$TEST_LINE CMakeLists.txt | tail -n +$CUT_LINE >> $TMP_FILE

mv $TMP_FILE CMakeLists.txt

# CMakeLists.txt & package.xml: Add dependencies if they not exist
DEP_PKGS=("realtime_tools" "rclcpp_lifecycle" "rclcpp" "pluginlib" "hardware_interface" "controller_interface" "control_msgs")

for DEP_PKG in "${DEP_PKGS[@]}"; do

  # CMakeLists.txt
  if `grep -q "find_package(${DEP_PKG} REQUIRED)" CMakeLists.txt`; then
    echo "'$DEP_PKG' is already dependency in CMakeLists.txt"
  else
    append_to_string="find_package(ament_cmake REQUIRED)"
    sed -i "s/$append_to_string/$append_to_string\\nfind_package(${DEP_PKG} REQUIRED)/g" CMakeLists.txt
  fi

  # package.xml
  if `grep -q "<depend>${DEP_PKG}</depend>" package.xml`; then
    echo "'$DEP_PKG' is already listed in package.xml"
  else
    append_to_string="<buildtool_depend>ament_cmake<\/buildtool_depend>"
    sed -i "s/$append_to_string/$append_to_string\\n\\n  <depend>${DEP_PKG}<\/depend>/g" package.xml
  fi

done

# CMakeLists.txt & package.xml: Add test dependencies if they not exist
TEST_DEP_PKGS=("ros2_control_test_assets" "hardware_interface" "controller_manager" "ament_cmake_gmock")

for DEP_PKG in "${TEST_DEP_PKGS[@]}"; do

  # CMakeLists.txt
  if `grep -q "  find_package(${DEP_PKG} REQUIRED)" CMakeLists.txt`; then
    echo "'$DEP_PKG' is already listed in CMakeLists.txt"
  else
    append_to_string="ament_lint_auto_find_test_dependencies()"
    sed -i "s/$append_to_string/$append_to_string\\n  find_package(${DEP_PKG} REQUIRED)/g" CMakeLists.txt
  fi

  # package.xml
  if `grep -q "<test_depend>${DEP_PKG}</test_depend>" package.xml`; then
    echo "'$DEP_PKG' is already listed in package.xml"
  else
    append_to_string="<test_depend>ament_lint_common<\/test_depend>"
    sed -i "s/$append_to_string/$append_to_string\\n  <test_depend>${DEP_PKG}<\/test_depend>/g" package.xml
  fi
done

# Remove lint dependencies because they should be not included into build
sed -i "/ament_lint_auto_find_test_dependencies()/d" CMakeLists.txt
sed -i "/<test_depend>ament_lint_common<\/test_depend>/d" package.xml

# extend README with general instructions
if [ -f README.md ]; then

  echo "" >> README.md
  echo "Pluginlib-Library: $FILE_NAME" >> README.md
  echo ""
  echo "Plugin: $PKG_NAME/${CLASS_NAME} (controller_interface::ControllerInterface)" >> README.md

fi

echo "Template files are adjusted."

git add .
# git commit -m "RosTeamWS: ros2_control skeleton files for $ROBOT_NAME generated."

# ament_uncrustify --reformat

# Compile and add new package the to the path
compile_and_source_package $PKG_NAME "yes"

echo ""
echo "FINISHED: Your package is set and the tests should be finished without any errors. (linter errors possible!)"
