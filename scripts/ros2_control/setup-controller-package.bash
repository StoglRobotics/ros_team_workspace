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

usage="setup-controller-package.bash FILE_NAME [CLASS_NAME]"

# Load Framework defines
script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $script_own_dir/../../setup.bash

check_and_set_ros_distro_and_version "${ROS_DISTRO}"

FILE_NAME=$1
if [ -z "$1" ]; then
  print_and_exit "You should provide the file name! Nothing to do 😯" "$usage"
fi
if [ -f src/$FILE_NAME.cpp ]; then
  print_and_exit "ERROR:The file '$FILE_NAME' already exist! 😱!" "$usage"
fi

if [ ! -f "package.xml" ]; then
  print_and_exit "ERROR: 'package.xml' not found. You should execute this script at the top level of your package folder. Nothing to do 😯" "$usage"
fi
PKG_NAME="$(grep -Po '(?<=<name>).*?(?=</name>)' package.xml | sed -e 's/[[:space:]]//g')"

echo ""  # Adds empty line

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
  echo -e "${TERMINAL_COLOR_USER_CONFIRMATION}ClassName guessed from the '$FILE_NAME': '$CLASS_NAME'. Is this correct? If not, provide it as the second parameter.${TERMINAL_COLOR_NC}"
fi

echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}Which license-header do you want to use? [1]"
echo "(0) None"
echo "(1) Apache-2.0"
echo "(2) Proprietary"
echo -n -e "${TERMINAL_COLOR_NC}"
read choice
choice=${choice:="1"}

if [ "$choice" != 0 ]; then
  echo -n -e "${TERMINAL_COLOR_USER_INPUT_DECISION}Insert your company or personal name (copyright): ${TERMINAL_COLOR_NC}"
  read NAME_ON_LICENSE
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
esac

echo -n -e "${TERMINAL_COLOR_USER_INPUT_DECISION}Is package already configured (is in there a working controller already)? (yes/no) [no] ${TERMINAL_COLOR_NC}"
read package_configured
package_configured=${package_configured:="no"}

echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}Do you want to setup a 'normal' or 'chainable' controller? [1]"
echo "(1) normal (single-level control)"
echo "(2) chainable"
echo -n -e "${TERMINAL_COLOR_NC}"
read choice
choice=${choice:="1"}

CONTROLLER_TYPE=""
case "$choice" in
"1")
  CONTROLLER_TYPE="normal (single-level control)"
  ;;
"2")
  CONTROLLER_TYPE="chainable"
esac

echo ""
echo -e "${TERMINAL_COLOR_USER_NOTICE}ATTENTION: Setting up ros2_control controller files with following parameters: file name '$FILE_NAME', class '$CLASS_NAME', package/namespace '$PKG_NAME', type '$CONTROLLER_TYPE'. Those will be placed in folder '`pwd`'.${TERMINAL_COLOR_NC}"
echo ""
echo -e "${TERMINAL_COLOR_USER_CONFIRMATION}If correct press <ENTER>, otherwise <CTRL>+C and start the script again from the package folder and/or with correct controller name.${TERMINAL_COLOR_NC}"
read

# Add folders if deleted
ADD_FOLDERS=("include/$PKG_NAME" "src" "test")

for FOLDER in "${ADD_FOLDERS[@]}"; do
    mkdir -p $FOLDER
done

# Set file constants
VC_H="include/$PKG_NAME/visibility_control.h"
CTRL_HPP="include/$PKG_NAME/$FILE_NAME.hpp"
CTRL_CPP="src/$FILE_NAME.cpp"
CTRL_PARAMS_YAML="src/$FILE_NAME.yaml"
CTRL_VALIDATE_PARAMS_HPP="include/$PKG_NAME/validate_${FILE_NAME}_parameters.hpp"
PLUGIN_XML="$PKG_NAME.xml"
LOAD_TEST_CPP="test/test_load_$FILE_NAME.cpp"
TEST_CPP="test/test_$FILE_NAME.cpp"
TEST_PRECEEDING_CPP="test/test_${FILE_NAME}_preceeding.cpp"
TEST_HPP="test/test_$FILE_NAME.hpp"
TEST_PARAMS_YAML="test/${FILE_NAME}_params.yaml"
TEST_PRECEEDING_PARAMS_YAML="test/${FILE_NAME}_preceeding_params.yaml"

# Copy files
if [[ ! -f "$VC_H" ]]; then
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_package_namespace/visibility_control.h $VC_H
fi

if [[ "$CONTROLLER_TYPE" == "chainable" ]]; then
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_package_namespace/dummy_chainable_controller.hpp $CTRL_HPP
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_chainable_controller.cpp $CTRL_CPP
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_controller.yaml $CTRL_PARAMS_YAML
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_package_namespace/validate_dummy_controller_parameters.hpp $CTRL_VALIDATE_PARAMS_HPP
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/test_dummy_chainable_controller.cpp $TEST_CPP
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/test_dummy_chainable_controller_preceeding.cpp $TEST_PRECEEDING_CPP
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/test_dummy_chainable_controller.hpp $TEST_HPP
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_controller_params.yaml $TEST_PARAMS_YAML
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_controller_preceeding_params.yaml $TEST_PRECEEDING_PARAMS_YAML
  cat $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_chainable_controller_pluginlib.xml >> $PLUGIN_XML
else
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_package_namespace/dummy_controller.hpp $CTRL_HPP
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_controller.cpp $CTRL_CPP
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_controller.yaml $CTRL_PARAMS_YAML
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_package_namespace/validate_dummy_controller_parameters.hpp $CTRL_VALIDATE_PARAMS_HPP
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/test_dummy_controller.cpp $TEST_CPP
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/test_dummy_controller_preceeding.cpp $TEST_PRECEEDING_CPP
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/test_dummy_controller.hpp $TEST_HPP
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_controller_params.yaml $TEST_PARAMS_YAML
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_controller_preceeding_params.yaml $TEST_PRECEEDING_PARAMS_YAML
  cat $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_controller_pluginlib.xml >> $PLUGIN_XML
fi

cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/test_load_dummy_controller.cpp $LOAD_TEST_CPP

echo -e "${TERMINAL_COLOR_USER_NOTICE}Template files copied.${TERMINAL_COLOR_NC}"

# TODO(anyone): fuse this with hardware interface package creating.

# Add license header to the files
FILES_TO_LICENSE=("$CTRL_HPP" "$CTRL_CPP" "$CTRL_VALIDATE_PARAMS_HPP" "$LOAD_TEST_CPP" "$TEST_CPP" "$TEST_PRECEEDING_CPP" "$TEST_HPP")
if [[ "$package_configured" == "no" ]]; then
  FILES_TO_LICENSE+=("$VC_H")
fi
TMP_FILE=".f_tmp"
if [[ "$LICENSE_HEADER" != "" ]]; then
  touch $TMP_FILE
  for FILE_TO_LIC in "${FILES_TO_LICENSE[@]}"; do
    cat $LICENSE_HEADER > $TMP_FILE
    sed "1,13d" $FILE_TO_LIC >> $TMP_FILE # delete first 13 lines which correspond to fake license
    mv $TMP_FILE $FILE_TO_LIC
    sed -i "s/\\\$YEAR\\\$/${YEAR_ON_LICENSE}/g" $FILE_TO_LIC
    sed -i "s/\\\$NAME_ON_LICENSE\\\$/${NAME_ON_LICENSE}/g" $FILE_TO_LIC
  done
#   echo "Licence header added to files: ("`declare -p FILES_TO_LICENSE`")"
fi

# sed all needed files
FILES_TO_SED=("${FILES_TO_LICENSE[@]}")
FILES_TO_SED+=("$PLUGIN_XML" "$CTRL_PARAMS_YAML" "$TEST_PARAMS_YAML" "$TEST_PRECEEDING_PARAMS_YAML")
# declare -p FILES_TO_SED

for SED_FILE in "${FILES_TO_SED[@]}"; do
  sed -i "s/TEMPLATES__ROS2_CONTROL__CONTROLLER__DUMMY_PACKAGE_NAMESPACE/${PKG_NAME^^}/g" $SED_FILE # package name for include guard
  sed -i "s/TEMPLATES__ROS2_CONTROL__VISIBILITY/${PKG_NAME^^}__VISIBILITY/g" $SED_FILE # visibility defines
  sed -i "s/dummy_package_namespace/${PKG_NAME}/g" $SED_FILE # package name for includes
  sed -i "s/dummy_controller/${FILE_NAME}/g" $SED_FILE # file name
  sed -i "s/dummy_chainable_controller/${FILE_NAME}/g" $SED_FILE # file name
  sed -i "s/DUMMY_CONTROLLER/${FILE_NAME^^}/g" $SED_FILE # file name for include guard
  sed -i "s/DUMMY_CHAINABLE_CONTROLLER/${FILE_NAME^^}/g" $SED_FILE # file name for include guard
  sed -i "s/DummyClassName/${CLASS_NAME}/g" $SED_FILE # class name
  sed -i "s/dummy_interface_type/${INTERFACE_TYPE}/g" $SED_FILE # interface type for includes
  sed -i "s/Dummy_Interface_Type/${INTERFACE_TYPE^}/g" $SED_FILE # Interface type in namespace resolution
done


# CMakeLists.txt: Remove comments if there any and add library
DEL_STRINGS=("# uncomment the following" "# further" "# find_package(<dependency>")

for DEL_STR in "${DEL_STRINGS[@]}"; do
  sed -i "/$DEL_STR/d" CMakeLists.txt
done

TMP_FILE=".f_tmp"
touch $TMP_FILE

# manage dependencies here
if [[ "$package_configured" == "no" ]]; then

  # Add `THIS_PACKAGE_INCLUDE_DEPENDS` structure to simplify the rest of the file
  TEST_LINE=`awk '$1 == "find_package(ament_cmake" { print NR }' CMakeLists.txt`  # get line before `ament_cmake` dependency
  let CUT_LINE=$TEST_LINE-1
  head -$CUT_LINE CMakeLists.txt >> $TMP_FILE

  echo "set(THIS_PACKAGE_INCLUDE_DEPENDS" >> $TMP_FILE
  echo "  control_msgs" >> $TMP_FILE
  echo "  controller_interface" >> $TMP_FILE
  echo "  hardware_interface" >> $TMP_FILE
  echo "  pluginlib" >> $TMP_FILE
  echo "  rclcpp" >> $TMP_FILE
  echo "  rclcpp_lifecycle" >> $TMP_FILE
  echo "  realtime_tools" >> $TMP_FILE
  echo "  std_srvs" >> $TMP_FILE
  echo ")" >> $TMP_FILE

  echo "" >> $TMP_FILE
  echo "find_package(ament_cmake REQUIRED)" >> $TMP_FILE
  echo "find_package(generate_parameter_library REQUIRED)" >> $TMP_FILE
  echo "foreach(Dependency IN ITEMS \${THIS_PACKAGE_INCLUDE_DEPENDS})" >> $TMP_FILE
  echo "  find_package(\${Dependency} REQUIRED)" >> $TMP_FILE
  echo "endforeach()" >> $TMP_FILE
  echo "" >> $TMP_FILE
fi

# Get line with if(BUILD_TESTING)
TEST_LINE=`awk '$1 == "if(BUILD_TESTING)" { print NR }' CMakeLists.txt`
let CUT_LINE=$TEST_LINE-1

# Add Plugin library related stuff
echo "# Add ${FILE_NAME} library related compile commands" >> $TMP_FILE
echo "generate_parameter_library(${FILE_NAME}_parameters" >> $TMP_FILE
echo "  ${CTRL_PARAMS_YAML}" >> $TMP_FILE
echo "  ${CTRL_VALIDATE_PARAMS_HPP}" >> $TMP_FILE
echo ")" >> $TMP_FILE

echo "add_library(" >> $TMP_FILE
echo "  $FILE_NAME" >> $TMP_FILE
echo "  SHARED" >> $TMP_FILE
echo "  $CTRL_CPP" >> $TMP_FILE
echo ")" >> $TMP_FILE
echo "target_include_directories($FILE_NAME PUBLIC" >> $TMP_FILE
echo '  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"' >> $TMP_FILE
echo '  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")' >> $TMP_FILE
echo "target_link_libraries($FILE_NAME ${FILE_NAME}_parameters)" >> $TMP_FILE
echo "ament_target_dependencies($FILE_NAME \${THIS_PACKAGE_INCLUDE_DEPENDS})" >> $TMP_FILE
echo "target_compile_definitions($FILE_NAME PRIVATE \"${FILE_NAME^^}_BUILDING_DLL\")" >> $TMP_FILE

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
    echo '  DESTINATION include/${PROJECT_NAME}' >> $TMP_FILE
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
echo "  add_rostest_with_parameters_gmock(test_$FILE_NAME $TEST_CPP \${CMAKE_CURRENT_SOURCE_DIR}/${TEST_PARAMS_YAML})" >> $TMP_FILE
echo "  target_include_directories(test_$FILE_NAME PRIVATE include)" >> $TMP_FILE
echo "  target_link_libraries(test_$FILE_NAME $FILE_NAME)" >> $TMP_FILE
echo "  ament_target_dependencies(" >> $TMP_FILE
echo "    test_$FILE_NAME" >> $TMP_FILE
echo "    controller_interface" >> $TMP_FILE
echo "    hardware_interface" >> $TMP_FILE
echo "  )" >> $TMP_FILE
echo ""

echo "" >> $TMP_FILE
echo "  add_rostest_with_parameters_gmock(test_${FILE_NAME}_preceeding $TEST_PRECEEDING_CPP \${CMAKE_CURRENT_SOURCE_DIR}/${TEST_PRECEEDING_PARAMS_YAML})" >> $TMP_FILE
echo "  target_include_directories(test_${FILE_NAME}_preceeding PRIVATE include)" >> $TMP_FILE
echo "  target_link_libraries(test_${FILE_NAME}_preceeding $FILE_NAME)" >> $TMP_FILE
echo "  ament_target_dependencies(" >> $TMP_FILE
echo "    test_${FILE_NAME}_preceeding" >> $TMP_FILE
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
  echo "ament_export_dependencies(" >> $TMP_FILE
  echo '  ${THIS_PACKAGE_INCLUDE_DEPENDS}' >> $TMP_FILE
  echo ")" >> $TMP_FILE
fi

echo "ament_export_libraries(" >> $TMP_FILE
echo "  $FILE_NAME" >> $TMP_FILE
echo ")" >> $TMP_FILE

# Add last part
let CUT_LINE=$END_TEST_LINE+1
tail -n +$TEST_LINE CMakeLists.txt | tail -n +$CUT_LINE >> $TMP_FILE

mv $TMP_FILE CMakeLists.txt


# manage dependencies in package.xml
if [[ "$package_configured" == "no" ]]; then

  # append `generate_parameter_library` to the package.xml file
  append_to_string="<buildtool_depend>ament_cmake<\/buildtool_depend>"
  sed -i "s/$append_to_string/$append_to_string\\n\\n  <build_depend>generate_parameter_library<\/build_depend>/g" package.xml

  # CMakeLists.txt & package.xml: Add dependencies if they not exist
  DEP_PKGS=("std_srvs" "realtime_tools" "rclcpp_lifecycle" "rclcpp" "pluginlib" "hardware_interface" "controller_interface" "control_msgs")

  for DEP_PKG in "${DEP_PKGS[@]}"; do
    # package.xml
    if `grep -q "<depend>${DEP_PKG}</depend>" package.xml`; then
      echo "'$DEP_PKG' is already listed in package.xml"
    else
      append_to_string="<build_depend>generate_parameter_library<\/build_depend>"
      sed -i "s/$append_to_string/$append_to_string\\n\\n  <depend>${DEP_PKG}<\/depend>/g" package.xml
    fi
  done
fi


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

echo -e "${TERMINAL_COLOR_USER_NOTICE}Template files were adjusted.${TERMINAL_COLOR_NC}"

git add .
# git commit -m "RosTeamWS: ros2_control skeleton files for $ROBOT_NAME generated."

# Compile and add new package the to the path
compile_and_source_package $PKG_NAME "yes"

echo ""
echo -e "${TERMINAL_COLOR_USER_NOTICE}FINISHED: Your package is set and the tests should be finished without any errors. (linter errors possible!)${TERMINAL_COLOR_NC}"
