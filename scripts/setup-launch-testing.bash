#!/bin/bash
#
# Copyright (c) 2021-2026, b»robotized group
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

usage="setup-launch-testing"

# Load Framework defines
script_own_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
source "$script_own_dir"/../setup.bash
check_and_set_ros_distro_and_version "${ROS_DISTRO}"

if [ ! -f "package.xml" ]; then
  print_and_exit "ERROR: 'package.xml' not found. Run this script from the top-level of your package folder. Nothing to do 😯" "$usage"
fi
PKG_NAME="$(grep -Po '(?<=<name>).*?(?=</name>)' package.xml | sed -e 's/[[:space:]]//g')"

if [ ! -f "CMakeLists.txt" ]; then
  print_and_exit "ERROR: 'CMakeLists.txt' not found. Run this script from the top-level of your package folder. Nothing to do 😯" "$usage"
fi

echo ""
echo -e "${TERMINAL_COLOR_USER_NOTICE}ATTENTION: Setting up launch_testing for package '$PKG_NAME' in folder '$(pwd)'.${TERMINAL_COLOR_NC}"
echo -e "${TERMINAL_COLOR_USER_CONFIRMATION}If correct press <ENTER>, otherwise <CTRL>+C and start the script again from the correct package folder.${TERMINAL_COLOR_NC}"
read

# Convert PKG_NAME snake_case to CamelCase for the test class name (e.g. my_pkg -> MyPkg)
PKG_NAME_CC="$(echo "$PKG_NAME" | sed 's/_\([a-z]\)/\U\1/g; s/^\([a-z]\)/\U\1/')"

# Create test directory
mkdir -p test

# Copy the template launch test file (skip if it already exists)
LAUNCH_TEST_FILE="test/test_${PKG_NAME}.launch.py"
if [ -f "$LAUNCH_TEST_FILE" ]; then
  echo "Test file '$LAUNCH_TEST_FILE' already exists — skipping copy."
else
  cp "$TESTING_TEMPLATES/test_package_name.launch.py" "$LAUNCH_TEST_FILE"
  # Substitute placeholders
  sed -i "s/TestPkgName/Test${PKG_NAME_CC}/g" "$LAUNCH_TEST_FILE"
  sed -i "s/\\\$PKG_NAME\\\$/${PKG_NAME}/g" "$LAUNCH_TEST_FILE"
  echo "Created '$LAUNCH_TEST_FILE'."
fi

# package.xml: Add test dependencies if not already present
DEP_PKGS=("ament_cmake_ros" "launch" "launch_ros" "launch_testing" "launch_testing_ament_cmake" "rclpy")

PREVIOUS_STRING="$(grep -E "^\s*<test_depend>" package.xml | tail -n 1)"
if [ -z "$PREVIOUS_STRING" ]; then
  PREVIOUS_STRING="$(grep -E "^\s*<exec_depend>" package.xml | tail -n 1)"
fi
if [ -z "$PREVIOUS_STRING" ]; then
  PREVIOUS_STRING="<export>"
fi

DEPEND_TAG="test_depend"
for DEP_PKG in "${DEP_PKGS[@]}"; do
  if grep -q "<${DEPEND_TAG}>${DEP_PKG}</${DEPEND_TAG}>" package.xml; then
    echo "'$DEP_PKG' is already listed in package.xml"
  else
    if [[ ! "$PREVIOUS_STRING" =~ ^\s*\<${DEPEND_TAG}.* ]]; then
      newline="\n\n"
    else
      newline="\n"
    fi
    sed -i "s#${PREVIOUS_STRING}#${PREVIOUS_STRING}${newline}  <${DEPEND_TAG}>${DEP_PKG}</${DEPEND_TAG}>#g" package.xml
    PREVIOUS_STRING="  <${DEPEND_TAG}>${DEP_PKG}</${DEPEND_TAG}>"
  fi
done

# CMakeLists.txt: Add option(BUILD_TESTING) before the if(BUILD_TESTING) block if missing
if ! grep -q 'option(BUILD_TESTING' CMakeLists.txt; then
  sed -i 's/if(BUILD_TESTING)/option(BUILD_TESTING "Build tests" ON)\n\nif(BUILD_TESTING)/' CMakeLists.txt
fi

# CMakeLists.txt: Add the launch test inside if(BUILD_TESTING); handle both existing and missing blocks
if grep -q "add_launch_test" CMakeLists.txt; then
  echo "add_launch_test already present in CMakeLists.txt — skipping."
elif grep -q "if(BUILD_TESTING)" CMakeLists.txt; then
  lines_to_append="  find_package(ament_cmake_ros REQUIRED)\n  find_package(launch_testing_ament_cmake REQUIRED)\n\n  add_launch_test(\n    test\/test_${PKG_NAME}.launch.py\n    TIMEOUT 600  # seconds; increase for slow CI machines\n  )"
  sed -i "/if(BUILD_TESTING)/a\\$lines_to_append" CMakeLists.txt
else
  # No BUILD_TESTING block at all — insert one before ament_package()
  testing_block="option(BUILD_TESTING \"Build tests\" ON)\n\nif(BUILD_TESTING)\n  find_package(ament_cmake_ros REQUIRED)\n  find_package(launch_testing_ament_cmake REQUIRED)\n\n  add_launch_test(\n    test\/test_${PKG_NAME}.launch.py\n    TIMEOUT 600  # seconds; increase for slow CI machines\n  )\nendif()"
  sed -i "s/ament_package()/${testing_block}\n\nament_package()/" CMakeLists.txt
fi

# Compile and source the updated package
compile_and_source_package "$PKG_NAME"

echo ""
echo -e "${TERMINAL_COLOR_USER_NOTICE}FINISHED: launch_testing setup complete for package '$PKG_NAME'.${TERMINAL_COLOR_NC}"
echo -e "${TERMINAL_COLOR_USER_NOTICE}  Test file: $LAUNCH_TEST_FILE${TERMINAL_COLOR_NC}"
echo -e "${TERMINAL_COLOR_USER_NOTICE}  Build:     cb ${PKG_NAME}${TERMINAL_COLOR_NC}"
echo -e "${TERMINAL_COLOR_USER_NOTICE}  Test:      ct ${PKG_NAME}${TERMINAL_COLOR_NC}"
echo -e "${TERMINAL_COLOR_USER_NOTICE}  Results:   ctres${TERMINAL_COLOR_NC}"
echo -e "${TERMINAL_COLOR_USER_NOTICE}  Verbose:   colcon test --event-handlers console_direct+ --packages-select ${PKG_NAME}${TERMINAL_COLOR_NC}"
