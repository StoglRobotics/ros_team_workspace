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

##
## This script is run in setup-robot-bringup in case the user wants a gazebo starting point.
##

# Copy launch files
GAZEBO_LAUNCH="launch/${ROBOT_NAME}_sim.launch.py"
cp -n "$ROS2_CONTROL_TEMPLATES/robot_ros2_control_sim.launch.py" "${GAZEBO_LAUNCH}"
# sed all needed files
FILES_TO_SED=($GAZEBO_LAUNCH)
for SED_FILE in "${FILES_TO_SED[@]}"; do
  sed -i "s/\\\$PKG_NAME\\\$/${PKG_NAME}/g" $SED_FILE
  sed -i "s/\\\$RUNTIME_CONFIG_PKG_NAME\\\$/${PKG_NAME}/g" $SED_FILE
  sed -i "s/\\\$ROBOT_NAME\\\$/${ROBOT_NAME}/g" $SED_FILE
  sed -i "s/\\\$DESCR_PKG_NAME\\\$/${DESCR_PKG_NAME}/g" $SED_FILE
done

# package.xml: Add dependencies if they not exist
DEP_PKGS=("gz_ros2_control")

for DEP_PKG in "${DEP_PKGS[@]}"; do
  if $(grep -q $DEP_PKG package.xml); then
    echo "'$DEP_PKG' is already listed in package.xml"
  else
    append_to_string="<buildtool_depend>ament_cmake<\/buildtool_depend>"
    sed -i "s/$append_to_string/$append_to_string\\n\\n  <exec_depend>${DEP_PKG}<\/exec_depend>/g" package.xml
  fi
done