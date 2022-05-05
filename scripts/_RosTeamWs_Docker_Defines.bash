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

## BEGIN: Default RosTeamWS Docker Definitions and Aliases

setup-ros-workspace-docker () {
   source "$RosTeamWS_FRAMEWORK_SCRIPTS_PATH"/setup-ros-workspace.bash
   create_workspace_docker "$@"
}

rtw_switch_to_docker () {
  if [ -z "$RosTeamWS_WS_DOCKER_SUPPORT" ] || [ "$RosTeamWS_WS_DOCKER_SUPPORT" == false ]; then
    print_and_exit"It seems your current workspace does not support docker. If it should, did you activate it by executing _\"<ws_alias>\"?"
  fi

  source "$RosTeamWS_FRAMEWORK_SCRIPTS_PATH"/docker/_RosTeamWs_Docker_Defines.bash
  start_and_connect_user_to_docker "$RosTeamWS_DOCKER_TAG"
}

rtw_switch_to_docker_root () {
  if [ -z "$RosTeamWS_WS_DOCKER_SUPPORT" ] || [ "$RosTeamWS_WS_DOCKER_SUPPORT" == false ]; then
    print_and_exit "It seems your current workspace does not support docker. If it should, did you activate it by executing _\"<ws_alias>\"?"
  fi

  source "$RosTeamWS_FRAMEWORK_SCRIPTS_PATH"/docker/_RosTeamWs_Docker_Defines.bash
  start_and_connect_root_to_docker "$RosTeamWS_DOCKER_TAG"
}

rtw_stop_docker () {
  if [ -z "$RosTeamWS_WS_DOCKER_SUPPORT" ] || [ "$RosTeamWS_WS_DOCKER_SUPPORT" == false ]; then
    print_and_exit "It seems your current workspace does not support docker. If it should, did you activate it by executing _\"<ws_alias>\"?"
  fi

  source "$RosTeamWS_FRAMEWORK_SCRIPTS_PATH"/docker/_RosTeamWs_Docker_Defines.bash
  stop_docker_container "$RosTeamWS_DOCKER_TAG"
}
