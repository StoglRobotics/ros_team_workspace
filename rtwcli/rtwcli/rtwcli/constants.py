# Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import pathlib


WORKSPACES_PATH = os.path.expanduser("~/.ros_team_workspace/workspaces.yaml")
CURRENT_FILE_DIR = pathlib.Path(__file__).parent.absolute()
USE_WORKSPACE_SCRIPT_PATH = os.path.normpath(
    CURRENT_FILE_DIR / ".." / ".." / ".." / "scripts" / "environment" / "setup.bash"
)
WORKSPACES_KEY = "workspaces"
ROS_TEAM_WS_RC_PATH = os.path.expanduser("~/.ros_team_ws_rc")
BACKUP_DATETIME_FORMAT = "%Y-%m-%d_%H-%M-%S-%f"
WORKSPACES_PATH_BACKUP_FORMAT = os.path.expanduser(
    "~/.ros_team_workspace/bkp/workspaces_bkp_{}.yaml"
)
WS_FOLDER_ENV_VAR = "RosTeamWS_WS_FOLDER"
ROS_TEAM_WS_PREFIX = "RosTeamWS_"

# constants for workspace field names
F_BASE_WS = "base_ws"
F_DISTRO = "distro"
F_DOCKER_TAG = "docker_tag"
F_WS_DOCKER_SUPPORT = "ws_docker_support"
F_WS_FOLDER = "ws_folder"
F_DOCKER_CONTAINER_NAME = "docker_container_name"

SKEL_BASHRC_PATH = "/etc/skel/.bashrc"
BASHRC_PATH = os.path.expanduser("~/.bashrc")

ROS_TEAM_WS_ENV_VARIABLES = [
    "RosTeamWS_BASE_WS",
    "RosTeamWS_DISTRO",
    "RosTeamWS_WS_FOLDER",
    "RosTeamWS_WS_DOCKER_SUPPORT",
    "RosTeamWS_DOCKER_TAG",
]
