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

ROS_TEAM_WS_GIT_HTTPS_URL = "https://github.com/StoglRobotics/ros_team_workspace.git"

ROS_TEAM_WS_PATH = os.path.expanduser("~/.ros_team_workspace")
WORKSPACES_PATH = os.path.join(ROS_TEAM_WS_PATH, "workspaces.yaml")
CURRENT_FILE_DIR = pathlib.Path(__file__).parent.absolute()
USE_WORKSPACE_SCRIPT_PATH = os.path.normpath(
    CURRENT_FILE_DIR / ".." / ".." / ".." / "scripts" / "environment" / "setup.bash"
)
WORKSPACES_KEY = "workspaces"
ROS_TEAM_WS_RC_PATH = os.path.expanduser("~/.ros_team_ws_rc")
BACKUP_DATETIME_FORMAT = "%Y-%m-%d_%H-%M-%S-%f"
WORKSPACES_PATH_BACKUP_FORMAT = os.path.join(ROS_TEAM_WS_PATH, "bkp", "workspaces_bkp_{}.yaml")

SKEL_BASHRC_PATH = "/etc/skel/.bashrc"
BASHRC_PATH = os.path.expanduser("~/.bashrc")

WS_USE_BASH_FILE_PATH_FORMAT = "/tmp/ros_team_workspace/workspace_{ppid}.bash"

# constants for workspace field names
F_BASE_WS = "base_ws"
F_DISTRO = "distro"
F_DOCKER_TAG = "docker_tag"
F_WS_DOCKER_SUPPORT = "ws_docker_support"
F_WS_FOLDER = "ws_folder"
F_DOCKER_CONTAINER_NAME = "docker_container_name"
F_WS_NAME = "ws_name"

ROS_TEAM_WS_PREFIX = "RosTeamWS_"
ROS_TEAM_WS_BASE_WS_ENV_VAR = ROS_TEAM_WS_PREFIX + F_BASE_WS.upper()
ROS_TEAM_WS_DISTRO_ENV_VAR = ROS_TEAM_WS_PREFIX + F_DISTRO.upper()
ROS_TEAM_WS_DOCKER_TAG_ENV_VAR = ROS_TEAM_WS_PREFIX + F_DOCKER_TAG.upper()
ROS_TEAM_WS_WS_DOCKER_SUPPORT_ENV_VAR = ROS_TEAM_WS_PREFIX + F_WS_DOCKER_SUPPORT.upper()
ROS_TEAM_WS_WS_FOLDER_ENV_VAR = ROS_TEAM_WS_PREFIX + F_WS_FOLDER.upper()
ROS_TEAM_WS_DOCKER_CONTAINER_NAME_ENV_VAR = ROS_TEAM_WS_PREFIX + F_DOCKER_CONTAINER_NAME.upper()
ROS_TEAM_WS_WS_NAME_ENV_VAR = ROS_TEAM_WS_PREFIX + F_WS_NAME.upper()

ROS_TEAM_WS_ENV_VARIABLES = [
    ROS_TEAM_WS_BASE_WS_ENV_VAR,
    ROS_TEAM_WS_DISTRO_ENV_VAR,
    ROS_TEAM_WS_WS_FOLDER_ENV_VAR,
    ROS_TEAM_WS_WS_DOCKER_SUPPORT_ENV_VAR,
    ROS_TEAM_WS_DOCKER_TAG_ENV_VAR,
]

DISPLAY_MANAGER_WAYLAND = "wayland"

RICH_TREE_LABEL_COLOR = "[bold green]"
RICH_TREE_GUIDE_STYLE = "bold"
RICH_TREE_FST_LVL_COLOR = "[bold blue]"
RICH_TREE_SND_LVL_COLOR = "[yellow]"
RICH_TREE_TRD_LVL_COLOR = "[cyan]"
RICH_TREE_STATUS_COLOR = "[bold magenta]"
