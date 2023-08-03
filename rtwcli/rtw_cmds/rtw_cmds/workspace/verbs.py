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

"""
workspace concept.

§§§ general configuration: "~/.ros_team_workspace/config.yaml
alternative_ros_rolling_location=""
team_license="Apache License 2.0"
team_repository_server="https://github.com"
team_private_config_path=""

§§§ ros related params: "~/.ros_team_workspace/ros_config.yaml
  - ros_env:
    - ros_domain_id: 34

§§§ workspaces: "~/.ros_team_workspace/workspaces.yaml"
<workspace_name>:
  - base_ws:
  - ...
  - docker_tag:
  - ros_env:
    - ros_domain_id: 1

workspace name prefix is defined as first letters of the path and last folder if not "workspace":
- /home/daniela/workspace/kuka_rolling_ws -> hdw__kuka_rolling_ws
- /home/daniela/workspace/mojin/pid_ws -> hdw_mojin__pid_ws
"""

import copy
import dataclasses
import os
import re
import shutil
import questionary
from typing import Dict, List
from rtwcli.command.info import ROS_TEAM_WS_VARIABLES
from rtwcli.helpers import (
    create_file_and_write,
    create_file_if_not_exists,
    load_yaml_file,
    write_to_yaml_file,
)
from rtwcli.verb import VerbExtension
from send2trash import send2trash

WORKSPACES_PATH = os.path.expanduser("~/.ros_team_workspace/workspaces.yaml")
USE_WORKSPACE_SCRIPT_PATH = os.path.expanduser(
    "~/ros_team_workspace/scripts/environment/setup.bash"
)
WORKSPACES_KEY = "workspaces"
ROS_TEAM_WS_RC_PATH = os.path.expanduser("~/.ros_team_ws_rc")
WORKSPACES_PATH_BACKUP = os.path.expanduser("~/.ros_team_workspace/workspaces_backup.yaml")
TRASH_FOLDER = os.path.expanduser("~/.local/share/Trash/ros_team_workspace")


@dataclasses.dataclass
class Workspace:
    base_ws: str
    distro: str
    docker_tag: str
    ws_docker_support: bool
    ws_folder: str


@dataclasses.dataclass
class WorkspacesConfig:
    workspaces: Dict[str, Workspace]

    @classmethod
    def from_dict(cls, data: Dict[str, Dict[str, dict]]) -> "WorkspacesConfig":
        if not data:
            return cls({})

        workspaces = {
            ws_name: Workspace(**ws_data)
            for ws_name, ws_data in data.get(WORKSPACES_KEY, {}).items()
        }
        return cls(workspaces)

    def to_dict(self) -> Dict[str, Dict[str, dict]]:
        return {
            WORKSPACES_KEY: {
                ws_name: dataclasses.asdict(ws) for ws_name, ws in self.workspaces.items()
            }
        }

    def get_ws_names(self) -> List[str]:
        if not self.workspaces:
            return []
        return list(self.workspaces.keys())


def load_workspaces_config_from_yaml_file(file_path: str):
    return WorkspacesConfig.from_dict(load_yaml_file(file_path))


def save_workspaces_config(filepath: str, config: WorkspacesConfig):
    return write_to_yaml_file(filepath, config.to_dict())


def update_workspaces_config(config_path: str, ws_name: str, workspace: Workspace) -> bool:
    if not create_file_if_not_exists(config_path):
        print("Could not create workspaces config file. Cannot proceed with porting.")
        return False

    workspaces_config = load_workspaces_config_from_yaml_file(config_path)

    if ws_name in workspaces_config.get_ws_names():
        print(
            f"Workspace name '{ws_name}' is already in the config. "
            "Duplicate workspace name handling is not implemented yet."
        )
        return False

    workspaces_config.workspaces[ws_name] = workspace

    # Backup current config file
    shutil.copy(config_path, WORKSPACES_PATH_BACKUP)
    print(f"Backed up current workspaces config file to '{WORKSPACES_PATH_BACKUP}'")

    if not save_workspaces_config(config_path, workspaces_config):
        print(f"Failed to update YAML file '{config_path}'.")
        return False

    print(f"Updated YAML file '{config_path}' with a new workspace '{ws_name}'")

    # Move backup to trash
    send2trash(WORKSPACES_PATH_BACKUP)
    print(f"Moved backup config file '{WORKSPACES_PATH_BACKUP}' to the trash")

    return True


def extract_workspaces_from_bash_script(script_path: str) -> Dict[str, dict]:
    with open(script_path) as file:
        data = file.read()

    workspaces = re.findall(r"(\w+ \(\) \{[^}]+\})", data, re.MULTILINE)
    workspaces_dict = {}
    for workspace in workspaces:
        ws_name = re.search(r"(\w+) \(", workspace).group(1)
        workspaces_dict[ws_name] = {}
        variables = re.findall(r'(?:export )?(\w+)=(".*?"|\'.*?\'|[^ ]+)', workspace)
        for var, val in variables:
            workspaces_dict[ws_name][var] = val.strip('"')

    return workspaces_dict


class CreateVerb(VerbExtension):
    """Create a new ROS workspace."""

    def main(self, *, args):
        print("Not implemented yet")


class UseVerb(VerbExtension):
    """Select and source an existing ROS workspace."""

    def main(self, *, args):
        if not os.path.isfile(WORKSPACES_PATH):
            print(
                "No workspaces are available as the workspaces config file "
                f"'{WORKSPACES_PATH}' does not exist"
            )
            return

        workspaces_config = load_workspaces_config_from_yaml_file(WORKSPACES_PATH)
        if not workspaces_config.workspaces:
            print(f"No workspaces found in config file '{WORKSPACES_PATH}'")
            return

        ws_names = workspaces_config.get_ws_names()
        ws_name = questionary.autocomplete(
            "Choose workspace",
            ws_names,
            qmark="'Tab' to see all workspaces\n",
            meta_information=copy.deepcopy(workspaces_config.to_dict()[WORKSPACES_KEY]),
            validate=lambda ws_choice: ws_choice in ws_names,
            style=questionary.Style([("answer", "bg:ansiwhite")]),
        ).ask()
        if not ws_name:
            return

        workspace = workspaces_config.workspaces[ws_name]
        print(f"Workspace data: {workspace}")

        distro = workspace.distro
        ws_folder = workspace.ws_folder
        to_write = "#!/bin/bash\n"
        ws_data = [
            ("base_ws", workspace.base_ws),
            ("distro", workspace.distro),
            ("docker_tag", workspace.docker_tag),
            ("ws_docker_support", workspace.ws_docker_support),
            ("ws_folder", workspace.ws_folder),
        ]
        for ws_var, ws_var_value in ws_data:
            new_var = "RosTeamWS_" + ws_var.upper()
            to_write += f"export {new_var}='{ws_var_value}'\n"
        to_write += f"source {USE_WORKSPACE_SCRIPT_PATH} {distro} {ws_folder}\n"
        tmp_file = f"/tmp/ros_team_workspace/wokspace_{os.getppid()}.bash"
        print(f"Following text will be written into file '{tmp_file}':\n{to_write}")
        if not create_file_and_write(tmp_file, content=to_write):
            print(f"Failed to write workspace data to a file {tmp_file}.")
            return


class PortAllVerb(VerbExtension):
    """Port the currently sourced ROS workspace by creating the corresponding config."""

    def main(self, *, args):
        print(f"Reading workspaces from script '{ROS_TEAM_WS_RC_PATH}'")
        script_workspaces = extract_workspaces_from_bash_script(ROS_TEAM_WS_RC_PATH)
        ws_num = len(script_workspaces)
        print(f"Found {ws_num} workspaces in script '{ROS_TEAM_WS_RC_PATH}'")
        for i, (script_ws, script_ws_data) in enumerate(script_workspaces.items()):
            print(
                f"Processing {i+1}/{ws_num} script workspace '{script_ws}',"
                f" ws_data: {script_ws_data}"
            )
            workspace_data_to_port = {}
            str_format = "\t{:<30} -> {:<20}: {}"
            for var, value in script_ws_data.items():
                new_var = var.replace("RosTeamWS_", "").lower()
                print(str_format.format(var, new_var, value))
                workspace_data_to_port[new_var] = value

            print("Generating workspace name from workspace path with first folder letters: ")
            ws_folder = script_ws_data["RosTeamWS_WS_FOLDER"]
            ws_path_folders = ws_folder.split(os.path.sep)[1:]
            ws_name = ws_path_folders[-1]
            new_ws_name_prefix = ""
            num_folders = len(ws_path_folders)

            if num_folders > 1:  # /folder1/my_ws
                # first letter of the folders
                new_ws_name_prefix += "".join([folder[0] for folder in ws_path_folders[:-2]])
                prev_folder = ws_path_folders[-2]
                if prev_folder != "workspace":
                    if new_ws_name_prefix:
                        new_ws_name_prefix += "_"
                    new_ws_name_prefix += prev_folder
                else:
                    new_ws_name_prefix += prev_folder[0]

            new_ws_name = "__".join([new_ws_name_prefix, ws_name])
            print(f"\t'{ws_folder}' -> {new_ws_name}")

            workspace_to_port = Workspace(**workspace_data_to_port)

            print(f"Updating workspace config in '{WORKSPACES_PATH}'")
            success = update_workspaces_config(WORKSPACES_PATH, new_ws_name, workspace_to_port)
            if success:
                print(f"Updated workspace config in '{WORKSPACES_PATH}'")
            else:
                print(f"Updating workspace config in '{WORKSPACES_PATH}' failed")


class PortVerb(VerbExtension):
    """Port the currently sourced ROS workspace by creating the corresponding config."""

    def main(self, *, args):
        workspace_data_to_port = {}
        print("Reading workspace environment variables: ")
        str_format = "\t{:<30} -> {:<20}: {}"
        for var in ROS_TEAM_WS_VARIABLES:
            value = os.environ.get(var, None)
            # check if variable is exported
            if value is None:
                print(f"Variable {var} is not exported. Cannot proceed with porting.")
                return
            new_var = var.replace("RosTeamWS_", "").lower()
            print(str_format.format(var, new_var, value))
            workspace_data_to_port[new_var] = value

        print("Generating workspace name from workspace path with first folder letters: ")
        ws_folder = os.environ.get("RosTeamWS_WS_FOLDER")
        ws_path_folders = ws_folder.split(os.path.sep)[1:]
        ws_name = ws_path_folders[-1]
        new_ws_name_prefix = ""
        num_folders = len(ws_path_folders)

        if num_folders > 1:  # /folder1/my_ws
            # first letter of the folders
            new_ws_name_prefix += "".join([folder[0] for folder in ws_path_folders[:-2]])
            prev_folder = ws_path_folders[-2]
            if prev_folder != "workspace":
                if new_ws_name_prefix:
                    new_ws_name_prefix += "_"
                new_ws_name_prefix += prev_folder
            else:
                new_ws_name_prefix += prev_folder[0]

        new_ws_name = "__".join([new_ws_name_prefix, ws_name])
        print(f"\t'{ws_folder}' -> {new_ws_name}")

        workspace_to_port = Workspace(**workspace_data_to_port)

        print(f"Updating workspace config in '{WORKSPACES_PATH}'")
        update_workspaces_config(WORKSPACES_PATH, new_ws_name, workspace_to_port)
