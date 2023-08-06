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
from typing import Any, Dict, List
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
WS_FOLDER_ENV_VAR = "RosTeamWS_WS_FOLDER"
ROS_TEAM_WS_PREFIX = "RosTeamWS_"
DELETE_WS_FORMAT = "{:<40} {:<10} {:<60} {}"


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


def generate_workspace_name(ws_path: str, excl_prev_folders=["workspace"]) -> str:
    """Generate workspace name based on the folder letters."""
    ws_path_folders = ws_path.split(os.path.sep)[1:]
    ws_name = ws_path_folders[-1]
    new_ws_name_prefix = ""
    num_folders = len(ws_path_folders)

    if num_folders > 1:  # /folder1/my_ws
        # First letters of the folders, except for the last two
        new_ws_name_prefix += "".join([folder[0] for folder in ws_path_folders[:-2]])
        prev_folder = ws_path_folders[-2]
        if prev_folder not in excl_prev_folders:
            if new_ws_name_prefix:
                new_ws_name_prefix += "_"
            new_ws_name_prefix += prev_folder
        else:
            new_ws_name_prefix += prev_folder[0]

    new_ws_name = "__".join([new_ws_name_prefix, ws_name])
    return new_ws_name


def env_var_to_workspace_var(env_var: str, env_var_value: str) -> str:
    ws_var = env_var.replace(ROS_TEAM_WS_PREFIX, "").lower()
    if env_var_value == "false":
        ws_var_value = False
    elif env_var_value == "true":
        ws_var_value = True
    else:
        ws_var_value = env_var_value
    return ws_var, ws_var_value


def workspace_var_to_env_var(ws_var: str, ws_var_value: Any) -> str:
    env_var = ROS_TEAM_WS_PREFIX + ws_var.upper()
    if type(ws_var_value) == bool:
        env_var_value = str(ws_var_value).lower()
    else:
        env_var_value = ws_var_value
    return env_var, env_var_value


def create_bash_script_content_for_using_ws(
    workspace: Workspace, use_workspace_script_path: str
) -> str:
    """Create a bash script content string using the workspace information."""
    bash_script_content = "#!/bin/bash\n"

    ws_data = dataclasses.asdict(workspace)
    for ws_var, ws_var_value in ws_data.items():
        env_var, env_var_value = workspace_var_to_env_var(ws_var, ws_var_value)
        bash_script_content += f"export {env_var}='{env_var_value}'\n"

    bash_script_content += (
        f"source {use_workspace_script_path} {workspace.distro} {workspace.ws_folder}\n"
    )

    return bash_script_content


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

        script_content = create_bash_script_content_for_using_ws(
            workspace, USE_WORKSPACE_SCRIPT_PATH
        )
        tmp_file = f"/tmp/ros_team_workspace/wokspace_{os.getppid()}.bash"
        print(f"Following text will be written into file '{tmp_file}':\n{script_content}")
        if not create_file_and_write(tmp_file, content=script_content):
            print(f"Failed to write workspace data to a file {tmp_file}.")
            return


class PortAllVerb(VerbExtension):
    """Port the currently sourced ROS workspace by creating the corresponding config."""

    def main(self, *, args):
        print(f"Reading workspaces from script '{ROS_TEAM_WS_RC_PATH}'")
        script_workspaces = extract_workspaces_from_bash_script(ROS_TEAM_WS_RC_PATH)
        ws_num = len(script_workspaces)
        print(f"Found {ws_num} workspaces in script '{ROS_TEAM_WS_RC_PATH}'")
        var_str_format = "\t{:>30} -> {:<20}: {}"
        for i, (script_ws, script_ws_data) in enumerate(script_workspaces.items()):
            print(
                f"Processing {i+1}/{ws_num} script workspace '{script_ws}',"
                f" ws_data: {script_ws_data}"
            )
            workspace_data_to_port = {}
            for env_var, env_var_value in script_ws_data.items():
                ws_var, ws_var_value = env_var_to_workspace_var(env_var, env_var_value)
                print(var_str_format.format(env_var, env_var_value, ws_var, ws_var_value))
                workspace_data_to_port[ws_var] = ws_var_value

            print("Generating workspace name from workspace path with first folder letters: ")
            ws_path = script_ws_data[WS_FOLDER_ENV_VAR]
            new_ws_name = generate_workspace_name(ws_path)
            print(f"\t'{ws_path}' -> {new_ws_name}")

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
        var_str_format = "\t{:>30} -> {:<20}: {}"
        for env_var in ROS_TEAM_WS_VARIABLES:
            env_var_value = os.environ.get(env_var, None)
            # check if variable is exported
            if env_var_value is None:
                print(f"Variable {env_var} is not exported. Cannot proceed with porting.")
                return
            ws_var, ws_var_value = env_var_to_workspace_var(env_var, env_var_value)
            print(var_str_format.format(env_var, ws_var, env_var_value))
            workspace_data_to_port[ws_var] = ws_var_value

        print("Generating workspace name from workspace path with first folder letters: ")
        ws_path = os.environ.get(WS_FOLDER_ENV_VAR)
        new_ws_name = generate_workspace_name(ws_path)
        print(f"\t'{ws_path}' -> {new_ws_name}")

        workspace_to_port = Workspace(**workspace_data_to_port)

        print(f"Updating workspace config in '{WORKSPACES_PATH}'")
        success = update_workspaces_config(WORKSPACES_PATH, new_ws_name, workspace_to_port)
        if success:
            print(f"Updated workspace config in '{WORKSPACES_PATH}'")
        else:
            print(f"Updating workspace config in '{WORKSPACES_PATH}' failed")


class DeleteVerb(VerbExtension):
    """Delete an available ROS workspace in the config."""

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

        choice_data = []
        for ws_name, ws in workspaces_config.workspaces.items():
            choice_data.append(
                DELETE_WS_FORMAT.format(ws_name, ws.distro, ws.ws_folder, ws.docker_tag)
            )

        ws_names_to_delete = questionary.checkbox(
            "Select workspaces to delete",
            choices=choice_data,
        ).ask()
        if not ws_names_to_delete:
            return

        ws_names_to_delete_to_confirm = ""
        for ws_name_to_delete in ws_names_to_delete:
            ws_names_to_delete_to_confirm += "\n" + ws_name_to_delete
        ws_names_to_delete_to_confirm += "\n"

        confirm_delete = questionary.confirm(
            f"Are you sure you want to delete following workspaces? {ws_names_to_delete_to_confirm}"
        ).ask()
        if not confirm_delete:
            return

        for ws_name_i, ws_name in enumerate(ws_names_to_delete):
            print(
                f"[Placeholder] Deleting workspace {ws_name_i+1}/{len(ws_names_to_delete)} '{ws_name}'..."
            )

        print("[Placeholder] Deleted all workspaces")
