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

import argparse
import copy
import dataclasses
import datetime
import os
import pathlib
import re
import shutil
import subprocess
from typing import Any, Dict, List

import questionary
from rtwcli.command.info import ROS_TEAM_WS_VARIABLES
from rtwcli.helpers import (
    create_file_and_write,
    create_file_if_not_exists,
    load_yaml_file,
    write_to_yaml_file,
)
from rtwcli.verb import VerbExtension

WORKSPACES_PATH = os.path.expanduser("~/.ros_team_workspace/workspaces.yaml")
CURRENT_FILE_DIR = pathlib.Path(__file__).parent.absolute()
USE_WORKSPACE_SCRIPT_PATH = (
    CURRENT_FILE_DIR / ".." / ".." / ".." / ".." / "scripts" / "environment" / "setup.bash"
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


@dataclasses.dataclass
class Workspace:
    distro: str
    ws_folder: str
    ws_docker_support: bool = False
    docker_tag: str = None
    base_ws: str = None

    def __post_init__(self):
        self.distro = str(self.distro)
        self.ws_folder = str(self.ws_folder)
        self.ws_docker_support = bool(self.ws_docker_support)
        if self.docker_tag is not None:
            self.docker_tag = str(self.docker_tag)
        if self.base_ws is not None:
            self.base_ws = str(self.base_ws)


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
    current_date = datetime.datetime.now().strftime(BACKUP_DATETIME_FORMAT)
    backup_filename = WORKSPACES_PATH_BACKUP_FORMAT.format(current_date)
    create_file_if_not_exists(backup_filename)
    shutil.copy(config_path, backup_filename)
    print(f"Backed up current workspaces config file to '{backup_filename}'")

    if not save_workspaces_config(config_path, workspaces_config):
        print(f"Failed to update YAML file '{config_path}'.")
        return False

    print(f"Updated YAML file '{config_path}' with a new workspace '{ws_name}'")
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
    if type(ws_var_value) is bool:
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


def get_expected_ws_field_names() -> List[str]:
    return [field.name for field in dataclasses.fields(Workspace)]


def is_docker_tag_valid(tag: str) -> bool:
    """Validate a given Docker tag by trying to inspect it."""
    try:
        result = subprocess.run(
            ["docker", "image", "inspect", tag],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=True,
        )
        if result.returncode == 0:
            return True
    except subprocess.CalledProcessError:
        print(f"'{tag}' is not a valid Docker tag or image.")
        return False
    except Exception as e:
        print(f"Error while trying to validate Docker tag: {e}")
        return False


def try_port_workspace(workspace_data_to_port: Dict[str, Any], new_ws_name: str) -> bool:
    # set workspace missing fields to default values
    if F_WS_DOCKER_SUPPORT not in workspace_data_to_port:
        workspace_data_to_port[F_WS_DOCKER_SUPPORT] = False
    if F_DOCKER_TAG not in workspace_data_to_port:
        workspace_data_to_port[F_DOCKER_TAG] = None
    if F_BASE_WS not in workspace_data_to_port:
        workspace_data_to_port[F_BASE_WS] = None

    # validate workspace fields
    if not workspace_data_to_port[F_WS_DOCKER_SUPPORT]:
        workspace_data_to_port[F_DOCKER_TAG] = None
    if workspace_data_to_port[F_WS_DOCKER_SUPPORT]:
        if not is_docker_tag_valid(workspace_data_to_port[F_DOCKER_TAG]):
            return False

    # Identify and inform about unexpected fields
    expected_ws_field_names = get_expected_ws_field_names()
    current_ws_field_names = list(workspace_data_to_port.keys())
    unexpected_fields = set(current_ws_field_names) - set(expected_ws_field_names)
    if unexpected_fields:
        print(f"Current fields to port: {', '.join(current_ws_field_names)}")
        print(f"Found unexpected fields: {', '.join(unexpected_fields)}")
        print(f"Available fields are: {', '.join(expected_ws_field_names)}")
        print("These unexpected fields will be skipped.")
        for field in unexpected_fields:
            del workspace_data_to_port[field]

    # ask user for missing ws fields
    choices = [
        "Skip this workspace",
        "Enter the missing field (validation not implemented yet)",
        "Stop porting entirely",
    ]
    for expected_ws_field_name in expected_ws_field_names:
        if expected_ws_field_name in current_ws_field_names:
            continue
        choice = questionary.select(
            f"Missing field '{expected_ws_field_name}'. What would you like to do?",
            choices=choices,
        ).ask()
        if choice is None:  # Cancelled by user
            return False
        elif choice == choices[0]:
            return False
        if choice == choices[1]:
            value = questionary.text(f"Enter value for {expected_ws_field_name}:").ask()
            workspace_data_to_port[expected_ws_field_name] = value
        else:
            exit("Stopped porting due to a missing field.")

    workspace_to_port = Workspace(**workspace_data_to_port)
    print(f"Updating workspace config in '{WORKSPACES_PATH}'")
    success = update_workspaces_config(WORKSPACES_PATH, new_ws_name, workspace_to_port)
    if success:
        print(f"Updated workspace config in '{WORKSPACES_PATH}'")
        return True
    else:
        print(f"Updating workspace config in '{WORKSPACES_PATH}' failed")
        return False


class CreateVerb(VerbExtension):
    """Create a new ROS workspace."""

    def main(self, *, args):
        print("Not implemented yet")


def get_workspace_names() -> List[str]:
    """Retrieve a list of workspace names from the YAML file."""
    if not os.path.isfile(WORKSPACES_PATH):
        return []
    workspaces_config = load_workspaces_config_from_yaml_file(WORKSPACES_PATH)
    return workspaces_config.get_ws_names()


def workspace_name_completer(**kwargs) -> List[str]:
    """Callable returning a list of workspace names."""
    ws_names = get_workspace_names()
    if not ws_names:
        return ["NO_WORKSPACES_FOUND"]
    return ws_names


def add_cli_workspace_args(parser: argparse.ArgumentParser):
    arg = parser.add_argument(
        "workspace_name",
        help="The workspace name",
        nargs="?",
    )
    arg.completer = workspace_name_completer


class UseVerb(VerbExtension):
    """Select and source an existing ROS workspace."""

    def add_arguments(self, parser: argparse.ArgumentParser, cli_name: str):
        add_cli_workspace_args(parser)

    def main(self, *, args):
        workspaces_config = load_workspaces_config_from_yaml_file(WORKSPACES_PATH)
        if not workspaces_config.workspaces:
            print(f"No workspaces found in config file '{WORKSPACES_PATH}'")
            return

        ws_names = workspaces_config.get_ws_names()

        ws_name = args.workspace_name
        if not ws_name:
            ws_name = questionary.autocomplete(
                "Choose workspace",
                ws_names,
                qmark="'Tab' to see all workspaces, type to filter, 'Enter' to select\n",
                meta_information=copy.deepcopy(workspaces_config.to_dict()[WORKSPACES_KEY]),
                validate=lambda ws_choice: ws_choice in ws_names,
                style=questionary.Style([("answer", "bg:ansiwhite")]),
                match_middle=True,
            ).ask()
            if not ws_name:  # Cancelled by user
                return

        workspace = workspaces_config.workspaces.get(ws_name)
        if not workspace:
            return f"Workspace '{ws_name}' not found."

        print(f"Workspace data: {workspace}")

        script_content = create_bash_script_content_for_using_ws(
            workspace, USE_WORKSPACE_SCRIPT_PATH
        )
        tmp_file = f"/tmp/ros_team_workspace/wokspace_{os.getppid()}.bash"
        print(f"Following text will be written into file '{tmp_file}':\n{script_content}")
        if not create_file_and_write(tmp_file, content=script_content):
            return f"Failed to write workspace data to a file {tmp_file}."

        print(f"Using workspace '{ws_name}'")


class PortAllVerb(VerbExtension):
    """Port the currently sourced ROS workspace by creating the corresponding config."""

    def main(self, *, args):
        print(f"Reading workspaces from script '{ROS_TEAM_WS_RC_PATH}'")

        script_workspaces = extract_workspaces_from_bash_script(ROS_TEAM_WS_RC_PATH)
        ws_num = len(script_workspaces)
        print(f"Found {ws_num} workspaces in script '{ROS_TEAM_WS_RC_PATH}'")

        var_str_format = "\t{:>30} -> {:<20}: {}"
        sep_line = "-" * 50

        # For statistics:
        porting_stats = {
            "successful": [],
            "failed": [],
        }

        for i, (script_ws, script_ws_data) in enumerate(script_workspaces.items()):
            ws_name_to_print = f"{i+1}/{ws_num} script workspace '{script_ws}'"
            print(f"{sep_line}\n" f"Processing {ws_name_to_print}," f" ws_data: {script_ws_data}")
            workspace_data_to_port = {}
            for env_var, env_var_value in script_ws_data.items():
                ws_var, ws_var_value = env_var_to_workspace_var(env_var, env_var_value)
                print(var_str_format.format(env_var, ws_var, env_var_value))
                workspace_data_to_port[ws_var] = ws_var_value

            print("Generating workspace name from workspace path with first folder letters: ")
            ws_path = script_ws_data[WS_FOLDER_ENV_VAR]
            new_ws_name = generate_workspace_name(ws_path)
            print(f"\t'{ws_path}' -> {new_ws_name}")

            success = try_port_workspace(workspace_data_to_port, new_ws_name)
            if success:
                print(f"Ported workspace '{new_ws_name}' successfully")
                porting_stats["successful"].append(ws_name_to_print)
            else:
                print(f"Porting workspace '{new_ws_name}' failed")
                porting_stats["failed"].append(ws_name_to_print)

        # Print the final summary:
        summary_sep_line = "#" * 50
        print("\n" + summary_sep_line)
        print("Porting Summary:")
        print(f"Total Workspaces: {ws_num}")
        print(f"Successfully Ported: {len(porting_stats['successful'])}")
        for ws_name in porting_stats["successful"]:
            print(f" - {ws_name}")
        print(f"Failed to Port: {len(porting_stats['failed'])}")
        for ws_name in porting_stats["failed"]:
            print(f" - {ws_name}")
        print(summary_sep_line)


class PortVerb(VerbExtension):
    """Port the currently sourced ROS workspace by creating the corresponding config."""

    def main(self, *, args):
        workspace_data_to_port = {}
        print("Reading workspace environment variables: ")
        var_str_format = "\t{:>30} -> {:<20}: {}"
        for env_var in ROS_TEAM_WS_VARIABLES:
            env_var_value = os.environ.get(env_var, None)
            ws_var, ws_var_value = env_var_to_workspace_var(env_var, env_var_value)
            # check if variable is exported
            if ws_var in [F_DISTRO, F_WS_FOLDER] and ws_var_value is None:
                print(f"Variable {env_var} is not exported. Cannot proceed with porting.")
                return
            print(var_str_format.format(env_var, ws_var, env_var_value))
            workspace_data_to_port[ws_var] = ws_var_value

        print("Generating workspace name from workspace path with first folder letters: ")
        ws_path = os.environ.get(WS_FOLDER_ENV_VAR)
        new_ws_name = generate_workspace_name(ws_path)
        print(f"\t'{ws_path}' -> {new_ws_name}")

        success = try_port_workspace(workspace_data_to_port, new_ws_name)
        if success:
            print(f"Ported workspace '{new_ws_name}' successfully")
        else:
            print(f"Porting workspace '{new_ws_name}' failed")
