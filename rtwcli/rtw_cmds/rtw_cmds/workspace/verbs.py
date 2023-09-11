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
import datetime
import os
import pathlib
import re
import shutil
import subprocess
from typing import Any, Dict, List, Tuple

import questionary
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
DELETE_WS_FORMAT = "{:<40} {:<10} {:<60} {}"

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

    def get_ws(self, ws_name: str) -> Workspace:
        return copy.deepcopy(self.workspaces[ws_name])

    def get_workspaces(self) -> Dict[str, Workspace]:
        return copy.deepcopy(self.workspaces)

    def add_ws(self, ws_name: str, ws: Workspace) -> None:
        self.workspaces[ws_name] = ws

    def delete_ws(self, ws_name) -> bool:
        if ws_name not in self.workspaces:
            return True
        self.workspaces.pop(ws_name)
        return True


@dataclasses.dataclass
class DeletionDockerStats:
    ws_name: str
    docker_image_tag: str
    failed_to_get_container_ids: str = ""
    container_ids: List[str] = dataclasses.field(default_factory=list)
    stopped_container_ids: List[str] = dataclasses.field(default_factory=list)
    failed_to_stop_container_ids: Dict[str, str] = dataclasses.field(default_factory=dict)
    removed_container_ids: List[str] = dataclasses.field(default_factory=list)
    failed_to_remove_containers: Dict[str, str] = dataclasses.field(default_factory=dict)
    docker_image_removed: bool = False
    failed_to_remove_image: str = ""


@dataclasses.dataclass
class DeletionSend2TrashStats:
    ws_name: str
    ws_folder: str
    top_level_items_to_remove: List[str] = dataclasses.field(default_factory=list)
    removed_top_level_items: List[str] = dataclasses.field(default_factory=list)
    failed_to_remove_top_level_items: Dict[str, str] = dataclasses.field(default_factory=dict)


@dataclasses.dataclass
class DeletionWSConfigStats:
    ws_name: str
    did_not_need_to_remove: bool = False
    backed_up_config: bool = False
    failed_to_backup_config: bool = False
    deleted_from_config: bool = False
    failed_to_delete_from_config: bool = False
    saved_new_config: bool = False
    failed_to_save_new_config: bool = False


@dataclasses.dataclass
class DeletionStats:
    ws_names_to_delete: List[str]
    docker_stats: Dict[str, DeletionDockerStats] = dataclasses.field(default_factory=dict)
    send2trash_stats: Dict[str, DeletionSend2TrashStats] = dataclasses.field(default_factory=dict)
    cancelled_by_user: Tuple[str, str] = None
    deleted_from_config_stats: Dict[str, DeletionWSConfigStats] = dataclasses.field(
        default_factory=dict
    )
    fully_succeeded: List[str] = dataclasses.field(default_factory=list)

    def print_summary(self, fst_level="└── ", snd_level="    │   └── ", cancel_level=">>> "):
        num_all_ws = len(self.ws_names_to_delete)
        print(f"\n------ Summary of Deletion ({num_all_ws} ws) ------")

        # Print Succeeded Workspaces:
        print(f"\n-- Succeeded ({len(self.fully_succeeded)}/{num_all_ws} ws) --")
        for ws_name in self.fully_succeeded:
            print(f"{fst_level}{ws_name}")

        # Print Failed Workspaces:
        num_failed = len(self.ws_names_to_delete) - len(self.fully_succeeded)
        deletion_order = "[Docker Stats -> Send2Trash Stats -> Config Deletion Stats]"
        print(f"\n-- Failed ({num_failed}/{num_all_ws} ws) {deletion_order} --")
        for ws_name in self.ws_names_to_delete:
            if ws_name in self.fully_succeeded:
                continue

            print(f"{fst_level}{ws_name}")

            docker_stat = self.docker_stats.get(ws_name, None)
            if docker_stat:
                docker_stats_title = "    ├── Docker Stats:"
                if docker_stat.docker_image_removed:
                    docker_stats_title += " OK"
                print(docker_stats_title)
                if docker_stat.failed_to_get_container_ids:
                    print(
                        f"{snd_level}Failed to get Docker containers for image "
                        f"{docker_stat.docker_image_tag}. "
                        f"Error: {docker_stat.failed_to_get_container_ids}"
                    )
                for container_id, error in docker_stat.failed_to_stop_container_ids.items():
                    print(
                        f"{snd_level}Failed to stop container {container_id}. " f"Error: {error}"
                    )
                for container_id, error in docker_stat.failed_to_remove_containers.items():
                    print(
                        f"{snd_level}Failed to remove container {container_id}. " f"Error: {error}"
                    )
                if not docker_stat.docker_image_removed:
                    print(
                        f"{snd_level}Failed to remove Docker image "
                        f"{docker_stat.docker_image_tag}. "
                        f"Error: {docker_stat.failed_to_remove_image}"
                    )

            send2trash_stat = self.send2trash_stats.get(ws_name, None)
            if send2trash_stat:
                send2trash_title = "    ├── Send2Trash Stats:"
                if not send2trash_stat.failed_to_remove_top_level_items:
                    send2trash_title += " OK"
                print(send2trash_title)
                for item, error in send2trash_stat.failed_to_remove_top_level_items.items():
                    print(f"{snd_level}Failed to send item {item} to trash. Error: {error}")

            config_stat = self.deleted_from_config_stats.get(ws_name, None)
            if config_stat:
                config_deletion_title = "    ├── Config Deletion Stats:"
                if config_stat.deleted_from_config:
                    config_deletion_title += " OK"
                print(config_deletion_title)
                if config_stat.failed_to_backup_config:
                    print(f"{snd_level}Failed to backup config.")
                if config_stat.failed_to_delete_from_config:
                    print(f"{snd_level}Failed to delete workspace from config.")
                if config_stat.failed_to_save_new_config:
                    print(f"{snd_level}Failed to save new config.")

        if self.cancelled_by_user:
            ws_name, reason = self.cancelled_by_user
            print(
                f"\n{cancel_level}Operation for workspace '{ws_name}' was cancelled by the user. "
                f"\n{cancel_level}Reason: {reason}\n"
            )


def load_workspaces_config_from_yaml_file(file_path: str):
    return WorkspacesConfig.from_dict(load_yaml_file(file_path))


def save_workspaces_config(filepath: str, config: WorkspacesConfig):
    return write_to_yaml_file(filepath, config.to_dict())


def backup_workspaces_config(config_path: str) -> bool:
    current_date = datetime.datetime.now().strftime(BACKUP_DATETIME_FORMAT)
    backup_filename = WORKSPACES_PATH_BACKUP_FORMAT.format(current_date)
    if not create_file_if_not_exists(backup_filename):
        return False
    try:
        shutil.copy(config_path, backup_filename)
    except Exception as e:
        print(f"Exception {type(e)} caught while creating backup file for config '{config_path}' ")
        return False
    return True


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

    workspaces_config.add_ws(ws_name, workspace)

    # Backup current config file
    backup_success = backup_workspaces_config(config_path)
    if backup_success:
        print(f"Backed up current workspaces config '{config_path}' successfully")
    else:
        print(f"Backing up current workspaces config '{config_path}' failed")

    if not save_workspaces_config(config_path, workspaces_config):
        print(f"Failed to update YAML file '{config_path}'.")
        return False

    print(f"Updated YAML file '{config_path}' with a new workspace '{ws_name}'")
    return True


def delete_ws_in_workspaces_config(config_path: str, ws_name: str) -> bool:
    if not os.path.exists(config_path):
        print(f"No workspaces config file found in '{config_path}'. Cannot proceed with deleting.")
        return False

    workspaces_config = load_workspaces_config_from_yaml_file(config_path)

    if ws_name not in workspaces_config.get_ws_names():
        print(f"Workspace name '{ws_name}' was not found in the config. Nothing to delete.")
        return True

    # Backup current config file
    backup_success = backup_workspaces_config(config_path)
    if backup_success:
        print(f"Backed up current workspaces config '{config_path}' successfully")
    else:
        print(f"Backing up current workspaces config '{config_path}' failed")

    workspaces_config.delete_ws(ws_name)

    if not save_workspaces_config(config_path, workspaces_config):
        print(f"Failed to update YAML file '{config_path}'.")
        return False

    print(f"Updated YAML file '{config_path}' while deleting workspace '{ws_name}'")
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
        if not workspaces_config.get_ws_names():
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
        if not ws_name:  # Cancelled by user
            return

        workspace = workspaces_config.get_ws(ws_name)
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
        if not workspaces_config.get_ws_names():
            print(f"No workspaces found in config file '{WORKSPACES_PATH}'")
            return

        choice_data = {}
        for ws_name, ws in workspaces_config.get_workspaces().items():
            ws_choice_data = DELETE_WS_FORMAT.format(
                ws_name, ws.distro, ws.ws_folder, ws.docker_tag
            )
            choice_data[ws_choice_data] = ws_name

        choices_selection_data = "[workspace name, distro, path, docker tag]"
        ws_choices_to_delete = questionary.checkbox(
            f"Select workspaces to delete {choices_selection_data}\n",
            choices=list(choice_data.keys()),
        ).ask()
        if not ws_choices_to_delete:
            return

        ws_names_to_delete_to_confirm = ""
        for ws_name_to_delete in ws_choices_to_delete:
            ws_names_to_delete_to_confirm += "\n" + ws_name_to_delete
        ws_names_to_delete_to_confirm += "\n"

        confirm_delete = questionary.confirm(
            f"Are you sure you want to delete following workspaces? {choices_selection_data}"
            f"{ws_names_to_delete_to_confirm}"
        ).ask()
        if not confirm_delete:
            return

        ws_names_to_delete = [
            choice_data[ws_choice_to_delete] for ws_choice_to_delete in ws_choices_to_delete
        ]
        stats = self.handle_delete_workspaces(ws_names_to_delete, workspaces_config)
        print("Deleting workspaces done.")
        stats.print_summary()

    def handle_docker_operations(self, ws_name: str, docker_tag: str) -> DeletionDockerStats:
        if docker_tag is None:
            return None

        stats = DeletionDockerStats(ws_name=ws_name, docker_image_tag=docker_tag)

        print(f"Getting container IDs for the given Docker image {docker_tag}")
        container_ids_cmd = ["docker", "ps", "-a", "-q", "--filter", f"ancestor={docker_tag}"]
        try:
            container_ids = subprocess.check_output(container_ids_cmd).decode("utf-8").splitlines()
            stats.container_ids.clear()
            stats.container_ids.extend(container_ids)
        except Exception as e:
            error_msg = (
                f"Exception {type(e)} caught during Docker cmd to get container IDs "
                f"{container_ids_cmd} for '{docker_tag}': {e}"
            )
            print(error_msg)
            stats.failed_to_get_container_ids = error_msg
            return stats

        # stop docker containers
        for container_id in container_ids:
            print(f"Stopping container {container_id}")
            docker_stop_cmd = ["docker", "stop", container_id]
            try:
                subprocess.check_output(docker_stop_cmd)
                stats.stopped_container_ids.append(container_id)
            except Exception as e:
                error_msg = (
                    f"Exception {type(e)} caught during Docker stop cmd {docker_stop_cmd} "
                    f"for '{docker_tag}': {e}"
                )
                print(error_msg)
                stats.failed_to_stop_container_ids[container_id] = error_msg

        # abort if there are any unstopped containers available
        if len(stats.failed_to_stop_container_ids) != 0:
            return stats

        # remove docker containers
        for container_id in container_ids:
            print(f"Removing container {container_id}")
            docker_rm_cmd = ["docker", "rm", container_id]
            try:
                subprocess.check_output(docker_rm_cmd)
                stats.stopped_container_ids.append(container_id)
            except Exception as e:
                error_msg = (
                    f"Exception {type(e)} caught during Docker rm cmd {docker_rm_cmd} for "
                    f"'{docker_tag}': {e}"
                )
                print(error_msg)
                stats.failed_to_remove_containers[container_id] = error_msg

        # abort if there are any containers left
        if len(stats.failed_to_remove_containers) != 0:
            return stats

        print(f"Removing workspace docker image '{docker_tag}'")
        docker_rmi_cmd = ["docker", "rmi", docker_tag]
        try:
            subprocess.check_output(docker_rmi_cmd)
            stats.docker_image_removed = True
        except Exception as e:
            stats.docker_image_removed = False
            error_msg = (
                f"Exception {type(e)} caught during Docker rmi cmd {docker_rmi_cmd} for "
                f"'{docker_tag}': {e}"
            )
            print(error_msg)
            stats.failed_to_remove_image = error_msg

        return stats

    def handle_send_to_trash(
        self, ws_name: str, ws_folder: str, src_folder="src"
    ) -> DeletionSend2TrashStats:
        stats = DeletionSend2TrashStats(ws_name=ws_name, ws_folder=ws_folder)

        # check if the src folder should be removed
        send_src_folder_to_trash = True
        items_in_src_folder = None
        src_path = os.path.join(ws_folder, src_folder)
        if os.path.exists(src_path):
            items_in_src_folder = os.listdir(src_path)
            if items_in_src_folder:
                send_src_folder_to_trash = questionary.confirm(
                    f"Found non-empty '{src_folder}' folder in '{src_path}' for ws '{ws_name}'. "
                    f"\nDo you want to delete the '{src_folder}' folder that has following content?"
                    f"\n{items_in_src_folder}\n"
                ).ask()
                if send_src_folder_to_trash is None:  # cancelled by user
                    return None
                print(f"send_src_folder_to_trash: {send_src_folder_to_trash}")

        top_level_items_to_remove = []

        # remove the src folder
        if send_src_folder_to_trash:
            print(f"Sending to trash the whole workspace folder '{ws_folder}' of ws '{ws_name}'")
            top_level_items_to_remove = [ws_folder]
        # keep the src folder
        else:
            items_in_ws_folder = os.listdir(ws_folder)
            if not items_in_ws_folder:
                print(f"Nothing to remove in ws folder '{ws_folder}'")
                return stats
            item_paths_in_ws_folder = [
                os.path.join(ws_folder, item) for item in items_in_ws_folder if item != src_folder
            ]
            print(
                f"Sending to trash everything in ws folder '{ws_folder}' of ws '{ws_name}' "
                f"except '{src_folder}' folder: {item_paths_in_ws_folder}"
            )
            top_level_items_to_remove = item_paths_in_ws_folder

        stats.top_level_items_to_remove = top_level_items_to_remove

        for item_path in top_level_items_to_remove:
            try:
                send2trash(item_path)
                stats.removed_top_level_items.append(item_path)
                print(f"Sent to trash: '{item_path}'")
            except Exception as e:
                error_msg = f"Exception {type(e)} caught while sending '{item_path}' to trash: {e}"
                print(error_msg)
                stats.failed_to_remove_top_level_items[item_path] = error_msg

        return stats

    def handle_delete_from_config(
        self, ws_name: str, ws_config: WorkspacesConfig, config_path: str
    ) -> DeletionWSConfigStats:
        stats = DeletionWSConfigStats(ws_name=ws_name)
        if ws_name not in ws_config.get_ws_names():
            stats.did_not_need_to_remove = True
            return stats

        # Backup current config file
        backup_success = backup_workspaces_config(config_path)
        if backup_success:
            print(f"Backed up current workspaces config '{config_path}' successfully")
        else:
            error_msg = f"Backing up current workspaces config '{config_path}' failed"
            print(error_msg)
            stats.failed_to_backup_config = True
            return stats

        config_delete_success = ws_config.delete_ws(ws_name)
        if config_delete_success:
            print(f"Deleted ws '{ws_name}' from workspaces config successfully")
        else:
            error_msg = f"Failed to delete ws '{ws_name}' from workspaces config"
            print(error_msg)
            stats.failed_to_delete_from_config = True
            return stats

        save_config_success = save_workspaces_config(config_path, ws_config)
        if save_config_success:
            print(
                f"Saved new workspaces config without ws '{ws_name}' into '{config_path}'"
                "successfully"
            )
        else:
            error_msg = (
                f"Failed to save new workspaces config without ws '{ws_name}' into '{config_path}'"
            )
            print(error_msg)
            stats.failed_to_save_new_config = True
            return stats

        stats.deleted_from_config = True
        return stats

    def handle_delete_workspaces(
        self, ws_names_to_delete: List[str], workspaces_config: WorkspacesConfig
    ) -> DeletionStats:
        stats = DeletionStats(ws_names_to_delete=ws_names_to_delete)
        num_to_delete = len(ws_names_to_delete)
        line_sep = "-" * 50
        for ws_name_i, ws_name in enumerate(ws_names_to_delete):
            print(f"{line_sep}\nDeleting workspace {ws_name_i+1}/{num_to_delete} '{ws_name}'...")
            ws = workspaces_config.get_ws(ws_name)

            if ws.ws_docker_support:
                docker_stats = self.handle_docker_operations(ws_name, ws.docker_tag)
                stats.docker_stats[ws_name] = docker_stats
                if docker_stats.failed_to_remove_image:
                    print(
                        f"Aborting delete operation for ws '{ws_name}' as removing docker "
                        "image failed"
                    )
                    continue

            send2trash_stats = self.handle_send_to_trash(ws_name=ws_name, ws_folder=ws.ws_folder)
            if send2trash_stats is None:  # cancelled by user
                stats.cancelled_by_user = ws_name, (
                    f"Send2trash deletion operation of ws '{ws_name}' was interrupted"
                )
                return stats

            stats.send2trash_stats[ws_name] = send2trash_stats

            if send2trash_stats.failed_to_remove_top_level_items:
                confirm_delete_from_config = questionary.confirm(
                    f"Failed to send to trash {send2trash_stats.failed_to_remove_top_level_items}."
                    f" Do you still want to delete this workspace ({ws_name}) from the config "
                    f"file? ({WORKSPACES_PATH})"
                ).ask()
                if confirm_delete_from_config is None:  # cancelled by user
                    stats.cancelled_by_user = ws_name, (
                        f"Confirmation to delete ws '{ws_name}' from config after failed "
                        "send2trash deletion operation was interrupted"
                    )
                    return stats
                if not confirm_delete_from_config:
                    continue

            delete_from_config_stats = self.handle_delete_from_config(
                ws_name=ws_name, ws_config=workspaces_config, config_path=WORKSPACES_PATH
            )
            if delete_from_config_stats is not None:
                stats.deleted_from_config_stats[ws_name] = delete_from_config_stats

            # check if the workspace deletion succeeded entirely
            if ws_name in stats.docker_stats:
                if not stats.docker_stats[ws_name].docker_image_removed:
                    continue
            if ws_name not in stats.send2trash_stats:
                continue
            if stats.send2trash_stats[ws_name].failed_to_remove_top_level_items:
                continue
            if ws_name not in stats.deleted_from_config_stats:
                continue
            if not stats.deleted_from_config_stats[ws_name].deleted_from_config:
                continue
            stats.fully_succeeded.append(ws_name)

        return stats
