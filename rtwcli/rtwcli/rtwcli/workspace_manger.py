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

import dataclasses
import datetime
import os
import re
import shutil
from typing import Any, Dict, List
import questionary

from rtwcli.constants import (
    BACKUP_DATETIME_FORMAT,
    F_BASE_WS,
    F_DOCKER_CONTAINER_NAME,
    F_DOCKER_TAG,
    F_WS_DOCKER_SUPPORT,
    ROS_TEAM_WS_PREFIX,
    WORKSPACES_KEY,
    WORKSPACES_PATH,
    WORKSPACES_PATH_BACKUP_FORMAT,
    WS_FOLDER_ENV_VAR,
)
from rtwcli.docker_utils import is_docker_tag_valid
from rtwcli.utils import create_file_if_not_exists, load_yaml_file, write_to_yaml_file


@dataclasses.dataclass
class Workspace:
    """A dataclass representing a workspace."""

    distro: str
    ws_folder: str
    ws_docker_support: bool = False
    docker_tag: str = None
    docker_container_name: str = None
    base_ws: str = None

    def __post_init__(self):
        self.distro = str(self.distro)
        self.ws_folder = str(self.ws_folder)
        self.ws_docker_support = bool(self.ws_docker_support)
        if self.docker_tag is not None:
            self.docker_tag = str(self.docker_tag)
        if self.base_ws is not None:
            self.base_ws = str(self.base_ws)
        if self.docker_container_name is not None:
            self.docker_container_name = str(self.docker_container_name)


@dataclasses.dataclass
class WorkspacesConfig:
    """A dataclass representing a workspaces configuration."""

    workspaces: Dict[str, Workspace] = dataclasses.field(default_factory=dict)

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

    def add_workspace(self, ws_name: str, workspace: Workspace) -> bool:
        if ws_name in self.workspaces:
            print(f"Workspace '{ws_name}' already exists in the config.")
            return False
        self.workspaces[ws_name] = workspace
        return True


def load_workspaces_config_from_yaml_file(file_path: str):
    """Load a WorkspacesConfig from a YAML file."""
    return WorkspacesConfig.from_dict(load_yaml_file(file_path))


def save_workspaces_config(filepath: str, config: WorkspacesConfig):
    """Save a WorkspacesConfig to a YAML file."""
    return write_to_yaml_file(filepath, config.to_dict())


def update_workspaces_config(config_path: str, ws_name: str, workspace: Workspace) -> bool:
    """Update the workspaces config with a new workspace."""
    if not create_file_if_not_exists(config_path):
        print("Could not create workspaces config file. Cannot proceed with porting.")
        return False

    workspaces_config = load_workspaces_config_from_yaml_file(config_path)
    if not workspaces_config.add_workspace(ws_name, workspace):
        print(f"Failed to add workspace '{ws_name}' to the config.")
        return False

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


def get_current_workspace() -> Workspace:
    """Retrieve the current workspace from the workspaces config."""
    ws_name = get_current_workspace_name()
    if not ws_name:
        return None

    workspaces_config = load_workspaces_config_from_yaml_file(WORKSPACES_PATH)
    if not workspaces_config:
        return None

    return workspaces_config.workspaces.get(ws_name, None)


def get_current_workspace_name() -> str:
    """Retrieve the current workspace name from the environment variable."""
    ros_ws_folder = os.environ.get(WS_FOLDER_ENV_VAR, None)
    if not ros_ws_folder:
        print(f"Environment variable '{WS_FOLDER_ENV_VAR}' not set.")
        return None

    return os.path.basename(ros_ws_folder)


def extract_workspaces_from_bash_script(script_path: str) -> Dict[str, dict]:
    """Extract workspaces from a bash script."""
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


def env_var_to_workspace_var(env_var: str, env_var_value: str) -> str:
    """Convert an environment variable to a workspace variable."""
    ws_var = env_var.replace(ROS_TEAM_WS_PREFIX, "").lower()
    if env_var_value == "false":
        ws_var_value = False
    elif env_var_value == "true":
        ws_var_value = True
    else:
        ws_var_value = env_var_value
    return ws_var, ws_var_value


def workspace_var_to_env_var(ws_var: str, ws_var_value: Any) -> str:
    """Convert a workspace variable to an environment variable."""
    env_var = ROS_TEAM_WS_PREFIX + ws_var.upper()
    if type(ws_var_value) == bool:
        env_var_value = str(ws_var_value).lower()
    else:
        env_var_value = ws_var_value
    return env_var, env_var_value


def create_bash_script_content_for_using_ws(
    workspace: Workspace, use_workspace_script_path: str
) -> str:
    """Create a bash script content for using a workspace."""
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
    """Return a list of expected workspace field names."""
    return [field.name for field in dataclasses.fields(Workspace)]


def try_port_workspace(workspace_data_to_port: Dict[str, Any], new_ws_name: str) -> bool:
    """Try to port a workspace."""
    # set workspace missing fields to default values
    if F_WS_DOCKER_SUPPORT not in workspace_data_to_port:
        workspace_data_to_port[F_WS_DOCKER_SUPPORT] = False
    if F_DOCKER_TAG not in workspace_data_to_port:
        workspace_data_to_port[F_DOCKER_TAG] = None
    if F_BASE_WS not in workspace_data_to_port:
        workspace_data_to_port[F_BASE_WS] = None
    if F_DOCKER_CONTAINER_NAME not in workspace_data_to_port:
        workspace_data_to_port[F_DOCKER_CONTAINER_NAME] = None

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


def get_compile_cmd(
    ws_path_abs: str,
    distro: str,
    setup_bash_path: str = None,
    distro_setup_bash_format: str = "/opt/ros/{distro}/setup.bash",
) -> List[str]:
    """Return a compile command for the given workspace."""
    distro_setup_bash_path = distro_setup_bash_format.format(distro=distro)
    compile_ws_cmd = [
        "cd",
        ws_path_abs,
        "&&",
        "source",
        distro_setup_bash_path if not setup_bash_path else setup_bash_path,
        "&&",
        "colcon",
        "build",
        "--symlink-install",
        "--cmake-args",
        "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
    ]
    return compile_ws_cmd


def get_workspace_names() -> List[str]:
    """Return a list of workspace names."""
    if not os.path.isfile(WORKSPACES_PATH):
        return []
    workspaces_config = load_workspaces_config_from_yaml_file(WORKSPACES_PATH)
    return workspaces_config.get_ws_names()


def workspace_name_completer(**kwargs) -> List[str]:
    """Return a list of workspace names for autocompletion."""
    ws_names = get_workspace_names()
    if not ws_names:
        return ["NO_WORKSPACES_FOUND"]
    return ws_names
