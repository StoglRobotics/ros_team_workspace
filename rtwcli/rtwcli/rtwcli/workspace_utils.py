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
import shutil
from typing import Any, Dict, List, Optional, Tuple, Union
import questionary

from rtwcli.constants import (
    BACKUP_DATETIME_FORMAT,
    F_WS_NAME,
    ROS_TEAM_WS_PREFIX,
    WORKSPACES_KEY,
    WORKSPACES_PATH,
    WORKSPACES_PATH_BACKUP_FORMAT,
    ROS_TEAM_WS_WS_NAME_ENV_VAR,
)
from rtwcli.utils import create_file_if_not_exists, load_yaml_file, write_to_yaml_file
from rtwcli import logger


@dataclasses.dataclass
class Workspace:
    """A dataclass representing a workspace."""

    ws_name: str
    distro: str
    ws_folder: Optional[str] = None
    ws_docker_support: bool = False
    docker_tag: Optional[str] = None
    docker_container_name: Optional[str] = None
    base_ws: Optional[str] = None
    standalone: bool = False

    def __post_init__(self):
        if self.ws_folder == "":
            self.ws_folder = None
        if self.docker_tag == "":
            self.docker_tag = None
        if self.docker_container_name == "":
            self.docker_container_name = None
        if self.base_ws == "":
            self.base_ws = None
        if not self.ws_name:
            raise ValueError("Workspace name cannot be empty.")
        if not self.distro:
            raise ValueError("Distro cannot be empty.")
        if self.ws_folder and not os.path.isabs(self.ws_folder):
            raise ValueError("Workspace folder must be an absolute path.")
        if self.standalone and not self.ws_folder:
            raise ValueError("Standalone workspace requires a workspace folder.")
        if self.ws_docker_support and not self.docker_tag:
            raise ValueError("Docker-supported workspace requires a docker tag.")
        if self.ws_docker_support and not self.docker_container_name:
            raise ValueError("Docker-supported workspace requires a docker container name.")

    def to_dict(self) -> Dict[str, Any]:
        # result = dataclasses.asdict(self)
        # for key, value in result.items():
        #     if value == "":
        #         result[key] = None
        # return result
        return dataclasses.asdict(self)


@dataclasses.dataclass
class WorkspacesConfig:
    """A dataclass representing a workspaces configuration."""

    workspaces: Dict[str, Workspace] = dataclasses.field(default_factory=dict)

    @property
    def ws_meta_information(self) -> Dict[str, Any]:
        return self.to_dict()[WORKSPACES_KEY]

    @classmethod
    def from_dict(cls, data: Dict[str, Dict[str, dict]]) -> "WorkspacesConfig":
        if not data:
            return cls({})

        workspaces = {}
        for ws_name, ws_data in data.get(WORKSPACES_KEY, {}).items():
            if F_WS_NAME not in ws_data:
                ws_data[F_WS_NAME] = ws_name
            workspaces[ws_name] = Workspace(**ws_data)
        return cls(workspaces)

    def to_dict(self) -> Dict[str, Dict[str, dict]]:
        return {WORKSPACES_KEY: {ws_name: ws.to_dict() for ws_name, ws in self.workspaces.items()}}

    def get_ws_names(self) -> List[str]:
        if not self.workspaces:
            return []
        return list(self.workspaces.keys())

    def add_workspace(self, workspace: Workspace) -> bool:
        if workspace.ws_name in self.workspaces:
            logger.error(f"Workspace '{workspace.ws_name}' already exists in the config.")
            return False
        self.workspaces[workspace.ws_name] = workspace
        return True

    def remove_workspace(self, workspace_name: str) -> bool:
        if workspace_name not in self.workspaces:
            logger.warning(f"Workspace '{workspace_name}' does not exist in the config.")
            return False
        del self.workspaces[workspace_name]
        return True


def load_workspaces_config_from_yaml_file(file_path: str) -> WorkspacesConfig:
    """Load a WorkspacesConfig from a YAML file."""
    return WorkspacesConfig.from_dict(load_yaml_file(file_path))


def save_workspaces_config(filepath: str, config: WorkspacesConfig) -> bool:
    """Save a WorkspacesConfig to a YAML file."""
    return write_to_yaml_file(filepath, config.to_dict())


def update_workspaces_config(config_path: str, workspace: Workspace) -> bool:
    """Update the workspaces config with a new workspace."""
    if not create_file_if_not_exists(config_path):
        logger.error("Could not create workspaces config file.")
        return False

    workspaces_config = load_workspaces_config_from_yaml_file(config_path)
    if not workspaces_config.add_workspace(workspace):
        logger.error(f"Failed to add workspace '{workspace.ws_name}' to the config.")
        return False

    # Backup current config file
    current_date = datetime.datetime.now().strftime(BACKUP_DATETIME_FORMAT)
    backup_filename = WORKSPACES_PATH_BACKUP_FORMAT.format(current_date)
    create_file_if_not_exists(backup_filename)
    shutil.copy(config_path, backup_filename)
    logger.info(f"Backed up current workspaces config file to '{backup_filename}'")

    if not save_workspaces_config(config_path, workspaces_config):
        logger.error(f"Failed to update YAML file '{config_path}'.")
        return False

    logger.info(f"Updated YAML file '{config_path}' with a new workspace '{workspace.ws_name}'")
    return True


def get_current_workspace() -> Union[Workspace, None]:
    """Retrieve the current workspace from the workspaces config."""
    ws_name = get_current_workspace_name()
    if not ws_name:
        return None

    workspaces_config = load_workspaces_config_from_yaml_file(WORKSPACES_PATH)
    if not workspaces_config:
        return None

    return workspaces_config.workspaces.get(ws_name, None)


def get_current_workspace_name() -> Union[str, None]:
    """Retrieve the current workspace name from the environment variable."""
    ros_ws_name = os.environ.get(ROS_TEAM_WS_WS_NAME_ENV_VAR, None)
    if not ros_ws_name:
        logger.debug(f"Environment variable '{ROS_TEAM_WS_WS_NAME_ENV_VAR}' not set.")
        return None

    return ros_ws_name


def env_var_to_workspace_var(env_var: str, env_var_value: Union[str, None]) -> Tuple[str, Any]:
    """Convert an environment variable to a workspace variable."""
    ws_var = env_var.replace(ROS_TEAM_WS_PREFIX, "").lower()
    if env_var_value == "false":
        ws_var_value = False
    elif env_var_value == "true":
        ws_var_value = True
    else:
        ws_var_value = env_var_value
    return ws_var, ws_var_value


def workspace_var_to_env_var(ws_var: str, ws_var_value: Any) -> Tuple[str, str]:
    """Convert a workspace variable to an environment variable."""
    env_var = ROS_TEAM_WS_PREFIX + ws_var.upper()
    if type(ws_var_value) is bool:
        env_var_value = str(ws_var_value).lower()
    else:
        env_var_value = ws_var_value
    return env_var, env_var_value


def create_bash_script_content_for_using_ws(
    workspace: Workspace, use_workspace_script_path: str
) -> str:
    """Create a bash script content for using a workspace."""
    bash_script_content = "#!/bin/bash\n"

    ws_data = workspace.to_dict()
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


def get_compile_cmd(
    ws_path_abs: str,
    distro: str,
    upstream_ws_abs_path: Union[str, None] = None,
    distro_setup_bash_format: str = "/opt/ros/{distro}/setup.bash",
    upstream_ws_setup_bash_format: str = "{upstream_ws_abs_path}/install/setup.bash",
) -> List[str]:
    """Return a compile command for the given workspace."""
    if upstream_ws_abs_path:
        setup_bash_path = upstream_ws_setup_bash_format.format(
            upstream_ws_abs_path=upstream_ws_abs_path
        )
    else:
        setup_bash_path = distro_setup_bash_format.format(distro=distro)
    compile_ws_cmd = [
        "cd",
        ws_path_abs,
        "&&",
        "source",
        setup_bash_path,
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


def create_choice_format_string(
    ws_name_width: int, distro_width: int, ws_folder_width: int, num_spaces: int = 2
) -> str:
    return (
        f"{{ws_name:<{ws_name_width+num_spaces}}}"
        f" {{distro:<{distro_width+num_spaces}}}"
        f" {{ws_folder:<{ws_folder_width+num_spaces}}}"
        f" {{docker_tag}}"
    )


def get_choice_format_template(choice_format: str) -> str:
    return (
        "["
        + (
            "".join(c for c in choice_format if c.isalpha() or c in "_ {}")
            .replace("{", "<")
            .replace("}", ">")
        )
        + "]"
    )


def get_selected_ws_names_from_user(
    workspaces: Dict[str, Workspace],
    select_question_msg: str = "Select workspaces",
    confirm_question_msg: str = "Are you sure you want to select the following workspaces?",
    start_index: int = 1,
) -> List[str]:
    logger.debug("ws selection")
    ws_name_width = max(len(ws_name) for ws_name in workspaces.keys())
    distro_width = max(len(ws.distro) for ws in workspaces.values())
    ws_folder_width = max(len(str(ws.ws_folder)) for ws in workspaces.values())
    choice_format = create_choice_format_string(ws_name_width, distro_width, ws_folder_width)

    choice_data = {}
    for ws_name, ws in workspaces.items():
        ws_choice_data = choice_format.format(
            ws_name=ws_name,
            distro=ws.distro,
            ws_folder=ws.ws_folder,
            docker_tag=ws.docker_tag,
        )
        choice_data[ws_choice_data] = ws_name

    choice_format_template = get_choice_format_template(choice_format)
    ws_choices_to_delete = questionary.checkbox(
        message=f"{select_question_msg} {choice_format_template}\n",
        choices=list(choice_data.keys()),
        style=questionary.Style([("highlighted", "bold")]),
    ).ask()
    if not ws_choices_to_delete:  # cancelled by user
        exit(0)

    logger.debug("ws selection confirmation")
    workspaces_to_confirm = "\n".join(
        f"{i}. {ws_choice}" for i, ws_choice in enumerate(ws_choices_to_delete, start=start_index)
    )

    confirm_delete = questionary.confirm(
        f"{confirm_question_msg} {choice_format_template}" f"\n{workspaces_to_confirm}\n"
    ).ask()
    if not confirm_delete:  # cancelled by user
        exit(0)

    ws_names_to_delete = [
        choice_data[ws_choice_to_delete] for ws_choice_to_delete in ws_choices_to_delete
    ]
    return ws_names_to_delete
