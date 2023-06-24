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
import os
import subprocess
import questionary
from typing import Any, List
from rtwcli.command.info import ROS_TEAM_WS_VARIABLES
from rtwcli.verb import VerbExtension
import yaml

WORKSPACES_PATH = "~/.ros_team_workspace/workspaces.yaml"
USE_WORKSPACE_SCRIPT_PATH = "~/ros_team_workspace/scripts/environment/setup.bash"
WORKSPACES_KEY = "workspaces"


def create_file_if_not_exists(file_path: str, initial_content: str = "") -> bool:
    if not os.path.isfile(file_path):
        print(f"Creating file in {file_path}")
        try:
            # Create the directories if they don't exist
            directory = os.path.dirname(file_path)
            if not os.path.exists(directory):
                os.makedirs(directory)

            with open(file_path, "w") as file:
                # Write initial content to the file if needed
                if initial_content:
                    file.write(f"{initial_content}")

            print(f"Created file in {file_path}")
            return True
        except OSError as e:
            print(f"Failed to create file in {file_path}. Error: {e}")
            return False
    else:
        print(f"File already exists in {file_path}")
        return True


def read_yaml_file(file_path: str) -> Any:
    with open(file_path) as file:
        return yaml.safe_load(file)


def write_yaml_file(file_path: str, yaml_data: Any) -> bool:
    with open(file_path, "w") as file:
        try:
            yaml.dump(yaml_data, file)
            print("YAML file updated and saved successfully!")
            return True
        except yaml.YAMLError as e:
            print(f"Failed to save the updated YAML file. Error: {e}")
    return False


def get_ws_names(workspaces_config: dict) -> List[str]:
    if (
        not workspaces_config
        or WORKSPACES_KEY not in workspaces_config
        or not workspaces_config[WORKSPACES_KEY]
    ):
        return []
    return list(workspaces_config[WORKSPACES_KEY].keys())


def update_workspaces_config(config_path: str, workspace: dict) -> bool:
    print(f"config_path: {config_path}")
    if not create_file_if_not_exists(config_path):
        print("Could not create workspaces config file. Cannot proceed with porting.")
        return False

    try:
        workspaces_config = read_yaml_file(config_path)
    except yaml.YAMLError as e:
        print(f"Failed to load yaml file {config_path}. Error: {e}")
        return False
    if not workspaces_config:
        workspaces_config = {WORKSPACES_KEY: {}}
    print(f"workspaces_config: {workspaces_config}")

    assert len(workspace.keys()) == 1, f"The workspace {workspace} does not have exactly one key."
    ws_name = list(workspace.keys())[0]
    existing_ws_names = get_ws_names(workspaces_config)

    if ws_name in existing_ws_names:
        raise NotImplementedError("TODO: Duplicate workspace name handling.")

    workspaces_config[WORKSPACES_KEY][ws_name] = workspace[ws_name]
    print(f"workspaces_config: {workspaces_config}")

    if not write_yaml_file(config_path, workspaces_config):
        print("Error: Failed to update YAML file.")
        return False

    print(f"Updated YAML file '{config_path}' with a new workspace '{workspace}'")
    return True


class CreateVerb(VerbExtension):
    """Create a new ROS workspace."""

    def main(self, *, args):
        print("Not implemented yet")


class UseVerb(VerbExtension):
    """Select and source an existing ROS workspace."""

    def main(self, *, args):
        print(f"Not implemented yet, a bash script is currently used: {USE_WORKSPACE_SCRIPT_PATH}")

        workspaces_config_path = os.path.expanduser(WORKSPACES_PATH)
        try:
            workspaces_config = read_yaml_file(workspaces_config_path)
        except yaml.YAMLError as e:
            print(f"Failed to load yaml file {workspaces_config_path}. Error: {e}")
            return
        if not workspaces_config:
            print(f"No workspaces found in config {workspaces_config_path}")
            return

        print(f"workspaces_config: {workspaces_config}")

        ws_names = get_ws_names(workspaces_config)
        print(f"type workspaces_config[WORKSPACES_KEY]: {type(workspaces_config[WORKSPACES_KEY])}")

        ws_name = str(
            questionary.autocomplete(
                "Choose workspace",
                ws_names,
                qmark="'Tab' to see all workspaces\n",
                meta_information=copy.deepcopy(workspaces_config[WORKSPACES_KEY]),
                validate=lambda ws_choice: ws_choice in ws_names,
            ).ask()
        )
        print(f"Chosen workspace name: {ws_name}")
        print(f"type workspaces_config[WORKSPACES_KEY]: {type(workspaces_config[WORKSPACES_KEY])}")

        if not ws_name or ws_name not in workspaces_config[WORKSPACES_KEY]:
            print(f"workspace name is not valid: {ws_name}")
            return

        ws_data = workspaces_config[WORKSPACES_KEY][ws_name]
        print(f"ws_data: {ws_data}")

        distro = workspaces_config[WORKSPACES_KEY][ws_name]["distro"]
        ws_folder = workspaces_config[WORKSPACES_KEY][ws_name]["ws_folder"]
        print(f"distro: {distro}")
        print(f"ws_folder: {ws_folder}")
        # cmd = ["bash", os.path.expanduser(USE_WORKSPACE_SCRIPT_PATH), distro, ws_folder]
        # cmd = [
        #     "bash",
        #     "-c",
        #     # f"source {os.path.expanduser('~/.bashrc')} && {os.path.expanduser(USE_WORKSPACE_SCRIPT_PATH)} {distro} {ws_folder}",
        #     f"echo $GZ_VERSION && source {os.path.expanduser('~/.bashrc')} && echo $GZ_VERSION",
        # ]
        # subprocess.run(cmd)

        # test_bash_file = "~/ros_team_workspace/scripts/create-new-package.bash"

        # [f"{os.path.expanduser(test_bash_file)}", "foo", "foo"],
        print(
            subprocess.run(
                [
                    "bash",
                    "-c",
                    f"source {os.path.expanduser(USE_WORKSPACE_SCRIPT_PATH)} {distro} {ws_folder}",
                ],
                capture_output=True,
            )
        )


class PortVerb(VerbExtension):
    """Port the currently sourced ROS workspace by creating the corresponding config."""

    def main(self, *, args):
        workspace_data_to_port = {}
        for var in ROS_TEAM_WS_VARIABLES:
            value = os.environ.get(var, None)
            # check if variable is exported
            if value is None:
                print(f"Variable {var} is not exported. Cannot proceed with porting.")
                return
            new_var = var.replace("RosTeamWS_", "").lower()
            workspace_data_to_port[new_var] = value

        ws_folder = os.environ.get("RosTeamWS_WS_FOLDER")
        print(f"ws_folder: {ws_folder}")
        ws_path_folders = ws_folder.split(os.path.sep)[1:]
        print(f"ws_path_folders: {ws_path_folders}")
        ws_name = ws_path_folders[-1]
        print(f"ws_name: {ws_name}")
        new_ws_name_prefix = ""
        num_folders = len(ws_path_folders)
        print(f"num_folders: {num_folders}")

        if num_folders > 1:  # /folder1/my_ws
            # first letter of the folders
            new_ws_name_prefix += "".join([folder[0] for folder in ws_path_folders[:-2]])
            print(f"new_ws_name_prefix: {new_ws_name_prefix}")
            prev_folder = ws_path_folders[-2]
            print(f"prev_folder: {prev_folder}")
            if prev_folder != "workspace":
                if new_ws_name_prefix:
                    new_ws_name_prefix += "_"
                new_ws_name_prefix += prev_folder
            else:
                new_ws_name_prefix += prev_folder[0]
            print(f"new_ws_name_prefix: {new_ws_name_prefix}")

        new_ws_name = "__".join([new_ws_name_prefix, ws_name])
        print(f"new_ws_name: {new_ws_name}")
        workspace_to_port = {new_ws_name: workspace_data_to_port}
        print(f"workspace_to_port: {workspace_to_port}")

        workspaces_config_path = os.path.expanduser(WORKSPACES_PATH)
        update_workspaces_config(workspaces_config_path, workspace_to_port)
