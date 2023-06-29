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
import questionary
from typing import List
from rtwcli.command.info import ROS_TEAM_WS_VARIABLES
from rtwcli.helpers import (
    create_file_and_write,
    create_file_if_not_exists,
    read_yaml_file,
    write_yaml_file,
)
from rtwcli.verb import VerbExtension
import yaml

WORKSPACES_PATH = "~/.ros_team_workspace/workspaces.yaml"
USE_WORKSPACE_SCRIPT_PATH = "~/ros_team_workspace/scripts/environment/setup.bash"
WORKSPACES_KEY = "workspaces"


def get_ws_names(workspaces_config: dict) -> List[str]:
    if (
        not workspaces_config
        or WORKSPACES_KEY not in workspaces_config
        or not workspaces_config[WORKSPACES_KEY]
    ):
        return []
    return list(workspaces_config[WORKSPACES_KEY].keys())


def update_workspaces_config(config_path: str, workspace: dict) -> bool:
    if not create_file_if_not_exists(config_path):
        print("Could not create workspaces config file. Cannot proceed with porting.")
        return False

    try:
        workspaces_config = read_yaml_file(config_path)
    except (OSError, yaml.YAMLError) as e:
        print(f"Failed to load yaml file {config_path}. Error: {e}")
        return False

    if not workspaces_config:
        workspaces_config = {WORKSPACES_KEY: {}}

    assert len(workspace.keys()) == 1, f"The workspace does not have exactly one key: {workspace}"
    ws_name = list(workspace.keys())[0]
    existing_ws_names = get_ws_names(workspaces_config)

    if ws_name in existing_ws_names:
        raise NotImplementedError(
            f"Workspace name '{ws_name}' is already in the config. "
            "Duplicate workspace name handling is not implemented yet."
        )

    workspaces_config[WORKSPACES_KEY][ws_name] = workspace[ws_name]

    try:
        write_yaml_file(config_path, workspaces_config)
    except (OSError, yaml.YAMLError) as e:
        print(f"Failed to update YAML file {config_path}. Error: {e}")
        return False

    print(f"Updated YAML file '{config_path}' with a new workspace '{ws_name}'")
    return True


class CreateVerb(VerbExtension):
    """Create a new ROS workspace."""

    def main(self, *, args):
        print("Not implemented yet")


class UseVerb(VerbExtension):
    """Select and source an existing ROS workspace."""

    def main(self, *, args):
        workspaces_config_path = os.path.expanduser(WORKSPACES_PATH)
        if not os.path.isfile(workspaces_config_path):
            print(
                "No workspaces are available as the workspaces config file "
                f"'{workspaces_config_path}' does not exist"
            )
            return

        try:
            workspaces_config = read_yaml_file(workspaces_config_path)
        except (OSError, yaml.YAMLError) as e:
            print(f"Failed to load yaml file {workspaces_config_path}. Error: {e}")
            return

        if not workspaces_config:
            print(f"No workspaces found in config file '{workspaces_config_path}'")
            return

        ws_names = get_ws_names(workspaces_config)
        ws_name = str(
            questionary.autocomplete(
                "Choose workspace",
                ws_names,
                qmark="'Tab' to see all workspaces\n",
                meta_information=copy.deepcopy(workspaces_config[WORKSPACES_KEY]),
                validate=lambda ws_choice: ws_choice in ws_names,
            ).ask()
        )
        if not ws_name or ws_name not in workspaces_config[WORKSPACES_KEY]:
            print(f"Workspace name is invalid: '{ws_name}'")
            return

        ws_data = workspaces_config[WORKSPACES_KEY][ws_name]
        print("Workspace data:")
        print(yaml.dump(ws_data, indent=4, default_flow_style=False))

        script = os.path.expanduser(USE_WORKSPACE_SCRIPT_PATH)
        distro = ws_data["distro"]
        ws_folder = ws_data["ws_folder"]
        to_write = "#!/bin/bash\n"
        for ws_var, ws_var_value in ws_data.items():
            new_var = "RosTeamWS_" + ws_var.upper()
            to_write += f"export {new_var}='{ws_var_value}'\n"
        to_write += f"source {script} {distro} {ws_folder}\n"
        tmp_file = f"/tmp/ros_team_workspace/wokspace_{os.getppid()}.bash"
        print(f"Following text will be written into file '{tmp_file}':\n{to_write}")
        try:
            create_file_and_write(tmp_file, content=to_write)
        except (OSError, yaml.YAMLError) as e:
            print(f"Failed to write workspace data to a file {tmp_file}. Error: {e}")
            return


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

        workspace_to_port = {new_ws_name: workspace_data_to_port}
        # print(f"workspace_to_port: {workspace_to_port}")

        workspaces_config_path = os.path.expanduser(WORKSPACES_PATH)

        print(f"Updating workspace config in '{workspaces_config_path}'")
        update_workspaces_config(workspaces_config_path, workspace_to_port)
