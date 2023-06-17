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
from rtwcli.verb import VerbExtension
import yaml

WORKSPACES_PATH = "~/.ros_team_workspace/workspaces.yaml"
WORKSPACES_KEY = "workspaces"
ROS_TEAM_WS_VARIABLES = [
    "RosTeamWS_BASE_WS",
    "RosTeamWS_DISTRO",
    "RosTeamWS_WS_FOLDER",
    "RosTeamWS_WS_DOCKER_SUPPORT",
    "RosTeamWS_DOCKER_TAG",
]


def create_workspaces_config(file_path: str, initial_content: str) -> bool:
    if not os.path.isfile(file_path):
        print(f"Creating workspaces config file in {file_path}")
        try:
            with open(file_path, "w") as file:
                # Write initial content to the file if needed
                file.write(f"{initial_content}:\n")

            print(f"Created workspaces config file in {file_path}")
            return True
        except IOError as e:
            print(f"Failed to create the workspaces config file in {file_path}. Error: {e}")
            return False
    else:
        print(f"Workspaces config file already exists in {file_path}")
        return True


def update_workspaces_config(file_path: str, workspace_key: str, workspace_data: dict) -> bool:
    # Load the YAML file
    with open(file_path, "r") as file:
        try:
            yaml_data = yaml.safe_load(file)
        except yaml.YAMLError as e:
            print(f"Failed to load workspaces config file {file_path}. Error: {e}")
            return False

    # Check if the key exists
    if workspace_key in yaml_data:
        existing_workspaces = yaml_data[workspace_key]

        # Check for duplicate workspace data
        if workspace_data in existing_workspaces:
            print("Error: Duplicate workspace data.")
            print("TODO: implement adding parent folder to the name")
            return

        # Add the new workspace data
        existing_workspaces.append(workspace_data)
    else:
        # If 'workspaces' key doesn't exist, create it with the new workspace data
        yaml_data[workspace_key] = [workspace_data]

    # Save the updated YAML file
    with open(file_path, "w") as file:
        try:
            yaml.dump(yaml_data, file)
            print("YAML file updated and saved successfully!")
        except yaml.YAMLError as e:
            print("Error: Failed to save the updated YAML file.")


class CreateVerb(VerbExtension):
    """Create a new ROS workspace."""

    def main(self, *, args):
        print("Not implemented yet")


class SwitchVerb(VerbExtension):
    """Switches to an existing ROS workspace."""

    def main(self, *, args):
        print("Not implemented yet")


class PortConfigVerb(VerbExtension):
    """Port the currently sourced ROS workspace by creating the corresponding config."""

    def main(self, *, args):
        if not create_workspaces_config(WORKSPACES_PATH, f"{WORKSPACES_KEY}:"):
            print("Could not create workspaces config file. Cannot proceed with porting.")
            return

        workspace_to_port = {}
        for var in ROS_TEAM_WS_VARIABLES:
            value = os.environ.get(var, None)
            # check if variable is exported
            if value is None:
                print(f"Variable {var} is not exported. Cannot proceed with porting.")
                return
            workspace_to_port[var] = value

        # workspaces = yaml.load(WORKSPACES_PATH)
        update_workspaces_config(WORKSPACES_PATH, workspace_to_port)
