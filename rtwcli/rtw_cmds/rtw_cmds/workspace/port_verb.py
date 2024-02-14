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

from rtwcli.command.info import ROS_TEAM_WS_VARIABLES
from rtwcli.constants import F_DISTRO, F_WS_FOLDER, WS_FOLDER_ENV_VAR
from rtwcli.verb import VerbExtension
from rtwcli.workspace_manger import (
    env_var_to_workspace_var,
    generate_workspace_name,
    try_port_workspace,
)


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
