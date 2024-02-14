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

from rtwcli.constants import ROS_TEAM_WS_RC_PATH, WS_FOLDER_ENV_VAR
from rtwcli.verb import VerbExtension
from rtwcli.workspace_manger import (
    env_var_to_workspace_var,
    extract_workspaces_from_bash_script,
    generate_workspace_name,
    try_port_workspace,
)


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
