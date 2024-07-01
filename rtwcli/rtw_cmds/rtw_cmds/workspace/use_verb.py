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

import argparse
import os
import questionary
from rtwcli.constants import (
    USE_WORKSPACE_SCRIPT_PATH,
    WORKSPACES_PATH,
    WS_USE_BASH_FILE_PATH_FORMAT,
)
from rtwcli.utils import create_file_and_write
from rtwcli.verb import VerbExtension
from rtwcli.workspace_utils import (
    create_bash_script_content_for_using_ws,
    load_workspaces_config_from_yaml_file,
    workspace_name_completer,
)


def add_rtw_workspace_use_args(parser: argparse.ArgumentParser):
    arg = parser.add_argument(
        "workspace_name",
        help="The workspace name",
        nargs="?",
    )
    arg.completer = workspace_name_completer  # type: ignore


class UseVerb(VerbExtension):
    """Select and source an existing ROS workspace."""

    def add_arguments(self, parser: argparse.ArgumentParser, cli_name: str):
        add_rtw_workspace_use_args(parser)

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
                meta_information=workspaces_config.ws_meta_information,
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
        tmp_file = WS_USE_BASH_FILE_PATH_FORMAT.format(ppid=os.getppid())
        print(f"Following text will be written into file '{tmp_file}':\n{script_content}")
        if not create_file_and_write(tmp_file, content=script_content):
            return f"Failed to write workspace data to a file {tmp_file}."

        print(f"Using workspace '{ws_name}'")
