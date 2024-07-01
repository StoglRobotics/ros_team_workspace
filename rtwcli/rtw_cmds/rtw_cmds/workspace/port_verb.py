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
from enum import Enum
import re
from typing import Dict, List

import rich
import rich.tree
from rtwcli import logger
from rtwcli.constants import (
    RICH_TREE_FST_LVL_COLOR,
    RICH_TREE_GUIDE_STYLE,
    RICH_TREE_LABEL_COLOR,
    RICH_TREE_STATUS_COLOR,
    ROS_TEAM_WS_BASE_WS_ENV_VAR,
    ROS_TEAM_WS_DISTRO_ENV_VAR,
    ROS_TEAM_WS_DOCKER_CONTAINER_NAME_ENV_VAR,
    ROS_TEAM_WS_DOCKER_TAG_ENV_VAR,
    ROS_TEAM_WS_PREFIX,
    ROS_TEAM_WS_WS_DOCKER_SUPPORT_ENV_VAR,
    ROS_TEAM_WS_WS_FOLDER_ENV_VAR,
    ROS_TEAM_WS_RC_PATH,
    WORKSPACES_PATH,
)
from rtwcli.verb import VerbExtension
from rtwcli.workspace_utils import (
    Workspace,
    get_selected_ws_names_from_user,
    update_workspaces_config,
)


WS_FUNCTION_PREFIX = ROS_TEAM_WS_PREFIX + "setup_"
BASE_WS_CURRENT = "<current>"
EMPTY_DOCKER_TAG = "-"
DEFAULT_DOCKER_CONTAINER_NAME_FORMAT = "{docker_tag}-instance"


class PortStatus(Enum):
    """Port status enum."""

    SUCCESS = "SUCCESS"
    FAILED = "FAILED"


def extract_workspaces_from_bash_script(script_path: str) -> Dict[str, Workspace]:
    """Extract workspaces from a bash script."""
    with open(script_path) as file:
        data = file.read()

    # regex: find all functions in the script
    ws_functions = re.findall(r"(\w+ \(\) \{[^}]+\})", data, re.MULTILINE)
    logger.debug(
        f"Found workspace functions: {len(ws_functions)}"
        f"\nFirst ws function: \n{ws_functions[0]} \nLast ws function: \n{ws_functions[-1]}"
    )

    workspaces: Dict[str, Workspace] = {}
    for ws_function in ws_functions:
        ws_function_name = re.search(r"(\w+) \(", ws_function).group(1)  # type: ignore
        if WS_FUNCTION_PREFIX not in ws_function_name:
            logger.info(
                f"WS function name '{ws_function_name}' does not contain "
                f"ws function prefix '{WS_FUNCTION_PREFIX}', skipping."
            )
            continue
        ws_name = ws_function_name[len(WS_FUNCTION_PREFIX) :]
        logger.debug(f"WS name: '{ws_name}'")

        # regex: find all variables and values in the function
        # e.g. (export) RosTeamWS_DISTRO="humble"
        variables_and_values: Dict[str, str] = {
            var: val.strip('"')
            for var, val in re.findall(r'(?:export )?(\w+)=(".*?"|\'.*?\'|[^ ]+)', ws_function)
        }
        logger.debug(f"WS function variables and values: {variables_and_values}")

        if ROS_TEAM_WS_DISTRO_ENV_VAR not in variables_and_values:
            logger.error(
                f"Workspace '{ws_name}' does not have distro '{ROS_TEAM_WS_DISTRO_ENV_VAR}', "
                "skipping."
            )
            continue
        distro = variables_and_values[ROS_TEAM_WS_DISTRO_ENV_VAR]

        if ROS_TEAM_WS_WS_FOLDER_ENV_VAR not in variables_and_values:
            logger.error(
                f"Workspace '{ws_name}' does not have ws folder '{ROS_TEAM_WS_WS_FOLDER_ENV_VAR}', "
                "skipping."
            )
            continue
        ws_folder = variables_and_values[ROS_TEAM_WS_WS_FOLDER_ENV_VAR]

        base_ws = None
        if (
            ROS_TEAM_WS_BASE_WS_ENV_VAR in variables_and_values
            and variables_and_values[ROS_TEAM_WS_BASE_WS_ENV_VAR] != BASE_WS_CURRENT
        ):
            logger.debug(
                f"Workspace '{ws_name}' has base ws that is not '{BASE_WS_CURRENT}', "
                "setting it."
            )
            base_ws = variables_and_values[ROS_TEAM_WS_BASE_WS_ENV_VAR]

        ws_docker_support = False
        if (
            ROS_TEAM_WS_WS_DOCKER_SUPPORT_ENV_VAR in variables_and_values
            and variables_and_values[ROS_TEAM_WS_WS_DOCKER_SUPPORT_ENV_VAR] == "true"
        ):
            logger.debug(
                f"Workspace '{ws_name}' has docker support ('{ROS_TEAM_WS_WS_DOCKER_SUPPORT_ENV_VAR}')"
            )
            ws_docker_support = True

        docker_tag = None
        if ws_docker_support:
            if ROS_TEAM_WS_DOCKER_TAG_ENV_VAR not in variables_and_values:
                logger.error(
                    f"Workspace '{ws_name}' has docker support but does not have docker tag "
                    f"variable '{ROS_TEAM_WS_DOCKER_TAG_ENV_VAR}', skipping."
                )
                continue
            docker_tag = variables_and_values[ROS_TEAM_WS_DOCKER_TAG_ENV_VAR]

        docker_container_name = None
        if ws_docker_support:
            if ROS_TEAM_WS_DOCKER_CONTAINER_NAME_ENV_VAR in variables_and_values:
                logger.debug(
                    f"Workspace '{ws_name}' has docker container name "
                    f"'{ROS_TEAM_WS_DOCKER_CONTAINER_NAME_ENV_VAR}', setting it."
                )
                docker_container_name = variables_and_values[
                    ROS_TEAM_WS_DOCKER_CONTAINER_NAME_ENV_VAR
                ]
            else:
                docker_container_name = DEFAULT_DOCKER_CONTAINER_NAME_FORMAT.format(
                    docker_tag=docker_tag
                )
                logger.info(
                    f"Workspace '{ws_name}' has docker support but does not have docker container name "
                    f"variable, setting it to {docker_container_name}"
                )

        ws = Workspace(
            ws_name=ws_name,
            distro=distro,
            ws_folder=ws_folder,
            ws_docker_support=ws_docker_support,
            docker_tag=docker_tag if docker_tag else "",
            docker_container_name=docker_container_name if docker_container_name else "",
            base_ws=base_ws if base_ws else "",
        )
        workspaces[ws_name] = ws

    return workspaces


def port_workspace_name_completer(**kwargs) -> List[str]:
    workspaces = extract_workspaces_from_bash_script(ROS_TEAM_WS_RC_PATH)
    if not workspaces:
        return ["NO_WORKSPACES_FOUND"]
    return [ws_name for ws_name in workspaces.keys()]


def print_port_stats(ported: List[str], to_be_ported: List[str]) -> None:
    total = len(to_be_ported)
    completed = len(ported)
    tree = rich.tree.Tree(
        f"{RICH_TREE_LABEL_COLOR}Workspace Porting Statistics: {completed}/{total}",
        guide_style=RICH_TREE_GUIDE_STYLE,
    )
    for ws_i, ws_name in enumerate(to_be_ported, start=1):
        status = PortStatus.SUCCESS if ws_name in ported else PortStatus.FAILED
        tree.add(
            f"{RICH_TREE_FST_LVL_COLOR}{ws_i}. {ws_name} - {RICH_TREE_STATUS_COLOR}{status.name}",
            highlight=True,
        )
    rich.print(tree)


class PortVerb(VerbExtension):
    """Port workspace(s) by creating the corresponding config entry."""

    def add_arguments(self, parser: argparse.ArgumentParser, cli_name: str) -> None:
        arg = parser.add_argument(
            "workspace_name",
            help="The workspace name",
            nargs="?",
        )
        arg.completer = port_workspace_name_completer  # type: ignore

    def main(self, *, args):
        script_workspaces = extract_workspaces_from_bash_script(ROS_TEAM_WS_RC_PATH)
        if not script_workspaces:
            logger.info(f"No workspaces found in script '{ROS_TEAM_WS_RC_PATH}'")
            return

        logger.debug(
            f"Script workspaces: {len(script_workspaces)}"
            f"\nFirst script workspace: \n{list(script_workspaces.values())[0]}"
            f"\nLast script workspace: \n{list(script_workspaces.values())[-1]}"
        )

        if args.workspace_name:
            logger.debug(f"Porting workspace from args: {args.workspace_name}")
            ws_names_to_port = [args.workspace_name]
        else:
            ws_names_to_port = get_selected_ws_names_from_user(
                workspaces=script_workspaces,
                select_question_msg="Select workspaces to port",
                confirm_question_msg="Are you sure you want to port the selected workspaces?",
            )
            if not ws_names_to_port:
                logger.info("No workspaces selected to port.")
                return
        logger.debug(f"Ws names to port: {ws_names_to_port}")

        ported = []
        for ws_name in ws_names_to_port:
            if update_workspaces_config(WORKSPACES_PATH, script_workspaces[ws_name]):
                ported.append(ws_name)
                logger.debug(f"Ported workspace: {ws_name}")
            else:
                logger.error(
                    f"Failed to port workspace: {ws_name}. Please see the logs for details."
                )

        print_port_stats(ported=ported, to_be_ported=ws_names_to_port)
