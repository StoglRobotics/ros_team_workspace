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
from dataclasses import dataclass, fields

import questionary
from rtw_cmds.workspace.create_verb import (
    DEFAULT_HOSTNAME_FORMAT,
    DEFAULT_SSH_ABS_PATH,
    DEFAULT_FINAL_IMAGE_NAME_FORMAT,
    DEFAULT_CONTAINER_NAME_FORMAT,
)
from rtwcli.constants import WORKSPACES_PATH
from rtwcli.rocker_utils import execute_rocker_cmd, generate_rocker_flags
from rtwcli.utils import get_filtered_args, replace_user_name_in_path
from rtwcli.verb import VerbExtension
from rtwcli.workspace_utils import Workspace, update_workspaces_config


@dataclass
class ImportVerbArgs:
    ws_name: str
    ros_distro: str
    standalone_docker_image: str
    docker: bool = True
    disable_nvidia: bool = False
    standalone: bool = True
    final_image_name: str = ""
    container_name: str = ""
    ssh_abs_path: str = ""
    hostname: str = ""
    user_override_name: str = ""
    ws_abs_path: str = ""

    @property
    def ssh_abs_path_in_docker(self) -> str:
        if self.user_override_name:
            return replace_user_name_in_path(self.ssh_abs_path, self.user_override_name)
        return self.ssh_abs_path

    def __post_init__(self):
        if not self.hostname:
            self.hostname = DEFAULT_HOSTNAME_FORMAT.format(workspace_name=self.ws_name)
        if not self.final_image_name:
            self.final_image_name = DEFAULT_FINAL_IMAGE_NAME_FORMAT.format(
                workspace_name=self.ws_name
            )
        if not self.container_name:
            self.container_name = DEFAULT_CONTAINER_NAME_FORMAT.format(
                final_image_name=self.final_image_name
            )


class ImportVerb(VerbExtension):
    """Import workspace by creating the corresponding config entry."""

    def add_arguments(self, parser: argparse.ArgumentParser, cli_name: str):
        parser.formatter_class = argparse.ArgumentDefaultsHelpFormatter
        parser.add_argument("--ws-name", type=str, help="Name of the workspace.", required=True)
        parser.add_argument(
            "--ros-distro",
            type=str,
            help="ROS distro of the workspace.",
            required=True,
            choices=["humble", "rolling"],
        )
        parser.add_argument(
            "--standalone-docker-image",
            type=str,
            required=False,
            help="Standalone docker image to use for the workspace.",
        )
        parser.add_argument(
            "--disable-nvidia",
            action="store_true",
            help="Disable nvidia rocker flag",
            default=False,
        )
        parser.add_argument(
            "--final-image-name",
            type=str,
            help=(
                "Final image name to use for the docker workspace. If not provided, "
                f"default format is used: {DEFAULT_FINAL_IMAGE_NAME_FORMAT}"
            ),
            default=None,
        )
        parser.add_argument(
            "--container-name",
            type=str,
            help=(
                "Name of the docker container to use. If not provided, "
                f"default format is used: {DEFAULT_CONTAINER_NAME_FORMAT}"
            ),
            default=None,
        )
        parser.add_argument(
            "--ssh-abs-path",
            type=str,
            help="Absolute path to the ssh folder.",
            default=DEFAULT_SSH_ABS_PATH,
        )
        parser.add_argument(
            "--hostname",
            type=str,
            help=(
                "Hostname to use for the docker workspace. If not provided, "
                "default format is used: "
                f"{DEFAULT_HOSTNAME_FORMAT}"
            ),
            default=None,
        )
        parser.add_argument(
            "--user-override-name",
            type=str,
            help="Override the user name for the workspace.",
            default=None,
        )

    def main(self, *, args):
        filtered_args = get_filtered_args(args, list(fields(ImportVerbArgs)))
        import_args = ImportVerbArgs(**filtered_args)
        rocker_flags = generate_rocker_flags(
            disable_nvidia=import_args.disable_nvidia,
            container_name=import_args.container_name,
            hostname=import_args.hostname,
            ssh_abs_path=import_args.ssh_abs_path,
            ssh_abs_path_in_docker=import_args.ssh_abs_path_in_docker,
            final_image_name=import_args.final_image_name,
            user_override_name=import_args.user_override_name,
        )

        if not execute_rocker_cmd(rocker_flags, import_args.standalone_docker_image):
            # ask the user to still save ws config even if there was a rocker error
            still_save_config = questionary.confirm(
                "Rocker command failed. Do you still want to save the workspace config?"
            ).ask()
            if not still_save_config:
                exit("Not saving the workspace config.")

        # create local main workspace
        local_main_ws = Workspace(
            ws_name=import_args.ws_name,
            ws_folder=import_args.ws_abs_path,
            distro=import_args.ros_distro,
            ws_docker_support=import_args.docker,
            docker_tag=import_args.final_image_name,
            docker_container_name=import_args.container_name,
            standalone=import_args.standalone,
        )
        if not update_workspaces_config(WORKSPACES_PATH, local_main_ws):
            raise RuntimeError("Failed to update workspaces config with main workspace.")
