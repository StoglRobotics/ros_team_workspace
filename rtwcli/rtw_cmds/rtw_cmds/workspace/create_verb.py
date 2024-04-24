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
import os
import pathlib
from pprint import pprint
import shutil
import textwrap
from typing import Any, List
import questionary
from rtwcli.constants import (
    BASHRC_PATH,
    ROS_TEAM_WS_GIT_HTTPS_URL,
    ROS_TEAM_WS_PATH,
    SKEL_BASHRC_PATH,
    WORKSPACES_PATH,
)
from rtwcli.docker_utils import (
    change_docker_path_permissions,
    docker_build,
    docker_cp,
    docker_exec_bash_cmd,
    docker_stop,
    is_docker_tag_valid,
)
from rtwcli.utils import (
    create_file_and_write,
    create_temp_file,
    git_clone,
    run_bash_command,
    run_command,
    vcs_import,
)
from rtwcli.verb import VerbExtension
import docker
from rtwcli.workspace_manger import (
    Workspace,
    WorkspacesConfig,
    get_compile_cmd,
    get_workspace_names,
    save_workspaces_config,
    update_workspaces_config,
)


DEFAULT_BASE_IMAGE_NAME_FORMAT = "osrf/ros:{ros_distro}-desktop"
DEFAULT_INTERMEDIATE_IMAGE_NAME_FORMAT = "rtw_{base_image_name}_intermediate"
DEFAULT_FINAL_IMAGE_NAME_FORMAT = "rtw_{workspace_name}_final"
DEFAULT_CONTAINER_NAME_FORMAT = "{final_image_name}-instance"
DEFAULT_RTW_DOCKER_BRANCH = "rtw_ws_create"
DEFAULT_UPSTREAM_WS_NAME_FORMAT = "{workspace_name}_upstream"
DEFAULT_WS_REPOS_FILE_FORMAT = "{repo_name}.{ros_distro}.repos"
DEFAULT_UPSTREAM_WS_REPOS_FILE_FORMAT = "{repo_name}.{ros_distro}.upstream.repos"
DEFAULT_APT_PACKAGES = [
    "bash-completion",
    "git",
    "git-lfs",
    "iputils-ping",
    "nano",
    "python3-colcon-common-extensions",
    "python3-pip",
    "python3-vcstool",
    "sudo",
    "tmux",
    "trash-cli",
    "tree",
    "vim",
    "wget",
]
DEFAULT_PYTHON_PACKAGES = ["pre-commit"]
DEFAULT_SSH_ABS_PATH = os.path.expanduser("~/.ssh")
DEFAULT_INTERMEDIATE_DOCKERFILE_NAME = "Dockerfile"
DEFAULT_INTERMEDIATE_DOCKERFILE_SAVE_FOLDER_FORMAT = "{ws_folder}/docker"


@dataclass
class CreateVerbArgs:
    ws_abs_path: str
    ros_distro: str
    docker: bool = False
    repos_location_url: str = None
    repos_branch: str = None
    repos_no_skip_existing: bool = False
    disable_nvidia: bool = False
    ws_repos_file_name: str = None
    upstream_ws_repos_file_name: str = None
    upstream_ws_name: str = None
    base_image_name: str = None
    intermediate_image_name: str = None
    final_image_name: str = None
    container_name: str = None
    rtw_docker_repo_url: str = None
    rtw_docker_branch: str = None
    rtw_docker_clone_abs_path: str = None
    apt_packages: List[str] = None
    python_packages: List[str] = None
    ssh_abs_path: str = None
    intermediate_dockerfile_name: str = None
    intermediate_dockerfile_save_folder: str = None
    has_upstream_ws: bool = False
    raise_on_ws_cmd_error: bool = True

    @property
    def ws_name(self) -> str:
        return pathlib.Path(self.ws_abs_path).name

    @property
    def ws_src_abs_path(self) -> str:
        return os.path.join(self.ws_abs_path, "src")

    @property
    def upstream_ws_abs_path(self) -> str:
        return os.path.normpath(os.path.join(self.ws_abs_path, "..", self.upstream_ws_name))

    @property
    def upstream_ws_src_abs_path(self) -> str:
        return os.path.join(self.upstream_ws_abs_path, "src")

    @property
    def repos_location_repo_name(self) -> str:
        if not self.repos_location_url:
            return None
        return self.repos_location_url.split("/")[-1].split(".")[0]

    @property
    def repos_clone_abs_path(self) -> str:
        return os.path.join(self.ws_src_abs_path, self.repos_location_repo_name)

    @property
    def ws_repos_file_abs_path(self) -> str:
        return os.path.join(self.repos_clone_abs_path, self.ws_repos_file_name)

    @property
    def upstream_ws_repos_file_abs_path(self) -> str:
        return os.path.join(self.repos_clone_abs_path, self.upstream_ws_repos_file_name)

    @property
    def intermediate_dockerfile_abs_path(self) -> str:
        return os.path.join(
            self.intermediate_dockerfile_save_folder, self.intermediate_dockerfile_name
        )

    def handle_main_ws_repos(self):
        if not vcs_import(
            self.ws_repos_file_abs_path,
            self.ws_src_abs_path,
            skip_existing=not self.repos_no_skip_existing,
        ):
            raise RuntimeError(
                f"Failed to import ws repos from='{self.ws_repos_file_abs_path}' "
                f"to='{self.ws_src_abs_path}', skip_existing={not self.repos_no_skip_existing}"
            )

    def handle_upstream_ws_repos(self):
        os.makedirs(self.upstream_ws_src_abs_path, exist_ok=True)

        if os.listdir(self.upstream_ws_src_abs_path):
            self.handle_non_empty_src_folder("Upstream workspace", self.upstream_ws_src_abs_path)

        if not vcs_import(
            self.upstream_ws_repos_file_abs_path,
            self.upstream_ws_src_abs_path,
            skip_existing=not self.repos_no_skip_existing,
        ):
            raise RuntimeError(
                f"Failed to import upstream ws repos from='{self.upstream_ws_repos_file_abs_path}' "
                f"to='{self.upstream_ws_src_abs_path}', skip_existing={not self.repos_no_skip_existing}"
            )

        self.has_upstream_ws = True
        print(
            f"Imported upstream ws repos from '{self.upstream_ws_repos_file_abs_path}' "
            f"to '{self.upstream_ws_src_abs_path}'"
        )

        if not os.listdir(self.upstream_ws_src_abs_path):
            remove_upstream_ws = questionary.confirm(
                f"Upstream workspace src folder '{self.upstream_ws_src_abs_path}' "
                "is empty after importing the repos. Do you want to remove the upstream "
                f"workspace folder {self.upstream_ws_abs_path} "
                f"({os.listdir(self.upstream_ws_src_abs_path)})"
                "and create only the main workspace?"
            ).ask()
            if remove_upstream_ws:
                print(f"Removing upstream workspace folder '{self.upstream_ws_abs_path}'")
                shutil.rmtree(self.upstream_ws_abs_path)
                self.has_upstream_ws = False

    def handle_repos(self):
        if not self.ws_repos_file_name:
            self.ws_repos_file_name = DEFAULT_WS_REPOS_FILE_FORMAT.format(
                repo_name=self.repos_location_repo_name, ros_distro=self.ros_distro
            )
        if not self.upstream_ws_repos_file_name:
            self.upstream_ws_repos_file_name = DEFAULT_UPSTREAM_WS_REPOS_FILE_FORMAT.format(
                repo_name=self.repos_location_repo_name, ros_distro=self.ros_distro
            )

        if not git_clone(self.repos_location_url, self.repos_branch, self.repos_clone_abs_path):
            raise RuntimeError(
                f"Failed to clone repo='{self.repos_location_url}', "
                f"branch='{self.repos_branch}', path='{self.repos_clone_abs_path}'"
            )

        print(
            f"Successfully cloned repo='{self.repos_location_url}', "
            f"branch='{self.repos_branch}', path='{self.repos_clone_abs_path}'"
        )

        # main ws repos
        if os.path.exists(self.ws_repos_file_abs_path):
            print(f"Found ws repos file '{self.ws_repos_file_abs_path}'")
            self.handle_main_ws_repos()
            print(
                f"Imported repos from '{self.ws_repos_file_abs_path}' to '{self.ws_src_abs_path}'"
            )
        else:
            print(f"Main ws repos file '{self.ws_repos_file_abs_path}' does not exist.")

        # upstream ws repos
        if os.path.isfile(self.upstream_ws_repos_file_abs_path):
            print(f"Found upstream ws repos file '{self.upstream_ws_repos_file_abs_path}'")
            self.handle_upstream_ws_repos()
            print(
                f"Imported upstream ws repos from '{self.upstream_ws_repos_file_abs_path}' "
                f"to '{self.upstream_ws_src_abs_path}'"
            )
        else:
            print(
                f"Upstream ws repos file '{self.upstream_ws_repos_file_abs_path}' does not exist."
            )

    def handle_non_empty_src_folder(self, ws_type: str, ws_src_path: str):
        # ask the user to still create the workspace even if the src folder is not empty
        still_create_ws = questionary.confirm(
            f"{ws_type} src folder '{ws_src_path}' is not empty. "
            "Do you still want to proceed and create the workspace?"
        ).ask()
        if not still_create_ws:
            exit("Not creating the workspace.")

    def set_default_values(self):
        if not self.upstream_ws_name:
            self.upstream_ws_name = DEFAULT_UPSTREAM_WS_NAME_FORMAT.format(
                workspace_name=self.ws_name
            )

        # docker related attributes
        if not self.base_image_name:
            self.base_image_name = DEFAULT_BASE_IMAGE_NAME_FORMAT.format(
                ros_distro=self.ros_distro
            )
        if not self.intermediate_image_name:
            self.intermediate_image_name = DEFAULT_INTERMEDIATE_IMAGE_NAME_FORMAT.format(
                base_image_name=self.base_image_name.replace(":", "_")
            )
        if not self.final_image_name:
            self.final_image_name = DEFAULT_FINAL_IMAGE_NAME_FORMAT.format(
                workspace_name=self.ws_name
            )
        if not self.container_name:
            self.container_name = DEFAULT_CONTAINER_NAME_FORMAT.format(
                final_image_name=self.final_image_name
            )
        if not self.intermediate_dockerfile_save_folder:
            self.intermediate_dockerfile_save_folder = (
                DEFAULT_INTERMEDIATE_DOCKERFILE_SAVE_FOLDER_FORMAT.format(
                    ws_folder=self.ws_abs_path
                )
            )

    def __post_init__(self):
        self.set_default_values()

        # make sure the workspace src folder exists
        os.makedirs(self.ws_src_abs_path, exist_ok=True)

        if os.listdir(self.ws_src_abs_path):
            self.handle_non_empty_src_folder("Main workspace", self.ws_src_abs_path)

        if self.repos_location_url:
            self.handle_repos()
        else:
            print("No repos location URL provided. Not importing any repos.")


class CreateVerb(VerbExtension):
    """Create a new ROS workspace."""

    def add_arguments(self, parser: argparse.ArgumentParser, cli_name: str):
        parser.formatter_class = argparse.ArgumentDefaultsHelpFormatter
        parser.add_argument(
            "--ws-folder", type=str, help="Path to the workspace folder to create.", required=True
        )
        parser.add_argument(
            "--ros-distro",
            type=str,
            help="ROS distro to use for the workspace.",
            required=True,
            choices=["humble", "rolling"],
        )
        parser.add_argument(
            "--docker", action="store_true", help="Create a docker workspace.", default=False
        )
        parser.add_argument(
            "--repos-location-url",
            type=str,
            help="URL to the workspace repos files.",
            default=None,
        )
        parser.add_argument(
            "--repos-branch",
            type=str,
            help="Branch to use for the workspace repos.",
            default="master",
        )
        parser.add_argument(
            "--repos-no-skip-existing",
            action="store_true",
            help="Parse this flag to vcs import to not overwrite existing packages.",
            default=False,
        )
        parser.add_argument(
            "--disable-nvidia",
            action="store_true",
            help="Disable nvidia rocker flag",
            default=False,
        )
        parser.add_argument(
            "--ws-repos-file-name",
            type=str,
            help=(
                "Workspace repos file name. If not provided, "
                f"default format is used: {DEFAULT_WS_REPOS_FILE_FORMAT}"
            ),
            default=None,
        )
        parser.add_argument(
            "--upstream-ws-repos-file-name",
            type=str,
            help=(
                "Upstream workspace repos file name. If not provided, "
                f"default format is used: {DEFAULT_UPSTREAM_WS_REPOS_FILE_FORMAT}"
            ),
            default=None,
        )
        parser.add_argument(
            "--upstream-ws-name",
            type=str,
            help=(
                "Name of the upstream workspace to use. If not provided, "
                f"default format is used: {DEFAULT_UPSTREAM_WS_NAME_FORMAT}"
            ),
            default=None,
        )
        parser.add_argument(
            "--base-image-name",
            type=str,
            help=(
                "Base image to use for the docker workspace. If not provided, "
                f"default format is used: {DEFAULT_BASE_IMAGE_NAME_FORMAT}"
            ),
            default=None,
        )
        parser.add_argument(
            "--intermediate-image-name",
            type=str,
            help=(
                "Intermediate image name to use for the docker workspace. If not provided, "
                f"default format is used: {DEFAULT_INTERMEDIATE_IMAGE_NAME_FORMAT}"
            ),
            default=None,
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
            "--rtw-docker-repo-url",
            type=str,
            help="URL to the ros_team_workspace repo for the docker workspace.",
            default=ROS_TEAM_WS_GIT_HTTPS_URL,
        )
        parser.add_argument(
            "--rtw-docker-branch",
            type=str,
            help="Branch to use for the ros_team_workspace repo for the docker workspace.",
            default=DEFAULT_RTW_DOCKER_BRANCH,
        )
        parser.add_argument(
            "--rtw-docker-clone-abs-path",
            type=str,
            help="Absolute path to clone the ros_team_workspace repo for the docker workspace.",
            default=ROS_TEAM_WS_PATH,
        )
        parser.add_argument(
            "--apt_packages",
            nargs="*",
            help="Additional apt packages to install.",
            default=DEFAULT_APT_PACKAGES,
        )
        parser.add_argument(
            "--python_packages",
            nargs="*",
            help="Additional python packages to install.",
            default=DEFAULT_PYTHON_PACKAGES,
        )
        parser.add_argument(
            "--ssh-abs-path",
            type=str,
            help="Absolute path to the ssh folder.",
            default=DEFAULT_SSH_ABS_PATH,
        )
        parser.add_argument(
            "--intermediate-dockerfile-name",
            type=str,
            help="Name of the intermediate dockerfile to create.",
            default=DEFAULT_INTERMEDIATE_DOCKERFILE_NAME,
        )
        parser.add_argument(
            "--intermediate-dockerfile-save-folder",
            type=str,
            help=(
                "Folder to save the intermediate dockerfile. If not provided, "
                "default format is used: "
                f"{DEFAULT_INTERMEDIATE_DOCKERFILE_SAVE_FOLDER_FORMAT}"
            ),
            default=None,
        )

    def generate_intermediate_dockerfile_content(self, create_args: CreateVerbArgs) -> str:
        if create_args.apt_packages:
            apt_packages_cmd = " ".join(
                ["RUN", "apt-get", "install", "-y"] + create_args.apt_packages
            )
        else:
            apt_packages_cmd = "# no apt packages to install"

        if create_args.python_packages:
            python_packages_cmd = "RUN pip3 install " + " ".join(create_args.python_packages)
        else:
            python_packages_cmd = "# no python packages to install"

        rtw_clone_cmd = " ".join(
            [
                "RUN",
                "git",
                "clone",
                "-b",
                create_args.rtw_docker_branch,
                create_args.rtw_docker_repo_url,
                create_args.rtw_docker_clone_abs_path,
            ]
        )

        rtw_install_cmd = " ".join(
            [
                "RUN",
                "cd",
                f"{create_args.rtw_docker_clone_abs_path}/rtwcli",
                "&&",
                "pip3",
                "install",
                "-r",
                "requirements.txt",
                "&&",
                "cd",
                "-",
            ]
        )

        return textwrap.dedent(
            f"""
            FROM {create_args.base_image_name}
            RUN apt-get update && apt-get upgrade -y
            {apt_packages_cmd}
            {python_packages_cmd}
            {rtw_clone_cmd}
            {rtw_install_cmd}
            RUN rm -rf /var/lib/apt/lists/*
            """
        )

    def build_intermediate_docker_image(self, create_args: CreateVerbArgs):
        # create intermediate dockerfile
        dockerfile_content = self.generate_intermediate_dockerfile_content(create_args)
        if not create_file_and_write(
            create_args.intermediate_dockerfile_abs_path, content=dockerfile_content
        ):
            raise RuntimeError(
                "Failed to create intermediate dockerfile "
                f"'{create_args.intermediate_dockerfile_abs_path}'"
            )

        # build intermediate docker image
        if not docker_build(
            create_args.intermediate_image_name,
            create_args.intermediate_dockerfile_save_folder,
            create_args.intermediate_dockerfile_abs_path,
        ):
            raise RuntimeError(
                "Failed to build intermediate docker image "
                f"'{create_args.intermediate_image_name}'"
            )

        # check if the intermediate docker image exists
        if not is_docker_tag_valid(create_args.intermediate_image_name):
            raise RuntimeError(
                f"Intermediate docker image '{create_args.intermediate_image_name}' "
                "does not exist."
            )

    def run_intermediate_container(self, create_args: CreateVerbArgs) -> Any:
        volumes = [f"{create_args.ws_abs_path}:{create_args.ws_abs_path}"]
        if create_args.has_upstream_ws:
            volumes.append(
                f"{create_args.upstream_ws_abs_path}:{create_args.upstream_ws_abs_path}"
            )

        print(f"Creating intermediate docker container with volumes: {volumes}")
        try:
            docker_client = docker.from_env()
            intermediate_container = docker_client.containers.run(
                create_args.intermediate_image_name,
                "/bin/bash",
                detach=True,
                remove=True,
                tty=True,
                volumes=volumes,
            )
        except (
            docker.errors.ContainerError,
            docker.errors.ImageNotFound,
            docker.errors.APIError,
        ) as e:
            raise RuntimeError(f"Failed to create intermediate docker container: {e}")

        return intermediate_container

    def get_ws_cmds(self, create_args: CreateVerbArgs) -> List[List[str]]:
        has_ws_packages = True if os.listdir(create_args.ws_src_abs_path) else False
        has_upstream_ws_packages = create_args.has_upstream_ws and os.listdir(
            create_args.upstream_ws_src_abs_path
        )

        rosdep_cmds = [["apt-get", "update"], ["rosdep", "update"]]
        rosdep_install_cmd_base = ["rosdep", "install", "-i", "-r", "-y", "--from-paths"]
        if has_upstream_ws_packages:
            rosdep_cmds.append(rosdep_install_cmd_base + [create_args.upstream_ws_src_abs_path])
        if has_ws_packages:
            rosdep_cmds.append(rosdep_install_cmd_base + [create_args.ws_src_abs_path])
        rosdep_cmds.append(["rm", "-rf", "/var/lib/apt/lists/*"])

        compile_cmds = []
        if create_args.has_upstream_ws:
            compile_cmds.append(
                get_compile_cmd(create_args.upstream_ws_abs_path, create_args.ros_distro)
            )
            compile_cmds.append(
                get_compile_cmd(
                    create_args.ws_abs_path,
                    create_args.ros_distro,
                    upstream_ws_abs_path=create_args.upstream_ws_abs_path,
                )
            )
        else:
            compile_cmds.append(get_compile_cmd(create_args.ws_abs_path, create_args.ros_distro))
        return rosdep_cmds + compile_cmds

    def execute_ws_cmds(
        self, create_args: CreateVerbArgs, ws_cmds: List[List[str]], intermediate_container=None
    ):
        for ws_cmd in ws_cmds:
            print(f"Running ws command: {ws_cmd}")
            ws_cmd_str = " ".join(ws_cmd)
            if create_args.docker:
                if not docker_exec_bash_cmd(intermediate_container.id, ws_cmd_str):
                    if create_args.raise_on_ws_cmd_error:
                        docker_stop(intermediate_container.id)
                        raise RuntimeError(f"Failed to execute ws command '{ws_cmd_str}'")
                    else:
                        print(f"Failed to execute ws command '{ws_cmd_str}'")
            else:
                if not run_bash_command(ws_cmd_str):
                    if create_args.raise_on_ws_cmd_error:
                        raise RuntimeError(f"Failed to execute ws command '{ws_cmd_str}'")
                    else:
                        print(f"Failed to execute ws command '{ws_cmd_str}'")

        if create_args.docker:
            print("Changing workspace folder permissions in the intermediate container.")
            if not change_docker_path_permissions(
                intermediate_container.id, create_args.ws_abs_path
            ):
                if create_args.raise_on_ws_cmd_error:
                    docker_stop(intermediate_container.id)
                    raise RuntimeError(
                        "Failed to change permissions for the main workspace folder "
                        f"{create_args.ws_abs_path}"
                    )
                else:
                    print(
                        "Failed to change permissions for the main workspace folder "
                        f"{create_args.ws_abs_path}"
                    )
            if create_args.has_upstream_ws:
                if not change_docker_path_permissions(
                    intermediate_container.id, create_args.upstream_ws_abs_path
                ):
                    if create_args.raise_on_ws_cmd_error:
                        docker_stop(intermediate_container.id)
                        raise RuntimeError(
                            "Failed to change permissions for the upstream workspace folder "
                            f"{create_args.upstream_ws_abs_path}"
                        )
                    else:
                        print(
                            "Failed to change permissions for the upstream workspace folder "
                            f"{create_args.upstream_ws_abs_path}"
                        )

    def setup_rtw_in_intermediate_image(
        self, create_args: CreateVerbArgs, intermediate_container: Any
    ):
        # create rtw workspaces file
        docker_workspaces_config = WorkspacesConfig()
        if create_args.has_upstream_ws:
            docker_workspaces_config.add_workspace(
                create_args.upstream_ws_name,
                Workspace(
                    distro=create_args.ros_distro,
                    ws_folder=create_args.upstream_ws_abs_path,
                ),
            )
        docker_workspaces_config.add_workspace(
            create_args.ws_name,
            Workspace(
                distro=create_args.ros_distro,
                ws_folder=create_args.ws_abs_path,
                base_ws=create_args.upstream_ws_name if create_args.has_upstream_ws else None,
            ),
        )

        temp_rtw_file = create_temp_file()
        if not save_workspaces_config(temp_rtw_file, docker_workspaces_config):
            docker_stop(intermediate_container.id)
            raise RuntimeError("Failed to save rtw workspaces file.")

        if not docker_cp(intermediate_container.id, temp_rtw_file, WORKSPACES_PATH):
            docker_stop(intermediate_container.id)
            raise RuntimeError("Failed to copy rtw workspaces file to container.")

        # use main workspace per default by adding "rtw workspace use {ws_name}" to bashrc
        # first read the default bashrc
        with open(SKEL_BASHRC_PATH) as file:
            default_bashrc_content = file.read()

        extra_bashrc_content = textwrap.dedent(
            f"""
            # source rtw
            . {create_args.rtw_docker_clone_abs_path}/setup.bash

            # automatically source RosTeamWorkspace if the .ros_team_ws file is present
            if [ -f ~/.ros_team_ws_rc ]; then
                . ~/.ros_team_ws_rc
            fi

            # Stogl Robotics custom setup for nice colors and showing ROS workspace
            . {create_args.rtw_docker_clone_abs_path}/scripts/configuration/terminal_coloring.bash

            # automatically use the main workspace
            rtw workspace use {create_args.ws_name}
            """
        )

        # write the default bashrc with the extra content to the container
        temp_bashrc_file = create_temp_file(content=default_bashrc_content + extra_bashrc_content)
        if not docker_cp(intermediate_container.id, temp_bashrc_file, BASHRC_PATH):
            docker_stop(intermediate_container.id)
            raise RuntimeError("Failed to copy bashrc file to container.")

        # change ownership of the whole home folder
        if not change_docker_path_permissions(intermediate_container.id, os.path.expanduser("~")):
            raise RuntimeError("Failed to change permissions for the home folder.")

        print(f"Committing container '{intermediate_container.id}'")
        try:
            intermediate_container.commit(create_args.intermediate_image_name)
        except docker.errors.APIError as e:
            docker_stop(intermediate_container.id)
            raise RuntimeError(f"Failed to commit container '{intermediate_container.id}': {e}")

        # stop the intermediate container after committing
        docker_stop(intermediate_container.id)

    def generate_rocker_flags(self, create_args: CreateVerbArgs) -> List[str]:
        # rocker flags have order, see rocker --help
        # rocker_flags = ["--nocleanup", "--pull", "--git"]
        rocker_flags = ["--nocleanup", "--git"]
        rocker_flags.extend(["--hostname", f"rtw-{create_args.ws_name}-docker"])
        rocker_flags.extend(["--name", f"{create_args.container_name}"])
        rocker_flags.extend(["--network", "host"])

        if not create_args.disable_nvidia:
            rocker_flags.extend(["--nvidia", "gpus"])

        rocker_flags.extend(["--user", "--user-preserve-home"])

        # rocker volumes
        rocker_flags.append("--volume")
        rocker_flags.append(f"{create_args.ssh_abs_path}:{create_args.ssh_abs_path}:ro")
        rocker_flags.append(f"{create_args.ws_abs_path}:{create_args.ws_abs_path}")
        if create_args.has_upstream_ws:
            rocker_flags.append(
                f"{create_args.upstream_ws_abs_path}:{create_args.upstream_ws_abs_path}"
            )

        rocker_flags.append("--x11")
        rocker_flags.extend(["--mode", "interactive"])
        rocker_flags.extend(["--image-name", f"{create_args.final_image_name}"])

        return rocker_flags

    def execute_rocker_cmd(self, create_args: CreateVerbArgs) -> bool:
        rocker_flags = self.generate_rocker_flags(create_args)
        rocker_base_image_name = create_args.intermediate_image_name
        rocker_cmd = ["rocker"] + rocker_flags + [rocker_base_image_name]
        rocker_cmd_str = " ".join(rocker_cmd)

        print(
            f"Creating final image '{create_args.final_image_name}' "
            f"and container '{create_args.container_name}' "
            f"with command '{rocker_cmd_str}'"
        )

        return run_command(rocker_cmd)

    def main(self, *, args):
        args_dict = vars(args)
        valid_fields = {field.name for field in fields(CreateVerbArgs)}
        filtered_args = {key: args_dict[key] for key in valid_fields if key in args_dict}
        filtered_args["ws_abs_path"] = os.path.normpath(os.path.abspath(args.ws_folder))

        create_args = CreateVerbArgs(**filtered_args)
        print("### CREATE ARGS ###")
        pprint(create_args)
        print("### CREATE ARGS ###")

        ws_names = get_workspace_names()
        if create_args.ws_name in ws_names:
            raise RuntimeError(
                f"Workspace with name '{create_args.ws_name}' already exists. "
                "Overwriting existing workspaces is not supported yet."
            )

        if create_args.docker:
            self.build_intermediate_docker_image(create_args)

        intermediate_container = None
        if create_args.docker:
            intermediate_container = self.run_intermediate_container(create_args)

        ws_cmds = self.get_ws_cmds(create_args)
        print("### WS CMDS ###")
        pprint(ws_cmds)
        print("### WS CMDS ###")
        self.execute_ws_cmds(create_args, ws_cmds, intermediate_container)

        if create_args.docker:
            self.setup_rtw_in_intermediate_image(create_args, intermediate_container)
            if not self.execute_rocker_cmd(create_args):
                # ask the user to still save ws config even if there was a rocker error
                still_save_config = questionary.confirm(
                    "Rocker command failed. Do you still want to save the workspace config?"
                ).ask()
                if not still_save_config:
                    exit("Not saving the workspace config.")

        # create local upstream workspace
        if create_args.has_upstream_ws:
            local_upstream_ws = Workspace(
                ws_folder=create_args.upstream_ws_abs_path,
                distro=create_args.ros_distro,
                ws_docker_support=True if create_args.docker else False,
                docker_tag=create_args.final_image_name if create_args.docker else None,
                docker_container_name=create_args.container_name if create_args.docker else None,
            )
            if not update_workspaces_config(
                WORKSPACES_PATH, create_args.upstream_ws_name, local_upstream_ws
            ):
                raise RuntimeError("Failed to update workspaces config with upstream workspace.")

        # create local main workspace
        local_main_ws = Workspace(
            ws_folder=create_args.ws_abs_path,
            distro=create_args.ros_distro,
            ws_docker_support=True if create_args.docker else False,
            docker_tag=create_args.final_image_name if create_args.docker else None,
            base_ws=create_args.upstream_ws_name if create_args.has_upstream_ws else None,
            docker_container_name=create_args.container_name if create_args.docker else None,
        )
        if not update_workspaces_config(WORKSPACES_PATH, create_args.ws_name, local_main_ws):
            raise RuntimeError("Failed to update workspaces config with main workspace.")
