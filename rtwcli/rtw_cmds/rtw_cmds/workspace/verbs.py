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

import argparse
import copy
import dataclasses
import datetime
import os
import pathlib
from pprint import pprint
import re
import shutil
import subprocess
import tempfile
import textwrap
import docker
from typing import Any, Dict, List

import questionary
from rtwcli.command.info import ROS_TEAM_WS_VARIABLES
from rtwcli.helpers import (
    create_file_and_write,
    create_file_if_not_exists,
    load_yaml_file,
    write_to_yaml_file,
)
from rtwcli.verb import VerbExtension


WORKSPACES_PATH = os.path.expanduser("~/.ros_team_workspace/workspaces.yaml")
CURRENT_FILE_DIR = pathlib.Path(__file__).parent.absolute()
USE_WORKSPACE_SCRIPT_PATH = (
    CURRENT_FILE_DIR / ".." / ".." / ".." / ".." / "scripts" / "environment" / "setup.bash"
)
WORKSPACES_KEY = "workspaces"
ROS_TEAM_WS_RC_PATH = os.path.expanduser("~/.ros_team_ws_rc")
BACKUP_DATETIME_FORMAT = "%Y-%m-%d_%H-%M-%S-%f"
WORKSPACES_PATH_BACKUP_FORMAT = os.path.expanduser(
    "~/.ros_team_workspace/bkp/workspaces_bkp_{}.yaml"
)
WS_FOLDER_ENV_VAR = "RosTeamWS_WS_FOLDER"
ROS_TEAM_WS_PREFIX = "RosTeamWS_"

# constants for workspace field names
F_BASE_WS = "base_ws"
F_DISTRO = "distro"
F_DOCKER_TAG = "docker_tag"
F_WS_DOCKER_SUPPORT = "ws_docker_support"
F_WS_FOLDER = "ws_folder"

SKEL_BASHRC_PATH = "/etc/skel/.bashrc"
BASHRC_PATH = os.path.expanduser("~/.bashrc")


@dataclasses.dataclass
class Workspace:
    distro: str
    ws_folder: str
    ws_docker_support: bool = False
    docker_tag: str = None
    docker_container_name: str = None
    base_ws: str = None

    def __post_init__(self):
        self.distro = str(self.distro)
        self.ws_folder = str(self.ws_folder)
        self.ws_docker_support = bool(self.ws_docker_support)
        if self.docker_tag is not None:
            self.docker_tag = str(self.docker_tag)
        if self.base_ws is not None:
            self.base_ws = str(self.base_ws)
        if self.docker_container_name is not None:
            self.docker_container_name = str(self.docker_container_name)


@dataclasses.dataclass
class WorkspacesConfig:
    workspaces: Dict[str, Workspace] = dataclasses.field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: Dict[str, Dict[str, dict]]) -> "WorkspacesConfig":
        if not data:
            return cls({})

        workspaces = {
            ws_name: Workspace(**ws_data)
            for ws_name, ws_data in data.get(WORKSPACES_KEY, {}).items()
        }
        return cls(workspaces)

    def to_dict(self) -> Dict[str, Dict[str, dict]]:
        return {
            WORKSPACES_KEY: {
                ws_name: dataclasses.asdict(ws) for ws_name, ws in self.workspaces.items()
            }
        }

    def get_ws_names(self) -> List[str]:
        if not self.workspaces:
            return []
        return list(self.workspaces.keys())

    def add_workspace(self, ws_name: str, workspace: Workspace) -> bool:
        if ws_name in self.workspaces:
            print(f"Workspace '{ws_name}' already exists in the config.")
            return False
        self.workspaces[ws_name] = workspace
        return True


def load_workspaces_config_from_yaml_file(file_path: str):
    return WorkspacesConfig.from_dict(load_yaml_file(file_path))


def save_workspaces_config(filepath: str, config: WorkspacesConfig):
    return write_to_yaml_file(filepath, config.to_dict())


def get_current_workspace() -> Workspace:
    ws_name = get_current_workspace_name()
    if not ws_name:
        return None

    workspaces_config = load_workspaces_config_from_yaml_file(WORKSPACES_PATH)
    if not workspaces_config:
        return None

    return workspaces_config.workspaces.get(ws_name, None)


def get_current_workspace_name() -> str:
    ros_ws_folder = os.environ.get(WS_FOLDER_ENV_VAR, None)
    if not ros_ws_folder:
        print(f"Environment variable '{WS_FOLDER_ENV_VAR}' not set.")
        return None

    return os.path.basename(ros_ws_folder)


def update_workspaces_config(config_path: str, ws_name: str, workspace: Workspace) -> bool:
    if not create_file_if_not_exists(config_path):
        print("Could not create workspaces config file. Cannot proceed with porting.")
        return False

    workspaces_config = load_workspaces_config_from_yaml_file(config_path)
    if not workspaces_config.add_workspace(ws_name, workspace):
        print(f"Failed to add workspace '{ws_name}' to the config.")
        return False

    # Backup current config file
    current_date = datetime.datetime.now().strftime(BACKUP_DATETIME_FORMAT)
    backup_filename = WORKSPACES_PATH_BACKUP_FORMAT.format(current_date)
    create_file_if_not_exists(backup_filename)
    shutil.copy(config_path, backup_filename)
    print(f"Backed up current workspaces config file to '{backup_filename}'")

    if not save_workspaces_config(config_path, workspaces_config):
        print(f"Failed to update YAML file '{config_path}'.")
        return False

    print(f"Updated YAML file '{config_path}' with a new workspace '{ws_name}'")
    return True


def extract_workspaces_from_bash_script(script_path: str) -> Dict[str, dict]:
    with open(script_path) as file:
        data = file.read()

    workspaces = re.findall(r"(\w+ \(\) \{[^}]+\})", data, re.MULTILINE)
    workspaces_dict = {}
    for workspace in workspaces:
        ws_name = re.search(r"(\w+) \(", workspace).group(1)
        workspaces_dict[ws_name] = {}
        variables = re.findall(r'(?:export )?(\w+)=(".*?"|\'.*?\'|[^ ]+)', workspace)
        for var, val in variables:
            workspaces_dict[ws_name][var] = val.strip('"')

    return workspaces_dict


def generate_workspace_name(ws_path: str, excl_prev_folders=["workspace"]) -> str:
    """Generate workspace name based on the folder letters."""
    ws_path_folders = ws_path.split(os.path.sep)[1:]
    ws_name = ws_path_folders[-1]
    new_ws_name_prefix = ""
    num_folders = len(ws_path_folders)

    if num_folders > 1:  # /folder1/my_ws
        # First letters of the folders, except for the last two
        new_ws_name_prefix += "".join([folder[0] for folder in ws_path_folders[:-2]])
        prev_folder = ws_path_folders[-2]
        if prev_folder not in excl_prev_folders:
            if new_ws_name_prefix:
                new_ws_name_prefix += "_"
            new_ws_name_prefix += prev_folder
        else:
            new_ws_name_prefix += prev_folder[0]

    new_ws_name = "__".join([new_ws_name_prefix, ws_name])
    return new_ws_name


def env_var_to_workspace_var(env_var: str, env_var_value: str) -> str:
    ws_var = env_var.replace(ROS_TEAM_WS_PREFIX, "").lower()
    if env_var_value == "false":
        ws_var_value = False
    elif env_var_value == "true":
        ws_var_value = True
    else:
        ws_var_value = env_var_value
    return ws_var, ws_var_value


def workspace_var_to_env_var(ws_var: str, ws_var_value: Any) -> str:
    env_var = ROS_TEAM_WS_PREFIX + ws_var.upper()
    if type(ws_var_value) == bool:
        env_var_value = str(ws_var_value).lower()
    else:
        env_var_value = ws_var_value
    return env_var, env_var_value


def create_bash_script_content_for_using_ws(
    workspace: Workspace, use_workspace_script_path: str
) -> str:
    """Create a bash script content string using the workspace information."""
    bash_script_content = "#!/bin/bash\n"

    ws_data = dataclasses.asdict(workspace)
    for ws_var, ws_var_value in ws_data.items():
        env_var, env_var_value = workspace_var_to_env_var(ws_var, ws_var_value)
        bash_script_content += f"export {env_var}='{env_var_value}'\n"

    bash_script_content += (
        f"source {use_workspace_script_path} {workspace.distro} {workspace.ws_folder}\n"
    )

    return bash_script_content


def get_expected_ws_field_names() -> List[str]:
    return [field.name for field in dataclasses.fields(Workspace)]


def is_docker_tag_valid(tag: str) -> bool:
    """Validate a given Docker tag by trying to inspect it."""
    try:
        result = subprocess.run(
            ["docker", "image", "inspect", tag],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=True,
        )
        if result.returncode == 0:
            return True
    except subprocess.CalledProcessError:
        print(f"'{tag}' is not a valid Docker tag or image.")
        return False
    except Exception as e:
        print(f"Error while trying to validate Docker tag: {e}")
        return False


def try_port_workspace(workspace_data_to_port: Dict[str, Any], new_ws_name: str) -> bool:
    # set workspace missing fields to default values
    if F_WS_DOCKER_SUPPORT not in workspace_data_to_port:
        workspace_data_to_port[F_WS_DOCKER_SUPPORT] = False
    if F_DOCKER_TAG not in workspace_data_to_port:
        workspace_data_to_port[F_DOCKER_TAG] = None
    if F_BASE_WS not in workspace_data_to_port:
        workspace_data_to_port[F_BASE_WS] = None

    # validate workspace fields
    if not workspace_data_to_port[F_WS_DOCKER_SUPPORT]:
        workspace_data_to_port[F_DOCKER_TAG] = None
    if workspace_data_to_port[F_WS_DOCKER_SUPPORT]:
        if not is_docker_tag_valid(workspace_data_to_port[F_DOCKER_TAG]):
            return False

    # Identify and inform about unexpected fields
    expected_ws_field_names = get_expected_ws_field_names()
    current_ws_field_names = list(workspace_data_to_port.keys())
    unexpected_fields = set(current_ws_field_names) - set(expected_ws_field_names)
    if unexpected_fields:
        print(f"Current fields to port: {', '.join(current_ws_field_names)}")
        print(f"Found unexpected fields: {', '.join(unexpected_fields)}")
        print(f"Available fields are: {', '.join(expected_ws_field_names)}")
        print("These unexpected fields will be skipped.")
        for field in unexpected_fields:
            del workspace_data_to_port[field]

    # ask user for missing ws fields
    choices = [
        "Skip this workspace",
        "Enter the missing field (validation not implemented yet)",
        "Stop porting entirely",
    ]
    for expected_ws_field_name in expected_ws_field_names:
        if expected_ws_field_name in current_ws_field_names:
            continue
        choice = questionary.select(
            f"Missing field '{expected_ws_field_name}'. What would you like to do?",
            choices=choices,
        ).ask()
        if choice is None:  # Cancelled by user
            return False
        elif choice == choices[0]:
            return False
        if choice == choices[1]:
            value = questionary.text(f"Enter value for {expected_ws_field_name}:").ask()
            workspace_data_to_port[expected_ws_field_name] = value
        else:
            exit("Stopped porting due to a missing field.")

    workspace_to_port = Workspace(**workspace_data_to_port)
    print(f"Updating workspace config in '{WORKSPACES_PATH}'")
    success = update_workspaces_config(WORKSPACES_PATH, new_ws_name, workspace_to_port)
    if success:
        print(f"Updated workspace config in '{WORKSPACES_PATH}'")
        return True
    else:
        print(f"Updating workspace config in '{WORKSPACES_PATH}' failed")
        return False


def vcs_import(
    repos_file_path: str,
    path: str,
    non_existing_ok: bool = True,
    empty_ok: bool = True,
    makedirs: bool = True,
    skip_existing: bool = False,
) -> bool:
    if not os.path.isfile(repos_file_path):
        print(f"Repos file '{repos_file_path}' does not exist. Nothing to import.")
        return non_existing_ok

    # check if the file is empty
    if os.path.getsize(repos_file_path) == 0:
        print(f"Repos file '{repos_file_path}' is empty. Nothing to import.")
        return empty_ok

    print(f"Found non-empty repos file '{repos_file_path}', importing repos.")

    if makedirs:
        os.makedirs(path, exist_ok=True)

    vcs_import_cmd = ["vcs", "import", "--input", repos_file_path, "--workers", "1"]
    if skip_existing:
        vcs_import_cmd.append("--skip-existing")
    return run_command(vcs_import_cmd, cwd=path)


def git_clone(url: str, branch: str, path: str) -> bool:
    return run_command(["git", "clone", url, "--branch", branch, path])


def get_compile_cmd(
    ws_path_abs: str,
    distro: str,
    setup_bash_path: str = None,
    distro_setup_bash_format: str = "/opt/ros/{distro}/setup.bash",
) -> List[str]:
    distro_setup_bash_path = distro_setup_bash_format.format(distro=distro)
    compile_ws_cmd = [
        "cd",
        ws_path_abs,
        "&&",
        "source",
        distro_setup_bash_path if not setup_bash_path else setup_bash_path,
        "&&",
        "colcon",
        "build",
        "--symlink-install",
        "--cmake-args",
        "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
    ]
    return compile_ws_cmd


def run_command(command, shell: bool = False, cwd: str = None, ignore_codes=None) -> bool:
    print(f"Running command: '{command}'")
    try:
        subprocess.run(command, shell=shell, check=True, cwd=cwd)
        return True
    except subprocess.CalledProcessError as e:
        print(f"Command '{command}' failed with exit code {e.returncode}")
        if ignore_codes and e.returncode in ignore_codes:
            return True
    except Exception as e:
        print(f"Command '{command}' failed: {e}")
    return False


def create_temp_file(content: str = None) -> str:
    with tempfile.NamedTemporaryFile(delete=False) as tmp_file:
        if content:
            tmp_file.write(content.encode("utf-8"))
        return tmp_file.name


def docker_cp(container_name: str, src_path: str, dest_path: str, make_dirs: bool = True) -> bool:
    if make_dirs:
        # docker_make_dirs_command = (
        #     f"docker exec {container_name} mkdir -p {os.path.dirname(dest_path)}"
        # )
        docker_make_dirs_command = [
            "docker",
            "exec",
            container_name,
            "mkdir",
            "-p",
            os.path.dirname(dest_path),
        ]
        if not run_command(docker_make_dirs_command):
            return False
    docker_cp_command = ["docker", "cp", src_path, f"{container_name}:{dest_path}"]
    return run_command(docker_cp_command)


def docker_exec(container_name: str, command: str) -> bool:
    docker_exec_command = [
        "docker",
        "exec",
        container_name,
        command,
    ]
    return run_command(docker_exec_command)


def docker_exec_bash_cmd(container_name: str, bash_cmd: str) -> bool:
    docker_exec_bash_command = [
        "docker",
        "exec",
        container_name,
        "/bin/bash",
        "-c",
        bash_cmd,
    ]
    return run_command(docker_exec_bash_command)


def docker_stop(container_name: str) -> bool:
    return run_command(["docker", "stop", container_name])


def change_docker_path_permissions(
    container_name: str, path: str, user_in: str = None, group_in: str = None
) -> bool:
    user = user_in if user_in else os.getuid()
    group = group_in if group_in else os.getgid()
    print(f"Changing permissions of the path '{path}' to '{user}:{group}' in '{container_name}'.")
    return docker_exec_bash_cmd(container_name, f"chown -R {user}:{group} {path}")


class CreateVerb(VerbExtension):
    """Create a new ROS workspace."""

    def add_arguments(self, parser: argparse.ArgumentParser, cli_name: str):
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
            "--ws-repos-file",
            type=str,
            help="Workspace repos file name.",
            default="{repo_name}.{ros_distro}.repos",
        )
        parser.add_argument(
            "--upstream-ws-repos-file",
            type=str,
            help="Upstream workspace repos file name.",
            default="{repo_name}.{ros_distro}.upstream.repos",
        )
        parser.add_argument(
            "--upstream-ws-name",
            type=str,
            help="Name of the upstream workspace to use.",
            default="{workspace_name}_upstream",
        )
        parser.add_argument(
            "--base-image",
            type=str,
            help="Base image to use for the docker workspace.",
            default="osrf/ros:{ros_distro}-desktop",
        )
        parser.add_argument(
            "--intermediate-image-name",
            type=str,
            help="Intermediate image name to use for the docker workspace.",
            default="rtw_{base_image_name}_intermediate",
        )
        parser.add_argument(
            "--final-image-name",
            type=str,
            help="Final image name to use for the docker workspace.",
            default="rtw_{workspace_name}_final",
        )
        parser.add_argument(
            "--rtw-repo",
            type=str,
            help="URL to the ros_team_workspace repo.",
            default="https://github.com/StoglRobotics/ros_team_workspace.git",
        )
        parser.add_argument(
            "--rtw-branch",
            type=str,
            help="Branch to use for the ros_team_workspace repo.",
            default="rtw_ws_create",
        )
        parser.add_argument(
            "--rtw-path",
            type=str,
            help="Path to clone the ros_team_workspace repo to.",
            default=os.path.expanduser("~/ros_team_workspace"),
        )
        parser.add_argument(
            "--apt_packages",
            nargs="*",
            help="Additional apt packages to install.",
            default=[
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
            ],
        )
        parser.add_argument(
            "--python_packages",
            nargs="*",
            help="Additional python packages to install.",
            default=["pre-commit"],
        )
        parser.add_argument(
            "--rocker-volumes",
            nargs="*",
            help="Rocker volumes to use for the docker workspace. [Will be overwritten as default for now]",
            default=[
                "~/.ssh:~/.ssh:ro",
                "{ws_folder}:{ws_folder}",
                "{upstream_ws_folder}:{upstream_ws_folder}",  # if upstream ws is used
            ],
        )
        parser.add_argument(
            "--rocker-flags",
            nargs="*",
            help="Additional rocker flags to use for the docker workspace. [Will be overwritten as default for now]",
            default=[
                "--image-name {final_image_name}",
                "--git",
                "--hostname rtw-{workspace_name}-docker",
                "--name {final_image_name}-instance",
                "--nocleanup",
                "--nvidia gpus",
                "--user",
                "--x11",
            ],
        )

    def main(self, *, args):
        print("### ARGS ###")
        pprint(args.__dict__)
        print("### ARGS ###")

        print(f"ROS distro is '{args.ros_distro}'")

        ws_path_abs = os.path.abspath(args.ws_folder)
        src_folder_path_abs = os.path.join(ws_path_abs, "src")
        os.makedirs(src_folder_path_abs, exist_ok=True)
        if os.listdir(src_folder_path_abs):
            print(f"Workspace src folder '{src_folder_path_abs}' is not empty, cannot proceed.")
            return

        ws_name = pathlib.Path(ws_path_abs).name
        print(f"Workspace name is '{ws_name}', path: '{ws_path_abs}'")

        has_upstream_ws = False
        if args.repos_location_url:
            # import repos
            repo_name = args.repos_location_url.split("/")[-1].split(".")[0]
            print(f"Importing repo '{repo_name}' from URL '{args.repos_location_url}'")

            repo_path_abs = os.path.join(src_folder_path_abs, repo_name)
            if not git_clone(args.repos_location_url, args.repos_branch, repo_path_abs):
                print(f"Failed to clone repo '{repo_name}' from '{args.repos_location_url}'.")
                return

            ws_repos_file_name = args.ws_repos_file.format(
                repo_name=repo_name, ros_distro=args.ros_distro
            )
            ws_repos_path_abs = os.path.join(repo_path_abs, ws_repos_file_name)
            if not vcs_import(
                ws_repos_path_abs,
                src_folder_path_abs,
                skip_existing=not args.repos_no_skip_existing,
            ):
                print(f"Failed to import repos from '{ws_repos_path_abs}'.")
                return

            # check if upstream repos file exists
            upstream_ws_name = args.upstream_ws_name.format(workspace_name=ws_name)
            upstream_ws_repos_file_name = args.upstream_ws_repos_file.format(
                repo_name=repo_name, ros_distro=args.ros_distro
            )
            upstream_ws_repos_path_abs = os.path.join(repo_path_abs, upstream_ws_repos_file_name)
            if not os.path.isfile(upstream_ws_repos_path_abs):
                print(
                    f"Upstream repos file '{upstream_ws_repos_path_abs}' does not exist. "
                    "Not creating upstream workspace."
                )
            else:
                # make upstream ws path parallel to the ws path
                upstream_ws_path_abs = os.path.normpath(
                    os.path.join(ws_path_abs, "..", upstream_ws_name)
                )
                upstream_src_folder_path_abs = os.path.join(upstream_ws_path_abs, "src")
                if not vcs_import(
                    upstream_ws_repos_path_abs,
                    upstream_src_folder_path_abs,
                    skip_existing=not args.repos_no_skip_existing,
                ):
                    print(f"Failed to import upstream repos from '{upstream_ws_repos_path_abs}'.")
                    return

                # check if ustream ws is empty
                if os.path.exists(upstream_src_folder_path_abs) and os.listdir(
                    upstream_src_folder_path_abs
                ):
                    has_upstream_ws = True
                    print(f"Upstream workspace '{upstream_ws_name}' is not empty.")
                else:
                    print(f"Upstream workspace '{upstream_ws_name}' is empty.")

        # docker handling: create docker workspace
        final_image_name = None
        has_rocker_error = False
        if args.docker:
            print("Creating docker workspace")

            # check if rocker is installed
            if shutil.which("rocker") is None:
                print("Rocker is not installed. Please install rocker and try again.")
                return

            # create dockerfile
            dockerfile_path_abs = os.path.join(ws_path_abs, "Dockerfile")
            if args.apt_packages:
                apt_packages_cmd = " ".join(
                    ["RUN", "apt-get", "install", "-y"] + args.apt_packages
                )
            else:
                apt_packages_cmd = "# no apt packages to install"
            if args.python_packages:
                python_packages_cmd = "RUN pip3 install " + " ".join(args.python_packages)
            else:
                python_packages_cmd = "# no python packages to install"
            base_image = args.base_image.format(ros_distro=args.ros_distro)
            dockerfile_content = textwrap.dedent(
                f"""
                FROM {base_image}
                RUN apt-get update
                {apt_packages_cmd}
                {python_packages_cmd}
                RUN git clone -b {args.rtw_branch} {args.rtw_repo} {args.rtw_path}
                RUN cd {args.rtw_path}/rtwcli && pip3 install -r requirements.txt && cd -
                """
            )

            # create dockerfile
            if not create_file_and_write(dockerfile_path_abs, content=dockerfile_content):
                print(f"Failed to create dockerfile '{dockerfile_path_abs}'.")
                return

            # build intermediate docker image
            intermediate_image_name = args.intermediate_image_name.format(
                base_image_name=base_image.replace(":", "_")
            )
            docker_client = docker.from_env()
            try:
                print(f"Building intermediate docker image '{intermediate_image_name}'")
                docker_client.images.build(
                    path=ws_path_abs,
                    tag=intermediate_image_name,
                    rm=True,
                )
            except docker.errors.BuildError as e:
                print(f"Failed to build docker image: {e}")
                return

            # check if the intermediate docker image exists
            try:
                docker_client.images.get(intermediate_image_name)
            except (docker.errors.ImageNotFound, docker.errors.APIError) as e:
                print(f"Failed to get docker image '{intermediate_image_name}': {e}")
                return

            has_ws_packages = True if os.listdir(src_folder_path_abs) else False
            has_upstream_ws_packages = has_upstream_ws and os.listdir(upstream_src_folder_path_abs)

            if not has_ws_packages and not has_upstream_ws_packages:
                print("No packages found in the workspaces. No dependencies to install.")
                rocker_base_image_name = intermediate_image_name
            else:
                # Install package dependencies with rosdep and create a new Docker image
                print("Installing ROS package dependencies with rosdep.")

                # Start a container from the intermediate image to install dependencies
                try:
                    deps_volumes = []
                    if has_ws_packages:
                        deps_volumes.append(f"{ws_path_abs}:{ws_path_abs}")
                    if has_upstream_ws_packages:
                        deps_volumes.append(f"{upstream_ws_path_abs}:{upstream_ws_path_abs}")
                    print(f"Creating intermediate docker container with volumes: {deps_volumes}")
                    intermediate_container = docker_client.containers.run(
                        intermediate_image_name,
                        "/bin/bash",
                        detach=True,
                        remove=True,
                        tty=True,
                        volumes=deps_volumes,
                    )
                except (
                    docker.errors.ContainerError,
                    docker.errors.ImageNotFound,
                    docker.errors.APIError,
                ) as e:
                    print(f"Failed to create intermediate docker container: {e}")
                    return

                # Update rosdep and install dependencies
                rosdep_cmds = [["rosdep", "update"]]
                rosdep_install_cmd_base = [
                    "rosdep",
                    "install",
                    "--ignore-src",
                    "-r",
                    "-y",
                    "--from-paths",
                ]
                if has_upstream_ws_packages:
                    rosdep_cmds.append(rosdep_install_cmd_base + [upstream_src_folder_path_abs])
                if has_ws_packages:
                    rosdep_cmds.append(rosdep_install_cmd_base + [src_folder_path_abs])

                for rosdep_cmd in rosdep_cmds:
                    print(f"Running rosdep command: {rosdep_cmd}")
                    if not docker_exec_bash_cmd(intermediate_container.id, " ".join(rosdep_cmd)):
                        print(f"Failed to execute rosdep command {rosdep_cmd}")
                        docker_stop(intermediate_container.id)
                        return

                # compile upstream workspace
                if has_upstream_ws:
                    compile_upstream_ws_cmd = get_compile_cmd(
                        upstream_ws_path_abs, args.ros_distro
                    )
                    print(f"Compiling upstream workspace with command '{compile_upstream_ws_cmd}'")
                    if not docker_exec_bash_cmd(
                        intermediate_container.id, " ".join(compile_upstream_ws_cmd)
                    ):
                        print("Failed to compile upstream workspace.")
                        docker_stop(intermediate_container.id)
                        return

                    # change upstream workspace folder permissions
                    if not change_docker_path_permissions(
                        intermediate_container.id, upstream_ws_path_abs
                    ):
                        print("Failed to change upstream workspace folder permissions.")
                        docker_stop(intermediate_container.id)
                        return

                # compile main workspace
                main_setup_bash_path = (
                    os.path.join(upstream_ws_path_abs, "install/setup.bash")
                    if has_upstream_ws
                    else None
                )
                compile_main_ws_cmd = get_compile_cmd(
                    ws_path_abs,
                    args.ros_distro,
                    setup_bash_path=main_setup_bash_path,
                )
                print(f"Compiling main workspace with command '{compile_main_ws_cmd}'")
                if not docker_exec_bash_cmd(
                    intermediate_container.id, " ".join(compile_main_ws_cmd)
                ):
                    print("Failed to compile main workspace.")
                    docker_stop(intermediate_container.id)
                    return

                # change main workspace folder permissions
                if not change_docker_path_permissions(intermediate_container.id, ws_path_abs):
                    print("Failed to change main workspace folder permissions.")
                    docker_stop(intermediate_container.id)
                    return

            # create rtw workspaces file
            docker_workspaces_config = WorkspacesConfig()
            if has_upstream_ws:
                docker_workspaces_config.add_workspace(
                    upstream_ws_name,
                    Workspace(
                        distro=args.ros_distro,
                        ws_folder=upstream_ws_path_abs,
                    ),
                )
            docker_workspaces_config.add_workspace(
                ws_name,
                Workspace(
                    distro=args.ros_distro,
                    ws_folder=ws_path_abs,
                    base_ws=upstream_ws_name if has_upstream_ws else None,
                ),
            )

            temp_rtw_file = create_temp_file()
            if not save_workspaces_config(temp_rtw_file, docker_workspaces_config):
                print("Failed to create rtw workspaces file.")
                docker_stop(intermediate_container.id)
                return

            if not docker_cp(intermediate_container.id, temp_rtw_file, WORKSPACES_PATH):
                print("Failed to copy rtw workspaces file to container.")
                docker_stop(intermediate_container.id)
                return

            # use main workspace per default by adding "rtw workspace use {ws_name}" to bashrc
            # first read the default bashrc
            with open(SKEL_BASHRC_PATH) as file:
                default_bashrc_content = file.read()

            extra_bashrc_content = textwrap.dedent(
                f"""
                # source rtw
                . {args.rtw_path}/setup.bash

                # automatically source RosTeamWorkspace if the .ros_team_ws file is present
                if [ -f ~/.ros_team_ws_rc ]; then
                    . ~/.ros_team_ws_rc
                fi

                # Stogl Robotics custom setup for nice colors and showing ROS workspace
                . {args.rtw_path}/scripts/configuration/terminal_coloring.bash

                # automatically use the main workspace
                rtw workspace use {ws_name}
                """
            )

            # write the default bashrc with the extra content to the container
            temp_bashrc_file = create_temp_file(
                content=default_bashrc_content + extra_bashrc_content
            )
            if not docker_cp(intermediate_container.id, temp_bashrc_file, BASHRC_PATH):
                print("Failed to copy bashrc file to container.")
                docker_stop(intermediate_container.id)
                return

            # change ownership of the whole home folder
            if not change_docker_path_permissions(
                intermediate_container.id, os.path.expanduser("~")
            ):
                print("Failed to change home folder permissions.")
                return

            # Commit the container to create a new Docker image with dependencies installed
            rocker_base_image_name = intermediate_image_name + "-deps"
            print(
                f"Committing container '{intermediate_container.id}' "
                f"to image '{rocker_base_image_name}'"
            )
            try:
                intermediate_container.commit(rocker_base_image_name)
            except docker.errors.APIError as e:
                print(f"Failed to commit container '{intermediate_image_name}': {e}")
                docker_stop(intermediate_container.id)
                return

            # stop the intermediate container after committing
            docker_stop(intermediate_container.id)

            final_image_name = args.final_image_name.format(workspace_name=ws_name)
            final_container_name = final_image_name + "-instance"

            # create final docker image with rocker
            # overwrite rocker flags for now
            ssh_path_abs = os.path.expanduser("~/.ssh")
            rocker_volumes = [
                ssh_path_abs + ":" + ssh_path_abs + ":ro",
                ws_path_abs + ":" + ws_path_abs,
            ]
            if has_upstream_ws:
                rocker_volumes.append(upstream_ws_path_abs + ":" + upstream_ws_path_abs)

            # rocker flags have order, see rocker --help
            rocker_flags = ["--nocleanup", "--git"]
            rocker_flags.extend(["--hostname", f"rtw-{ws_name}-docker"])
            rocker_flags.extend(["--name", f"{final_container_name}"])

            if not args.disable_nvidia:
                rocker_flags.extend(["--nvidia", "gpus"])

            rocker_flags.extend(["--user", "--user-preserve-home"])

            if rocker_volumes:
                rocker_flags.append("--volume")
                rocker_flags.extend(rocker_volumes)

            rocker_flags.append("--x11")
            rocker_flags.extend(["--mode", "interactive"])
            rocker_flags.extend(["--image-name", f"{final_image_name}"])

            rocker_cmd = ["rocker"] + rocker_flags + [rocker_base_image_name]
            rocker_cmd_str = " ".join(rocker_cmd)

            print(
                f"Creating final image '{final_image_name}' "
                f"and final container '{final_container_name}' "
                f"with command '{rocker_cmd_str}'"
            )

            has_rocker_error = not run_command(rocker_cmd)

        # ask the user to still save ws config even if there was a rocker error
        if has_rocker_error:
            still_save_config = questionary.confirm(
                "There was an error with rocker. Do you want to save the workspace config?"
            ).ask()
            if not still_save_config:
                print("Not saving the workspace config.")
                return

        # create local upstream workspace
        if has_upstream_ws:
            local_upstream_ws = Workspace(
                ws_folder=upstream_ws_path_abs,
                distro=args.ros_distro,
                ws_docker_support=True if args.docker else False,
                docker_tag=final_image_name if args.docker else None,
                docker_container_name=final_container_name if args.docker else None,
            )
            if not update_workspaces_config(WORKSPACES_PATH, upstream_ws_name, local_upstream_ws):
                print("Failed to update workspaces config with upstream workspace.")
                return

        # create local main workspace
        local_main_ws = Workspace(
            ws_folder=ws_path_abs,
            distro=args.ros_distro,
            ws_docker_support=True if args.docker else False,
            docker_tag=final_image_name if args.docker else None,
            base_ws=upstream_ws_name if has_upstream_ws else None,
            docker_container_name=final_container_name if args.docker else None,
        )
        if not update_workspaces_config(WORKSPACES_PATH, ws_name, local_main_ws):
            print("Failed to update workspaces config with main workspace.")
            return


def get_workspace_names() -> List[str]:
    """Retrieve a list of workspace names from the YAML file."""
    if not os.path.isfile(WORKSPACES_PATH):
        return []
    workspaces_config = load_workspaces_config_from_yaml_file(WORKSPACES_PATH)
    return workspaces_config.get_ws_names()


def workspace_name_completer(**kwargs) -> List[str]:
    """Callable returning a list of workspace names."""
    ws_names = get_workspace_names()
    if not ws_names:
        return ["NO_WORKSPACES_FOUND"]
    return ws_names


def add_rtw_workspace_use_args(parser: argparse.ArgumentParser):
    arg = parser.add_argument(
        "workspace_name",
        help="The workspace name",
        nargs="?",
    )
    arg.completer = workspace_name_completer


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
                meta_information=copy.deepcopy(workspaces_config.to_dict()[WORKSPACES_KEY]),
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
        tmp_file = f"/tmp/ros_team_workspace/wokspace_{os.getppid()}.bash"
        print(f"Following text will be written into file '{tmp_file}':\n{script_content}")
        if not create_file_and_write(tmp_file, content=script_content):
            return f"Failed to write workspace data to a file {tmp_file}."

        print(f"Using workspace '{ws_name}'")


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
