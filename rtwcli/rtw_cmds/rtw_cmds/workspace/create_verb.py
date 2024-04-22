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
import pathlib
from pprint import pprint
import shutil
import textwrap
import questionary
from rtwcli.constants import BASHRC_PATH, SKEL_BASHRC_PATH, WORKSPACES_PATH
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
    run_command,
    vcs_import,
)
from rtwcli.verb import VerbExtension
import docker
from rtwcli.workspace_manger import (
    Workspace,
    WorkspacesConfig,
    get_compile_cmd,
    save_workspaces_config,
    update_workspaces_config,
)


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
                "--pull",
                "--git",
                "--hostname rtw-{workspace_name}-docker",
                "--name {final_image_name}-instance",
                "--network host",
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
            if not docker_build(ws_path_abs, intermediate_image_name):
                print(f"Failed to build intermediate docker image '{intermediate_image_name}'.")
                return

            # check if the intermediate docker image exists
            if not is_docker_tag_valid(intermediate_image_name):
                print(f"Intermediate docker image '{intermediate_image_name}' does not exist.")
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
                deps_volumes = []
                if has_ws_packages:
                    deps_volumes.append(f"{ws_path_abs}:{ws_path_abs}")
                if has_upstream_ws_packages:
                    deps_volumes.append(f"{upstream_ws_path_abs}:{upstream_ws_path_abs}")
                print(f"Creating intermediate docker container with volumes: {deps_volumes}")
                try:
                    docker_client = docker.from_env()
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
            rocker_flags = ["--nocleanup", "--pull", "--git"]
            rocker_flags.extend(["--hostname", f"rtw-{ws_name}-docker"])
            rocker_flags.extend(["--name", f"{final_container_name}"])
            rocker_flags.extend(["--network", "host"])

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
