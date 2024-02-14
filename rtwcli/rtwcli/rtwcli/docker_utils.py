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

from rtwcli.utils import run_command


def is_docker_tag_valid(tag: str) -> bool:
    """Validate a given Docker tag by trying to inspect it."""
    docker_image_inspect_command = ["docker", "image", "inspect", tag]
    return run_command(docker_image_inspect_command)


def docker_cp(container_name: str, src_path: str, dest_path: str, make_dirs: bool = True) -> bool:
    if make_dirs:
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
