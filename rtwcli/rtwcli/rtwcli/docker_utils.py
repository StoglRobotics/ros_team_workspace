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
import docker


def is_docker_tag_valid(tag: str) -> bool:
    """Check if a docker image with the given tag exists."""
    try:
        docker_client = docker.from_env()
        return docker_client.images.get(tag) is not None
    except (docker.errors.ImageNotFound, docker.errors.APIError) as e:
        print(f"Failed to get docker image '{tag}': {e}")
        return False


def docker_build(dockerfile_path: str, tag: str) -> bool:
    """Build a docker image with the given tag from the given dockerfile path."""
    docker_build_command = ["docker", "build", "-t", tag, dockerfile_path]
    return run_command(docker_build_command)


def docker_cp(container_name: str, src_path: str, dest_path: str, make_dirs: bool = True) -> bool:
    """Copy a file or directory from the host to the given container."""
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
    """Run a command in the given container."""
    docker_exec_command = [
        "docker",
        "exec",
        container_name,
        command,
    ]
    return run_command(docker_exec_command)


def docker_exec_interactive_bash(container_name: str) -> bool:
    """Run an interactive bash shell in the given container."""
    docker_exec_interactive_bash_command = [
        "docker",
        "exec",
        "-it",
        container_name,
        "/bin/bash",
    ]
    return run_command(docker_exec_interactive_bash_command)


def docker_exec_bash_cmd(container_name: str, bash_cmd: str) -> bool:
    """Run a bash command in the given container."""
    docker_exec_bash_command = [
        "docker",
        "exec",
        container_name,
        "/bin/bash",
        "-c",
        bash_cmd,
    ]
    return run_command(docker_exec_bash_command)


def docker_start(container_name: str) -> bool:
    """Start the given container."""
    return run_command(["docker", "start", container_name])


def docker_stop(container_name: str) -> bool:
    """Stop the given container."""
    return run_command(["docker", "stop", container_name])


def is_docker_container_running(id_or_name: str, running_status: str = "running") -> bool:
    """Check if a docker container with the given id or name is running."""
    try:
        docker_client = docker.from_env()
        container = docker_client.containers.get(id_or_name)
        return container.status == running_status
    except (docker.errors.NotFound, docker.errors.APIError) as e:
        print(f"Failed to get docker container '{id_or_name}': {e}")
        return False


def change_docker_path_permissions(
    container_name: str, path: str, user_in: str = None, group_in: str = None
) -> bool:
    """Change the permissions of the given path in the given container to the given user and group."""
    user = user_in if user_in else os.getuid()
    group = group_in if group_in else os.getgid()
    print(f"Changing permissions of the path '{path}' to '{user}:{group}' in '{container_name}'.")
    return docker_exec_bash_cmd(container_name, f"chown -R {user}:{group} {path}")
