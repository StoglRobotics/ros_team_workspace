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
from typing import Union

from rtwcli import logger
from rtwcli.utils import create_file_if_not_exists, run_command
import docker


def is_docker_tag_valid(tag: str) -> bool:
    """Check if a docker image with the given tag exists."""
    try:
        docker_client = docker.from_env()
        return docker_client.images.get(tag) is not None
    except (
        docker.errors.ImageNotFound,  # type: ignore
        docker.errors.APIError,  # type: ignore
    ) as e:
        logger.error(f"Failed to get docker image '{tag}': {e}")
        return False


def docker_build(
    tag: str,
    dockerfile_path: str,
    file: Union[str, None] = None,
    pull: bool = True,
    no_cache: bool = True,
) -> bool:
    """Build a docker image with the given tag from the given dockerfile path."""
    docker_build_command = ["docker", "build", "-t", tag]
    if no_cache:
        docker_build_command.append("--no-cache")
    if pull:
        docker_build_command.append("--pull")
    if file:
        docker_build_command.extend(["-f", file])
    docker_build_command.append(dockerfile_path)
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


def remove_docker_image(tag: str, force: bool = False) -> bool:
    """Remove a docker image with the given tag."""
    try:
        docker_client = docker.from_env()
        image = docker_client.images.get(tag)
        docker_client.images.remove(image.id, force=force)
        return True
    except (
        docker.errors.ImageNotFound,  # type: ignore
        docker.errors.APIError,  # type: ignore
    ) as e:
        logger.error(f"Failed to remove docker image '{tag}': {e}")
        return False


def docker_container_exists(id_or_name: str) -> bool:
    """Check if a docker container with the given id or name exists."""
    try:
        docker_client = docker.from_env()
        docker_client.containers.get(id_or_name)
        return True
    except (
        docker.errors.NotFound,  # type: ignore
        docker.errors.APIError,  # type: ignore
    ) as e:
        logger.error(f"Failed to get docker container '{id_or_name}': {e}")
        return False


def remove_docker_container(id_or_name: str, force: bool = False) -> bool:
    """Remove a docker container with the given id or name."""
    try:
        docker_client = docker.from_env()
        container = docker_client.containers.get(id_or_name)
        container.remove(force=force)
        return True
    except (
        docker.errors.NotFound,  # type: ignore
        docker.errors.APIError,  # type: ignore
    ) as e:
        logger.error(f"Failed to remove docker container '{id_or_name}': {e}")
        return False


def is_docker_container_running(id_or_name: str, running_status: str = "running") -> bool:
    """Check if a docker container with the given id or name is running."""
    try:
        docker_client = docker.from_env()
        container = docker_client.containers.get(id_or_name)
        return container.status == running_status
    except (
        docker.errors.NotFound,  # type: ignore
        docker.errors.APIError,  # type: ignore
    ) as e:
        logger.error(f"Failed to get docker container '{id_or_name}': {e}")
        return False


def change_docker_path_permissions(
    container_name: str,
    path: str,
    user_in: Union[str, None] = None,
    group_in: Union[str, None] = None,
) -> bool:
    """Change the permissions of the given path in the given container to the given user and group."""
    user = user_in if user_in else os.getuid()
    group = group_in if group_in else os.getgid()
    logger.info(
        f"Changing permissions of the path '{path}' to '{user}:{group}' in '{container_name}'."
    )
    return docker_exec_bash_cmd(container_name, f"chown -R {user}:{group} {path}")


def fix_missing_xauth_file(
    container_name: str,
    mounts_attr: str = "Mounts",
    source_key: str = "Source",
    xauth_file_ext: str = ".xauth",
) -> bool:
    """Fix missing xauth file for the given container."""
    try:
        docker_client = docker.from_env()
        container = docker_client.containers.get(container_name)
    except (
        docker.errors.NotFound,  # type: ignore
        docker.errors.APIError,  # type: ignore
    ) as e:
        logger.error(f"Failed to get docker container '{container_name}': {e}")
        return False

    if not container:
        logger.error(f"Container object is None for container '{container_name}'.")
        return False

    if not container.attrs:
        logger.error(f"Container attributes are None for container '{container_name}'.")
        return False

    if mounts_attr not in container.attrs:
        logger.error(
            f"Container attributes do not contain '{mounts_attr}' for container '{container_name}'."
        )
        return False

    xauth_file_abs_path = None
    for mount in container.attrs[mounts_attr]:
        if source_key in mount and xauth_file_ext in mount[source_key]:
            xauth_file_abs_path = mount[source_key]
            logger.info(
                f"Found {xauth_file_ext} file '{xauth_file_abs_path}' for container '{container_name}'."
            )
            break

    if not xauth_file_abs_path:
        logger.info(
            f"There is no {xauth_file_ext} file for container '{container_name}'. Nothing to do."
        )
        return True

    if os.path.isfile(xauth_file_abs_path):
        logger.info(f"File '{xauth_file_abs_path}' already exists.")
        return True

    if os.path.isdir(xauth_file_abs_path):
        logger.info(f"Path '{xauth_file_abs_path}' is a directory, removing it.")
        try:
            os.rmdir(xauth_file_abs_path)
        except OSError as e:
            logger.error(f"Failed to remove directory '{xauth_file_abs_path}': {e}")
            logger.error("========================================")
            logger.error("Please remove it manually and try again.")
            logger.error("========================================")
            return False

    if not create_file_if_not_exists(xauth_file_abs_path):
        logger.error(f"Failed to create file '{xauth_file_abs_path}'.")
        return False

    cmd = f"xauth nlist :0 | sed -e 's/^..../ffff/' | xauth -f {xauth_file_abs_path} nmerge -"

    if not run_command(cmd, shell=True):
        logger.error(
            f"Failed to run command '{cmd}'. File '{xauth_file_abs_path}' will be removed."
        )
        os.remove(xauth_file_abs_path)
        return False

    return True
