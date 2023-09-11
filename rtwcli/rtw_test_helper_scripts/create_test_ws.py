#!/usr/bin/env python3

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

import docker
import os
import yaml

try:
    from rtw_cmds.workspace.verbs import WORKSPACES_PATH
except ImportError as e:
    print(f"Please install rtwcli first: {e}")
    exit(0)

A_DUMMY_DOCKER_WS_NAME = "a_dummy_docker_ws"
A_DUMMY_WS_NAME = "a_dummy_ws"
TESTING_PATH = "/tmp/ros_team_workspace_testing"
IMAGE_NAME = "hello-world"
DOCKER_TAG = f"hello-world-for-{A_DUMMY_DOCKER_WS_NAME}:latest"
CONTAINER_PREFIX = f"container-for-{A_DUMMY_DOCKER_WS_NAME}"
MAKE_DUMMY_DIRS = True


def build_and_run_docker_image(
    client: docker.DockerClient,
    image_name: str,
    docker_tag: str,
    container_prefix: str,
    num_containers=5,
    detach=True,
    remove=False,
) -> bool:
    try:
        image = client.images.pull(image_name)
    except Exception as e:
        print(f"Error by pulling image '{image_name}': {e}")
        return False

    # Tag image
    try:
        image.tag(docker_tag)
    except Exception as tag_e:
        print(f"Error tagging image '{image_name}' with tag '{docker_tag}': {tag_e}")
        return False

    # Create and run containers from the new image
    for i in range(num_containers):
        container_name = f"{container_prefix}{i}"
        try:
            client.containers.run(
                docker_tag,
                name=container_name,
                detach=detach,
                remove=remove,
            )
        except Exception as e:
            print(f"Caught error for container '{container_name}': {e}")
            return False

    return True


def update_yaml_config(
    workspaces_config_path: str,
    a_dummy_docker_ws_name: str,
    a_dummy_ws_name: str,
    docker_tag: str,
    docker_ws_path: str,
    ws_path: str,
    make_dirs=True,
):
    with open(workspaces_config_path) as f:
        config = yaml.safe_load(f)

    if not config:
        config = {"workspaces": {}}

    # Update or add new workspace configurations
    config["workspaces"].update(
        {
            a_dummy_docker_ws_name: {
                "base_ws": "<current>",
                "distro": "rolling",
                "docker_tag": docker_tag,
                "ws_docker_support": True,
                "ws_folder": docker_ws_path,
            },
            a_dummy_ws_name: {
                "base_ws": "<current>",
                "distro": "rolling",
                "docker_tag": None,
                "ws_docker_support": False,
                "ws_folder": ws_path,
            },
        }
    )

    with open(workspaces_config_path, "w") as f:
        yaml.safe_dump(config, f)

    if make_dirs:
        os.makedirs(ws_path)
        os.makedirs(docker_ws_path)


if __name__ == "__main__":
    client = docker.from_env()

    if build_and_run_docker_image(
        client=client,
        image_name=IMAGE_NAME,
        docker_tag=DOCKER_TAG,
        container_prefix=CONTAINER_PREFIX,
    ):
        update_yaml_config(
            workspaces_config_path=WORKSPACES_PATH,
            a_dummy_docker_ws_name=A_DUMMY_DOCKER_WS_NAME,
            a_dummy_ws_name=A_DUMMY_WS_NAME,
            docker_tag=DOCKER_TAG,
            docker_ws_path=os.path.join(TESTING_PATH, A_DUMMY_DOCKER_WS_NAME),
            ws_path=os.path.join(TESTING_PATH, A_DUMMY_WS_NAME),
            make_dirs=MAKE_DUMMY_DIRS,
        )
