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
import subprocess


def start_container(container_instance_name: str):
    if not container_instance_name:
        print("No container name given. Can not start docker instance.")
        return

    print(f"Starting container instance: {container_instance_name}")
    subprocess.run(["xhost", "+local:docker"])
    subprocess.run(["docker", "start", container_instance_name])


def stop_container(container_instance_name):
    if not container_instance_name:
        print("No container name given. Cannot stop docker instance.")
        return

    print(f"Stopping container instance: {container_instance_name}")
    subprocess.run(["xhost", "-local:docker"])
    subprocess.run(["docker", "stop", container_instance_name])


def connect_root_user(container_instance_name: str):
    connect_to_container(container_instance_name, True)


def connect_user(container_instance_name: str):
    connect_to_container(container_instance_name, False)


def connect_to_container(container_instance_name: str, as_root: bool):
    if not container_instance_name:
        print("No container name given. Can not connect to instance.")
        return

    user = "root" if as_root else os.environ.get("USER")
    print(f"Connecting to container instance as user: {container_instance_name}")
    subprocess.run(["docker", "exec", "-u", user, "-it", container_instance_name, "/bin/bash"])


def start_and_connect_user(tag):
    start_and_connect(tag, False)


def start_and_connect_root_user(tag):
    start_and_connect(tag, True)


def start_and_connect(tag: str, as_root: bool):
    if not tag:
        print(
            "The given docker tag is empty, something went wrong. Does your current workspace contain a RosTeamWS_DOCKER_TAG in .ros_team_ws_rc ?"
        )
        return

    container_instance_name = f"{tag}-instance"
    start_container(container_instance_name)
    if as_root:
        connect_root_user(container_instance_name)
    else:
        connect_user(container_instance_name)
