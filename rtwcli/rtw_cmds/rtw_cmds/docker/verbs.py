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
from rtwcli.verb import VerbExtension
from rtw_cmds.workspace.verbs import get_current_workspace
import docker


class EnterVerb(VerbExtension):
    """Enter docker container."""

    def main(self, *, args):
        ws = get_current_workspace()
        if not ws:
            print("No workspace is active.")
            return

        if not ws.ws_docker_support:
            print("The workspace does not support docker.")
            return

        docker_client = docker.from_env()

        # check if the docker container exists
        try:
            container = docker_client.containers.get(ws.docker_container_name)
        except (docker.errors.NotFound, docker.errors.APIError) as e:
            print(f"Failed to get docker container '{ws.docker_container_name}': {e}")
            return

        # start docker container if not running
        if container.status != "running":
            try:
                container.start()
            except docker.errors.APIError as e:
                print(f"Failed to start docker container '{ws.docker_container_name}': {e}")
                return

        # Get container
        # container = client.inspect_container(container_instance_name)

        # Start an exec instance
        exec_instance = docker_client.exec_create(
            ws.docker_container_name,
            cmd="/bin/bash",
            tty=True,
            stdin=True,
            working_dir=ws.ws_folder,
        )

        # Start interactive session
        docker_client.exec_start(exec_instance, detach=False, tty=True, stream=True, stdin=True)

        # Attach to the interactive session
        os.system(f"docker exec -it {ws.docker_container_name} /bin/bash")
