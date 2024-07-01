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

from rtwcli.docker_utils import (
    docker_exec_interactive_bash,
    docker_start,
    fix_missing_xauth_file,
    is_docker_container_running,
)
from rtwcli.verb import VerbExtension
from rtwcli.workspace_utils import get_current_workspace


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

        if not is_docker_container_running(ws.docker_container_name):
            print(
                f"The docker container '{ws.docker_container_name}' is not running, starting it now."
            )
            # fix missing .xauth file if it is not present
            if not fix_missing_xauth_file(ws.docker_container_name):
                print(f"Failed to fix missing .xauth file for '{ws.docker_container_name}'.")
                return

            if not docker_start(ws.docker_container_name):
                print(f"Failed to start docker container '{ws.docker_container_name}'.")
                return

        docker_exec_interactive_bash(ws.docker_container_name)
