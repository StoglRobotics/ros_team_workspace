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

from rtwcli import logger
from rtwcli.docker_utils import (
    docker_exec_interactive_bash,
    docker_start,
    docker_stop,
    fix_missing_xauth_file,
    is_docker_container_running,
)
from rtwcli.verb import VerbExtension
from rtwcli.workspace_utils import Workspace, get_current_workspace


def check_workspace(ws: Workspace) -> bool:
    if not ws:
        logger.info("No workspace is active.")
        return False

    if not ws.ws_docker_support:
        logger.info("The workspace does not support docker.")
        return False

    if not ws.docker_container_name:
        logger.error("The workspace is missing the docker_container_name attribute.")
        return False

    return True


def container_start_routine(ws: Workspace) -> bool:
    if not check_workspace(ws):
        return False

    if not is_docker_container_running(ws.docker_container_name):
        logger.info(
            f"The docker container '{ws.docker_container_name}' is not running, starting it now."
        )
        # fix missing .xauth file if it is not present
        if not fix_missing_xauth_file(ws.docker_container_name):
            logger.error(f"Failed to fix missing .xauth file for '{ws.docker_container_name}'.")
            return False

        if not docker_start(ws.docker_container_name):
            logger.error(f"Failed to start docker container '{ws.docker_container_name}'.")
            return False

    return True


def container_stop_routine(ws: Workspace) -> bool:
    if not check_workspace(ws):
        return False

    if is_docker_container_running(ws.docker_container_name):
        logger.info(f"Stopping docker container '{ws.docker_container_name}'.")
        if not docker_stop(ws.docker_container_name):
            logger.error(f"Failed to stop docker container '{ws.docker_container_name}'.")
            return False

    return True


class EnterVerb(VerbExtension):
    """Enter docker container."""

    def main(self, *, args):
        ws = get_current_workspace()
        if not container_start_routine(ws):
            return
        docker_exec_interactive_bash(ws.docker_container_name)


class StartVerb(VerbExtension):
    """Start docker container."""

    def main(self, *, args):
        container_start_routine(get_current_workspace())


class StopVerb(VerbExtension):
    """Stop docker container."""

    def main(self, *, args):
        container_stop_routine(get_current_workspace())


class RestartVerb(VerbExtension):
    """Restart docker container."""

    def main(self, *, args):
        ws = get_current_workspace()
        if not container_stop_routine(ws):
            return
        container_start_routine(ws)
