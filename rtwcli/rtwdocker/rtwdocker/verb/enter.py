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
import re

from rtwdocker.api import start_and_connect_user
from rtwdocker.verb import VerbExtension


class EnterVerb(VerbExtension):
    """Enter docker container."""

    def extract_variables(self, ws_name, bash_script_path):
        with open(bash_script_path) as file:
            content = file.read()

        ws_function_pattern = re.compile(rf"RosTeamWS_setup_{ws_name}.*?}}", re.DOTALL)
        ws_function_match = ws_function_pattern.search(content)

        if ws_function_match:
            ws_function = ws_function_match.group()
            variable_pattern = re.compile(r'(\w+)\s*=\s*"([^"]*)"')
            variable_matches = variable_pattern.findall(ws_function)
            return {var: value for var, value in variable_matches}
        else:
            return {}

    def main(self, *, args):
        print("##### enter docker verb begin #####")

        ROS_WS = os.environ.get("ROS_WS", None)
        if not ROS_WS:
            print(
                'It seems ROS_WS was not exported. Did you source your workspace by executing _"<ws_alias>"?'
            )
            return

        ros_ws_name = ROS_WS.split("/")[-1]
        print(f"ROS_WS is exported: {ROS_WS} -> ros_ws_name: {ros_ws_name}")

        print(
            "RosTeamWS_WS_DOCKER* variables are not exported, therefore getting them from reading '~/.ros_team_ws_rc'"
        )
        variables = self.extract_variables(ros_ws_name, os.path.expanduser("~/.ros_team_ws_rc"))
        print(f"Read variables: {variables}")

        RosTeamWS_WS_DOCKER_SUPPORT = variables.get("RosTeamWS_WS_DOCKER_SUPPORT", None)
        if not RosTeamWS_WS_DOCKER_SUPPORT or RosTeamWS_WS_DOCKER_SUPPORT == "false":
            print(
                'It seems your current workspace does not support docker. If it should, did you activate it by executing _"<ws_alias>"?'
            )
            return

        RosTeamWS_DOCKER_TAG = variables.get("RosTeamWS_DOCKER_TAG", None)
        print(f"RosTeamWS_DOCKER_TAG: {RosTeamWS_DOCKER_TAG}")

        start_and_connect_user(RosTeamWS_DOCKER_TAG)

        print("##### enter docker verb end #####")
