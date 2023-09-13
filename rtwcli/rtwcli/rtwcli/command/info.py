# Copyright 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
from rtwcli.command import CommandExtension

ROS_TEAM_WS_VARIABLES = [
    "RosTeamWS_BASE_WS",
    "RosTeamWS_DISTRO",
    "RosTeamWS_WS_FOLDER",
    "RosTeamWS_WS_DOCKER_SUPPORT",
    "RosTeamWS_DOCKER_TAG",
]


class InfoCommand(CommandExtension):
    """Provide useful information."""

    def main(self, *, parser, args):
        ros_ws_path = os.environ.get("ROS_WS", None)
        if ros_ws_path:
            ros_version = os.environ.get("ROS_VERSION", None)
            ros_distro = os.environ.get("ROS_DISTRO", None)
            # TODO(destogl): use this instead of 'ROS_WS' in the future
            # ros_ws_folder = os.environ.get("RosTeamWS_WS_FOLDER", None)
            print(os.environ.keys())
            rtw_docker_tag = os.environ.get("RosTeamWS_DOCKER_TAG", None)
            print(f"Using: ROS{ros_version} - {ros_distro}")
            print(f"Your workspace path is '{ros_ws_path}'")
            if rtw_docker_tag:
                print(f"Using docker tag: '{rtw_docker_tag}'")
        else:
            print("ROS_WS was not exported.")
