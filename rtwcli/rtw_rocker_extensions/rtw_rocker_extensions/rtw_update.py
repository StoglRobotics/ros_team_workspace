# Copyright 2019 Open Source Robotics Foundation
# Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt)
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


import argparse
from rocker.extensions import name_to_argument
from rocker.core import RockerExtension


class RTWUPdate(RockerExtension):
    @staticmethod
    def get_name() -> str:
        return "rtw_update"

    def __init__(self):
        self.name = RTWUPdate.get_name()

    def get_user_snippet(self, cliargs):
        return "RUN sudo apt-get update && rosdep update"

    @staticmethod
    def register_arguments(parser: argparse.ArgumentParser, defaults: dict = {}) -> None:
        parser.add_argument(
            name_to_argument(RTWUPdate.get_name()),
            action="store_true",
            default=defaults.get(RTWUPdate.get_name(), None),
            help="RTW: Update after finishing",
        )
