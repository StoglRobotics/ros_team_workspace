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


class X11Tmp(RockerExtension):
    @staticmethod
    def get_name() -> str:
        return "x11tmp"

    def __init__(self):
        self.name = X11Tmp.get_name()

    def get_docker_args(self, cliargs: dict) -> str:
        args = [" "]  # To separate from the previous argument
        args.extend(["-e", "DISPLAY"])
        args.extend(["-e", "TERM"])
        args.extend(["-e", "QT_X11_NO_MITSHM=1"])
        args.extend(["--tmpfs", "/tmp"])
        args.extend(["-v", "/tmp/.X11-unix:/tmp/.X11-unix"])
        args.extend(["-v", "/etc/localtime:/etc/localtime:ro"])
        args.append(" ")  # To separate from the next argument
        return " ".join(args)

    @staticmethod
    def register_arguments(parser: argparse.ArgumentParser, defaults: dict = {}) -> None:
        parser.add_argument(
            name_to_argument(X11Tmp.get_name()),
            action="store_true",
            default=defaults.get(X11Tmp.get_name(), None),
            help="Enable x11tmp extension",
        )
