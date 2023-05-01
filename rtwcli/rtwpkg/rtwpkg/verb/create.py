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
import shutil
import ament_copyright
import re
import subprocess

from rtwcli.helpers import (
    check_ros2_command_available,
    get_choice_number_from_user,
    print_error,
    print_info,
)
from rtwpkg.verb import VerbExtension


class CreateVerb(VerbExtension):
    """Create a new ROS package."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument("package_name", help="The package name")
        parser.add_argument(
            "description",
            # default='TODO: Package description',
            help="The description given in the package.xml",
        )

    def extract_build_types(self) -> list:
        ros2_pkg_create_help = subprocess.run(
            ["ros2", "pkg", "create", "-h"], stdout=subprocess.PIPE, text=True
        ).stdout
        build_type_line = re.search(r"--build-type {(.+?)}", ros2_pkg_create_help).group(1)
        build_types = build_type_line.split(",")
        return build_types

    def get_path_from_user(self, message, error_msg) -> str:
        while True:
            path = input(message).strip()
            if os.path.exists(path) and os.path.isdir(path):
                return os.path.abspath(path)
            print(error_msg)

    def get_maintainer_name_from_input(self) -> str:
        while True:
            name = input("Enter the maintainer's name: ").strip()
            if name:
                return name
            print("Name cannot be empty, please enter your name.")

    def get_maintainer_email_from_input(self) -> str:
        email_pattern = re.compile(r"^[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Z|a-z]{2,}$")
        while True:
            email = input("Enter the maintainer's email address: ").strip()
            if email_pattern.match(email):
                return email
            print("Invalid email format, please enter a valid email.")

    def get_maintainer_info(self):
        def get_git_config(config: str):
            git = shutil.which("git")
            if git is not None:
                p = subprocess.Popen([git, "config", f"user.{config}"], stdout=subprocess.PIPE)
                resp = p.communicate()
                return resp[0].decode().rstrip()

        maintainer_name, maintainer_email = None, None
        maintainer_mapping = {}

        user_input_option = "user input"
        maintainer_mapping[1] = user_input_option, None, None

        maintainer_name_git = get_git_config("name")
        maintainer_email_git = get_git_config("email")

        if not maintainer_name_git or not maintainer_email_git:
            maintainer_name = self.get_maintainer_name_from_input()
            maintainer_email = self.get_maintainer_email_from_input()
            return maintainer_name, maintainer_email

        maintainer_mapping[2] = (
            f"git: {maintainer_name_git} <{maintainer_email_git}>",
            maintainer_name_git,
            maintainer_email_git,
        )

        maintainer_options = [
            f"{idx}) {option}" for idx, (option, _, _) in maintainer_mapping.items()
        ]
        print("\n".join(maintainer_options))

        maintainer_choice_idx = get_choice_number_from_user(range(1, len(maintainer_mapping) + 1))

        if maintainer_mapping[maintainer_choice_idx][0] == user_input_option:
            maintainer_name = self.get_maintainer_name_from_input()
            maintainer_email = self.get_maintainer_email_from_input()
        else:
            maintainer_name = maintainer_mapping[maintainer_choice_idx][1]
            maintainer_email = maintainer_mapping[maintainer_choice_idx][2]

        return maintainer_name, maintainer_email

    def get_destination_path(self) -> str:
        destination_mapping = {}

        user_input_option = "user input"
        destination_mapping[1] = (user_input_option, None)

        cwd = os.getcwd()
        destination_mapping[2] = (f"current directory: {cwd}", cwd)

        ros_ws = os.environ.get("ROS_WS", None)
        ros_ws_src = None if ros_ws is None else os.path.join(ros_ws, "src")
        if ros_ws_src is not None:
            destination_mapping[3] = (f"src directory: {ros_ws_src}", ros_ws_src)

        destination_options = [
            f"{idx}) {option}" for idx, (option, _) in destination_mapping.items()
        ]
        print("\n".join(destination_options))

        destination_choice_idx = get_choice_number_from_user(
            range(1, len(destination_options) + 1)
        )

        if destination_mapping[destination_choice_idx][0] == user_input_option:
            return self.get_path_from_user(
                "Enter the destination path to create the ROS package: ",
                "Invalid path, please enter a valid directory path.",
            )
        else:
            return destination_mapping[destination_choice_idx][1]

    def get_build_type(self):
        build_types = self.extract_build_types()
        return self.get_option_from_user(build_types)

    def get_pkg_type(self, pkg_types=["standard", "metapackage", "subpackage"]):
        return self.get_option_from_user(pkg_types)

    def get_option_from_user(self, options):
        options_mapping = {idx: option for idx, option in enumerate(options, start=1)}
        options_to_ask = [f"{idx}) {option}" for idx, option in options_mapping.items()]
        print("\n".join(options_to_ask))
        pkg_type_choice_idx = get_choice_number_from_user(range(1, len(options_to_ask) + 1))
        return options_mapping[pkg_type_choice_idx]

    def get_license_from_user(self) -> str:
        while True:
            name = input("Enter the license's name: ").strip()
            if name:
                return name
            print("License cannot be empty, please enter the license's name.")

    def get_license(self) -> str:
        available_licenses = {}
        for shortname, entry in ament_copyright.get_licenses().items():
            available_licenses[entry.spdx] = entry.license_files

        available_licenses_short_names = list(available_licenses.keys())

        licenses_mapping = {}

        user_input_option = "user input"
        licenses_mapping[1] = "user input", None

        for idx, available_license in enumerate(available_licenses_short_names, start=2):
            licenses_mapping[idx] = available_license, available_license

        licenses_options = [f"{idx}) {option}" for idx, (option, _) in licenses_mapping.items()]
        print("\n".join(licenses_options))

        licenses_choice_idx = get_choice_number_from_user(range(1, len(licenses_options) + 1))

        if licenses_mapping[licenses_choice_idx][0] == user_input_option:
            return self.get_license_from_user()
        else:
            return licenses_mapping[licenses_choice_idx][1]

    def main(self, *, args):
        print("##### rtw pkg create verb begin #####")

        ros2_command_available = check_ros2_command_available()
        print(f"ros2_command_available: {ros2_command_available}")

        if not ros2_command_available:
            print_error(
                "The 'ros2' command is not available. Did you source your workspace by executing _\"<ws_alias>\"?"
            )
            return

        print_info("Where to create the package?")
        destination = self.get_destination_path()
        print_info(f"The package '{args.package_name}' will be created in '{destination}'")

        print()
        print_info("What type of package you want to create?")
        pkg_type = self.get_pkg_type()
        pkg_type_to_print = {
            "standard": "Standard package",
            "metapackage": "Meta-package",
            "subpackage": "Subpackage",
        }[pkg_type]
        print_info(f"{pkg_type_to_print} '{args.package_name}' will be created in '{destination}'")

        print()
        print_info("Who will maintain the package you want to create? Please provide the info.")
        maintainer_name, maintainer_email = self.get_maintainer_info()
        print_info(
            f"The name '{maintainer_name}' and email address '{maintainer_email}' will be used as maintainer info!"
        )

        print()
        print_info("How do you want to licence your package?")
        pkg_license = self.get_license()
        print_info(f"The licence '{pkg_license}' will be used! ($)")

        print()
        print_info("Please choose your package build type:")
        build_type = self.get_build_type()

        print()
        print_info(
            f"ATTENTION: Creating '{pkg_type}' package '{args.package_name}' in '{destination}' with description '{args.description}', license '{pkg_license}', build type '{build_type}' and maintainer '{maintainer_name} <{maintainer_email}>'"
        )
        print("If correct press <ENTER>, otherwise <CTRL>+C and start the script again.")
        input()

        print_error("todo: implement the actual package creating")

        print("##### rtw pkg create verb end #####")
