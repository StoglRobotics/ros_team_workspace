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

import click
import shutil
import sys


def print_and_exit(message, usage, color="red"):
    click.echo(click.style(message, fg=color))
    click.echo(usage)
    sys.exit()


def print_info(message, color="yellow"):
    print_message(message, color)


def print_error(message, color="red"):
    print_message(message, color)


def print_message(message, color):
    click.echo(click.style(message, fg=color))


def check_ros2_command_available():
    ros2 = shutil.which("ros2")
    if ros2:
        return True
    else:
        return False


def get_choice_number_from_user(num_range: range):
    while True:
        try:
            user_input = int(input("Enter the number corresponding to your choice: "))
            if user_input in num_range:
                return user_input
            else:
                print("Invalid input. Please enter a valid number.")
        except ValueError:
            print("Invalid input. Please enter a number.")
