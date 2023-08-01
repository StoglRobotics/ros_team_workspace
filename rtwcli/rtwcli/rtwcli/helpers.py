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
from typing import Any

import yaml


def create_file_if_not_exists(file_path: str) -> bool:
    if os.path.isfile(file_path):
        return True

    try:
        # Create the directories if they don't exist
        directory = os.path.dirname(file_path)
        if not os.path.exists(directory):
            os.makedirs(directory)

        with open(file_path, "w"):
            return True
    except OSError as e:
        print(f"Failed to create file in {file_path}. Error: {e}")
        return False


def load_yaml_file(file_path: str) -> Any:
    try:
        with open(file_path) as file:
            return yaml.safe_load(file)
    except (OSError, yaml.YAMLError) as e:
        print(f"Failed to load YAML file. Error: {e}")
        return None


def write_to_yaml_file(file_path: str, yaml_data: Any) -> bool:
    try:
        with open(file_path, "w") as file:
            yaml.dump(yaml_data, file)
            return True
    except (OSError, yaml.YAMLError) as e:
        print(f"Failed to write to a YAML file. Error: {e}")
        return False


def create_file_and_write(file_path: str, content: str) -> bool:
    if not create_file_if_not_exists(file_path):
        return False
    try:
        with open(file_path, "w") as file:
            file.write(f"{content}")
            return True
    except OSError as e:
        print(f"Failed to write to a file. Error: {e}", e)
        return False
