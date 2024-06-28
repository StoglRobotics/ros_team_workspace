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


import argparse
from dataclasses import Field
import getpass
import os
import subprocess
import tempfile
from typing import Any, Dict, List, Union

import yaml


def create_file_if_not_exists(file_path: str) -> bool:
    """
    Create a file if it does not exist.

    Return True if the file exists or was created successfully.
    """
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
    """
    Load a YAML file and return its content.

    Return None if the file does not exist or an error occurred.
    """
    try:
        with open(file_path) as file:
            return yaml.safe_load(file)
    except (OSError, yaml.YAMLError) as e:
        print(f"Failed to load YAML file. Error: {e}")
        return None


def write_to_yaml_file(file_path: str, yaml_data: Any) -> bool:
    """Write data to a YAML file. Return True if the data was written successfully."""
    try:
        with open(file_path, "w") as file:
            yaml.dump(yaml_data, file)
            return True
    except (OSError, yaml.YAMLError) as e:
        print(f"Failed to write to a YAML file. Error: {e}")
        return False


def create_file_and_write(file_path: str, content: str) -> bool:
    """
    Create a file if it does not exist and write the given content to it.

    Return True if the file was created and the content was written successfully.
    """
    if not create_file_if_not_exists(file_path):
        return False
    try:
        with open(file_path, "w") as file:
            file.write(f"{content}")
            return True
    except OSError as e:
        print(f"Failed to write to a file. Error: {e}", e)
        return False


def run_command(
    command,
    shell: bool = False,
    cwd: Union[str, None] = None,
    ignore_codes: Union[List[int], None] = None,
) -> bool:
    """Run a command and return True if it was successful."""
    print(f"Running command: '{command}'")
    try:
        subprocess.run(command, shell=shell, check=True, cwd=cwd)
        return True
    except subprocess.CalledProcessError as e:
        print(f"Command '{command}' failed with exit code {e.returncode}")
        if ignore_codes and e.returncode in ignore_codes:
            return True
    except Exception as e:
        print(f"Command '{command}' failed: {e}")
    return False


def run_bash_command(command: str, shell: bool = False, cwd: Union[str, None] = None) -> bool:
    """Run a bash command and return True if it was successful."""
    return run_command(["bash", "-c", command], shell=shell, cwd=cwd)


def create_temp_file(content: Union[str, None] = None) -> str:
    """Create a temporary file and return its path."""
    with tempfile.NamedTemporaryFile(delete=False) as tmp_file:
        if content:
            tmp_file.write(content.encode("utf-8"))
        return tmp_file.name


def vcs_import(
    repos_file_path: str,
    path: str,
    non_existing_ok: bool = True,
    empty_ok: bool = True,
    makedirs: bool = True,
    skip_existing: bool = False,
) -> bool:
    """Import repositories from a file using vcs. Return True if the import was successful."""
    if not os.path.isfile(repos_file_path):
        print(f"Repos file '{repos_file_path}' does not exist. Nothing to import.")
        return non_existing_ok

    # check if the file is empty
    if os.path.getsize(repos_file_path) == 0:
        print(f"Repos file '{repos_file_path}' is empty. Nothing to import.")
        return empty_ok

    print(f"Found non-empty repos file '{repos_file_path}', importing repos.")

    if makedirs:
        os.makedirs(path, exist_ok=True)

    vcs_import_cmd = ["vcs", "import", "--input", repos_file_path, "--workers", "1"]
    if skip_existing:
        vcs_import_cmd.append("--skip-existing")
    return run_command(vcs_import_cmd, cwd=path)


def git_clone(url: str, branch: str, path: str) -> bool:
    """
    Clone a git repository with the given branch to the given path.

    Return True if the clone was successful.
    """
    return run_command(["git", "clone", url, "--branch", branch, path])


def replace_user_name_in_path(
    path: str, new_user: str, current_user: str = getpass.getuser()
) -> str:
    return path.replace(f"/home/{current_user}", f"/home/{new_user}")


def get_display_manager() -> str:
    # Command to get the display manager type
    cmd = "loginctl show-session $(awk '/tty/ {print $1}' <(loginctl)) -p Type | awk -F= '{print $2}'"
    result = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True)

    # Capture the output and remove any trailing newlines or spaces
    return result.stdout.strip()


def get_filtered_args(args: argparse.Namespace, dataclass_fields: List[Field]) -> Dict[str, Any]:
    args_dict = vars(args)
    valid_fields = {field.name for field in dataclass_fields}
    return {key: args_dict[key] for key in valid_fields if key in args_dict}
