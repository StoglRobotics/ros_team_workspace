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


from abc import ABC, abstractmethod
import argparse
from dataclasses import dataclass, field
from enum import Enum, auto
import os
import shutil
import questionary.styles
import rich
from rich.tree import Tree
from typing import Dict, List
import questionary
from rtwcli import logger
from rtwcli.constants import (
    WORKSPACES_PATH,
)
from rtwcli.docker_utils import (
    docker_container_exists,
    docker_stop,
    is_docker_tag_valid,
    remove_docker_container,
    remove_docker_image,
)
from rtwcli.verb import VerbExtension
from rtwcli.workspace_utils import (
    Workspace,
    WorkspacesConfig,
    load_workspaces_config_from_yaml_file,
    save_workspaces_config,
    workspace_name_completer,
)


class StepStatus(Enum):
    NOT_STARTED = auto()
    COMPLETED = auto()
    FAILED = auto()
    SKIPPED = auto()


class WorkspaceRemovalStatus(Enum):
    NOT_STARTED = auto()
    COMPLETED = auto()
    FAILED = auto()
    INTERRUPTED = auto()
    CAUGHT_EXCEPTION = auto()


class StepName(Enum):
    REMOVE_DOCKER_CONTAINER = "Remove Docker Container"
    REMOVE_DOCKER_IMAGE = "Remove Docker Image"
    REMOVE_WORKSPACE_FOLDER = "Remove Workspace Folder"
    REMOVE_CONFIGURATION = "Remove Configuration"


REMOVAL_STEPS = [
    StepName.REMOVE_DOCKER_CONTAINER,
    StepName.REMOVE_DOCKER_IMAGE,
    StepName.REMOVE_WORKSPACE_FOLDER,
    StepName.REMOVE_CONFIGURATION,
]


@dataclass
class StepStats:
    name: StepName
    status: StepStatus = StepStatus.NOT_STARTED
    message: str = ""


@dataclass
class WorkspaceRemovalStats:
    status: WorkspaceRemovalStatus = WorkspaceRemovalStatus.NOT_STARTED
    message: str = ""
    steps: List[StepStats] = field(default_factory=list)


class RemovalStep(ABC):
    def __init__(self, name: StepName):
        self.name = name
        self.status = StepStatus.NOT_STARTED
        self.message = ""

    @abstractmethod
    def should_run(self, workspace: Workspace) -> bool:
        pass

    @abstractmethod
    def run(self, workspace: Workspace, config: WorkspacesConfig) -> bool:
        pass

    def execute(self, workspace: Workspace, config: WorkspacesConfig) -> StepStats:
        if self.should_run(workspace):
            logger.info(f"Starting ws removal step: '{self.name.value}'")
            try:
                if self.run(workspace, config):
                    self.status = StepStatus.COMPLETED
                    logger.info(f"Successfully completed {self.name.value}: {self.message}")
                else:
                    self.status = StepStatus.FAILED
                    logger.error(f"Failed to complete {self.name.value}: {self.message}")
            except Exception as e:
                self.status = StepStatus.FAILED
                self.message = (
                    f"Exception caught in {self.name.value}: {str(e)}"
                    f"\nOriginal message: '{self.message}'"
                )
                logger.error(self.message)
                raise
        else:
            self.status = StepStatus.SKIPPED
            self.message = f"Skipping {self.name.value} (not applicable, .e.g. no Docker support)"
            logger.info(self.message)

        return StepStats(name=self.name, status=self.status, message=self.message)


class RemoveDockerContainer(RemovalStep):
    def __init__(self):
        super().__init__(StepName.REMOVE_DOCKER_CONTAINER)

    def should_run(self, workspace: Workspace) -> bool:
        return bool(workspace.ws_docker_support and workspace.docker_container_name)

    def run(self, workspace: Workspace, config: WorkspacesConfig) -> bool:
        if docker_container_exists(workspace.docker_container_name):
            if docker_stop(workspace.docker_container_name):
                logger.info(f"Stopped Docker container '{workspace.docker_container_name}'")
            else:
                self.message = (
                    f"Failed to stop Docker container '{workspace.docker_container_name}'"
                )
                return False

            if remove_docker_container(workspace.docker_container_name):
                self.message = f"Removed Docker container '{workspace.docker_container_name}'"
                return True
            else:
                self.message = (
                    f"Failed to remove Docker container '{workspace.docker_container_name}'"
                )
                return False
        else:
            self.message = (
                f"Container '{workspace.docker_container_name}' does not exist. Skipping removal."
            )
            return True


class RemoveDockerImage(RemovalStep):
    def __init__(self):
        super().__init__(StepName.REMOVE_DOCKER_IMAGE)

    def should_run(self, workspace: Workspace) -> bool:
        return bool(workspace.ws_docker_support and workspace.docker_tag)

    def run(self, workspace: Workspace, config: WorkspacesConfig) -> bool:
        if is_docker_tag_valid(workspace.docker_tag):
            if remove_docker_image(workspace.docker_tag):
                self.message = f"Removed Docker image '{workspace.docker_tag}'"
                return True
            else:
                self.message = f"Failed to remove Docker image '{workspace.docker_tag}'"
                return False
        else:
            self.message = (
                f"Docker image '{workspace.docker_tag}' does not exist. Skipping removal."
            )
            return True


class RemoveWorkspaceFolder(RemovalStep):
    def __init__(self):
        super().__init__(StepName.REMOVE_WORKSPACE_FOLDER)

    def should_run(self, workspace: Workspace) -> bool:
        return not workspace.standalone

    def run(self, workspace: Workspace, config: WorkspacesConfig) -> bool:
        if not os.path.exists(workspace.ws_folder):
            self.message = (
                f"Workspace folder '{workspace.ws_folder}' does not exist. Skipping removal."
            )
            return True

        # ask the user what should be removed, everything per default
        items_in_ws_folder = os.listdir(workspace.ws_folder)
        if not items_in_ws_folder:
            self.message = f"Nothing to remove in workspace folder '{workspace.ws_folder}'"
            return True

        item_paths = [os.path.join(workspace.ws_folder, item) for item in items_in_ws_folder]
        item_choices_to_delete = {
            f"{item} {os.listdir(item_path) if os.path.isdir(item_path) else ''}": item_path
            for item, item_path in zip(items_in_ws_folder, item_paths)
        }
        items_to_delete = questionary.checkbox(
            message=f"Select items to delete in workspace folder '{workspace.ws_folder}'\n",
            choices=list(item_choices_to_delete.keys()),
            style=questionary.Style([("highlighted", "bold")]),
        ).ask()
        if items_to_delete is None:  # cancelled by user
            self.message = f"User cancelled removal of workspace folder '{workspace.ws_folder}'"
            return False

        logger.debug(f"Items to delete: {items_to_delete}")

        item_paths_to_delete = [item_choices_to_delete[item] for item in items_to_delete]
        for item_path in item_paths_to_delete:
            logger.debug(f"Removing item: {item_path}")
            if os.path.isfile(item_path) or os.path.islink(item_path):
                os.unlink(item_path)
            elif os.path.isdir(item_path):
                shutil.rmtree(item_path)
            logger.debug(f"Removed item: {item_path}")

        self.message = f"Removed items: {item_paths_to_delete}"
        return True


class RemoveFromConfig(RemovalStep):
    def __init__(self):
        super().__init__(StepName.REMOVE_CONFIGURATION)

    def should_run(self, workspace: Workspace) -> bool:
        return True

    def run(self, workspace: Workspace, config: WorkspacesConfig) -> bool:
        if config.remove_workspace(workspace.ws_name):
            if save_workspaces_config(WORKSPACES_PATH, config):
                self.message = (
                    f"Removed workspace '{workspace.ws_name}' from config file '{WORKSPACES_PATH}'"
                )
                return True
            else:
                self.message = (
                    f"Failed to save config file '{WORKSPACES_PATH}' after removing workspace"
                )
                return False
        else:
            self.message = (
                f"Failed to remove workspace '{workspace.ws_name}' from workspace config"
            )
            return False


def get_removal_step(step_name: StepName) -> RemovalStep:
    step_classes = {
        StepName.REMOVE_DOCKER_CONTAINER: RemoveDockerContainer,
        StepName.REMOVE_DOCKER_IMAGE: RemoveDockerImage,
        StepName.REMOVE_WORKSPACE_FOLDER: RemoveWorkspaceFolder,
        StepName.REMOVE_CONFIGURATION: RemoveFromConfig,
    }
    return step_classes[step_name]()


def create_choice_format_string(ws_name_width, distro_width, ws_folder_width, num_spaces=2) -> str:
    return (
        f"{{ws_name:<{ws_name_width+num_spaces}}}"
        f" {{distro:<{distro_width+num_spaces}}}"
        f" {{ws_folder:<{ws_folder_width+num_spaces}}}"
        f" {{docker_tag}}"
    )


def get_choice_format_template(choice_format: str) -> str:
    return (
        "["
        + (
            "".join(c for c in choice_format if c.isalpha() or c in "_ {}")
            .replace("{", "<")
            .replace("}", ">")
        )
        + "]"
    )


def remove_workspaces(
    config: WorkspacesConfig, workspace_names: List[str]
) -> Dict[str, WorkspaceRemovalStats]:
    removal_stats: Dict[str, WorkspaceRemovalStats] = {
        ws_name: WorkspaceRemovalStats(status=WorkspaceRemovalStatus.NOT_STARTED)
        for ws_name in workspace_names
    }

    for ws_name in workspace_names:
        workspace = config.workspaces.get(ws_name)
        if not workspace:
            removal_stats[ws_name].status = WorkspaceRemovalStatus.FAILED
            removal_stats[ws_name].message = (
                f"Workspace '{ws_name}' not found in the configuration"
            )
            logger.error(removal_stats[ws_name].message)
            continue

        try:
            interrupted = False
            for step_name in REMOVAL_STEPS:
                step = get_removal_step(step_name)
                step_stats = step.execute(workspace, config)
                removal_stats[ws_name].steps.append(step_stats)
                if step_stats.status == StepStatus.FAILED:
                    removal_stats[ws_name].status = WorkspaceRemovalStatus.FAILED
                    removal_stats[ws_name].message = (
                        f"Failed to remove workspace '{ws_name}' during {step.name.value}"
                    )
                    interrupted = True
                    break
            if interrupted:
                logger.debug(f"Removal of workspace '{ws_name}' interrupted")
                break
        except KeyboardInterrupt:
            removal_stats[ws_name].status = WorkspaceRemovalStatus.INTERRUPTED
            removal_stats[ws_name].message = (
                f"Removal process for workspace '{ws_name}' interrupted by user"
            )
            logger.warning(removal_stats[ws_name].message)
            break
        except Exception as e:
            removal_stats[ws_name].status = WorkspaceRemovalStatus.CAUGHT_EXCEPTION
            removal_stats[ws_name].message = (
                f"Error during removal of workspace '{ws_name}': {str(e)}"
            )
            logger.exception(removal_stats[ws_name].message)
            break

        removal_stats[ws_name].status = WorkspaceRemovalStatus.COMPLETED
        removal_stats[ws_name].message = f"Successfully removed workspace '{ws_name}'"
        logger.info(removal_stats[ws_name].message)

    return removal_stats


def print_removal_stats(stats: Dict[str, WorkspaceRemovalStats]):
    total = len(stats)
    completed = sum(
        1 for ws_stats in stats.values() if ws_stats.status == WorkspaceRemovalStatus.COMPLETED
    )
    tree = Tree(
        f"[bold green]Workspace Removal Statistics: {completed}/{total}", guide_style="bold"
    )
    for ws_i, (ws_name, ws_stats) in enumerate(stats.items(), start=1):
        ws_node = tree.add(
            f"[bold blue]{ws_i}. {ws_name} - [bold magenta]{ws_stats.status.name}", highlight=True
        )
        for step_i, step in enumerate(ws_stats.steps, start=1):
            step_node = ws_node.add(
                f"[yellow]{step_i}. {step.name.value}: [bold magenta]{step.status.name}"
            )
            step_node.add(f"[cyan]{step.message}")
    rich.print(tree)


def get_ws_names_to_delete_from_user(workspaces_config: WorkspacesConfig) -> List[str]:
    logger.debug("ws selection")
    ws_name_width = max(len(ws_name) for ws_name in workspaces_config.workspaces.keys())
    distro_width = max(len(ws.distro) for ws in workspaces_config.workspaces.values())
    ws_folder_width = max(len(ws.ws_folder) for ws in workspaces_config.workspaces.values())
    choice_format = create_choice_format_string(ws_name_width, distro_width, ws_folder_width)

    choice_data = {}
    for ws_name, ws in workspaces_config.workspaces.items():
        ws_choice_data = choice_format.format(
            ws_name=ws_name,
            distro=ws.distro,
            ws_folder=ws.ws_folder,
            docker_tag=ws.docker_tag,
        )
        choice_data[ws_choice_data] = ws_name

    choice_format_template = get_choice_format_template(choice_format)
    ws_choices_to_delete = questionary.checkbox(
        message=f"Select workspaces to delete {choice_format_template}\n",
        choices=list(choice_data.keys()),
        style=questionary.Style([("highlighted", "bold")]),
    ).ask()
    if not ws_choices_to_delete:  # cancelled by user
        exit(0)

    logger.debug("ws selection confirmation")
    workspaces_to_delete_to_confirm = "\n".join(
        f"{i}. {ws_choice}" for i, ws_choice in enumerate(ws_choices_to_delete, start=1)
    )

    confirm_delete = questionary.confirm(
        f"Are you sure you want to delete following workspaces? {choice_format_template}"
        f"\n{workspaces_to_delete_to_confirm}\n"
    ).ask()
    if not confirm_delete:  # cancelled by user
        exit(0)

    ws_names_to_delete = [
        choice_data[ws_choice_to_delete] for ws_choice_to_delete in ws_choices_to_delete
    ]
    return ws_names_to_delete


class DeleteVerb(VerbExtension):
    """Delete an available ROS workspace in the config."""

    def add_arguments(self, parser: argparse.ArgumentParser, cli_name: str) -> None:
        arg = parser.add_argument(
            "workspace_name",
            help="The workspace name",
            nargs="?",
        )
        arg.completer = workspace_name_completer  # type: ignore

    def main(self, *, args):
        workspaces_config = load_workspaces_config_from_yaml_file(WORKSPACES_PATH)
        if not workspaces_config.workspaces:
            logger.info(f"No workspaces found in config file '{WORKSPACES_PATH}'")
            return

        if args.workspace_name:
            logger.debug(f"Deleting workspace from args: {args.workspace_name}")
            ws_names_to_delete = [args.workspace_name]
        else:
            ws_names_to_delete = get_ws_names_to_delete_from_user(workspaces_config)

        logger.debug(f"Ws names to delete: {ws_names_to_delete}")

        removal_stats = remove_workspaces(workspaces_config, ws_names_to_delete)
        print_removal_stats(removal_stats)
