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

from rtwcli.command import add_subparsers_on_demand
from rtwcli.command import CommandExtension


class BaseCommand(CommandExtension):
    def __init__(self, verbs_group):
        super().__init__()
        self.verbs_group = verbs_group

    def add_arguments(self, parser, cli_name):
        self._subparser = parser
        # add arguments and sub-commands of verbs
        add_subparsers_on_demand(parser, cli_name, "_verb", self.verbs_group, required=False)

    def main(self, *, parser, args):
        if not hasattr(args, "_verb"):
            # in case no verb was passed
            self._subparser.print_help()
            return 0

        extension = getattr(args, "_verb")

        # call the verb's main method
        return extension.main(args=args)


class DockerCommand(BaseCommand):
    """Various Docker related sub-commands."""

    def __init__(self):
        super().__init__("rtw_cmds.docker.verbs")


class PkgCommand(BaseCommand):
    """Various package related sub-commands."""

    def __init__(self):
        super().__init__("rtw_cmds.pkg.verbs")


class WorkspaceCommand(BaseCommand):
    """Various workspace related sub-commands."""

    def __init__(self):
        super().__init__("rtw_cmds.workspace.verbs")
