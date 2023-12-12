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

import argparse
from rtwcli.command import CommandExtension
from rtw_cmds.workspace.verbs import UseVerb, add_cli_workspace_arg


class WSAlias(CommandExtension):
    """ALIAS for 'rtw workspace use'."""

    def __init__(self):
        super().__init__()

    def add_arguments(self, parser: argparse.ArgumentParser, cli_name: str):
        add_cli_workspace_arg(parser)

    def main(self, *, parser, args):
        use_verb = UseVerb()
        return use_verb.main(args=args)
