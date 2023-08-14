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

from setuptools import find_packages
from setuptools import setup

package_name = "rtw_cmds"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    install_requires=["questionary", "rtwcli"],
    zip_safe=True,
    keywords=[],
    classifiers=[
        "Environment :: Console",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
    ],
    description="The commands for ROS Team Workspace command line tools.",
    long_description="""\
The package provides the commands for the ROS Team Workspace command line tools.""",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "rtwcli.command": [
            "docker = rtw_cmds.commands:DockerCommand",
            "pkg = rtw_cmds.commands:PkgCommand",
            "workspace = rtw_cmds.commands:WorkspaceCommand",
            "ws = rtw_cmds.aliases:WSAlias",
        ],
        "rtwcli.extension_point": [
            "rtw_cmds.docker.verbs = rtwcli.verb:VerbExtension",
            "rtw_cmds.pkg.verbs = rtwcli.verb:VerbExtension",
            "rtw_cmds.workspace.verbs = rtwcli.verb:VerbExtension",
        ],
        "rtw_cmds.docker.verbs": [
            "enter = rtw_cmds.docker.verbs:EnterVerb",
        ],
        "rtw_cmds.pkg.verbs": [
            "create = rtw_cmds.pkg.verbs:CreateVerb",
        ],
        "rtw_cmds.workspace.verbs": [
            "create = rtw_cmds.workspace.verbs:CreateVerb",
            "port = rtw_cmds.workspace.verbs:PortVerb",
            "port_all = rtw_cmds.workspace.verbs:PortAllVerb",
            "use = rtw_cmds.workspace.verbs:UseVerb",
        ],
    },
)
