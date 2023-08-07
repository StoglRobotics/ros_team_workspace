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

setup(
    name="rtwcli",
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    extras_require={
        "completion": ["argcomplete"],
    },
    zip_safe=False,
    keywords=[],
    classifiers=[
        "Environment :: Console",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
    ],
    description="Framework for ROS Team Workspace command line tools.",
    long_description="""\
The framework provides a single command line script which can be extended with
commands and verbs.""",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "rtwcli.command": [
            "extension_points =" " rtwcli.command.extension_points:ExtensionPointsCommand",
            "extensions = rtwcli.command.extensions:ExtensionsCommand",
            "info = rtwcli.command.info:InfoCommand",
        ],
        "rtwcli.extension_point": [
            "rtwcli.command = rtwcli.command:CommandExtension",
        ],
        "console_scripts": [
            "rtw = rtwcli.cli:main",
        ],
    },
)
