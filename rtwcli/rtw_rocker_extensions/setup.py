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
    name="rtw_rocker_extensions",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    install_requires=["rocker"],
    extras_require={},
    zip_safe=False,
    keywords=[],
    classifiers=[
        "Environment :: Console",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
    ],
    description="Extra plugins for rocker.",
    long_description="Extra plugins for rocker",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "rocker.extensions": [
            "rtw_x11tmp = rtw_rocker_extensions.x11tmp:X11Tmp",
        ],
    },
)
