#!/bin/bash
#
# Copyright 2023 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#
# Author: Dr. Denis Stogl


usage='setup-formatting.bash'

# Load Framework defines
script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $script_own_dir/../setup.bash

# Setting up formatting
cp -n ${PACKAGE_TEMPLATES}/.clang-format .
cp -n ${PACKAGE_TEMPLATES}/.pre-commit-config.yaml .
cp -n ${PACKAGE_TEMPLATES}/.codespell-ignore-words.txt .
pre-commit install
pre-commit autoupdate

echo ""
echo -e "${TERMINAL_COLOR_BLUE}FINISHED formatting setup: Now commit the formatting configuration and format all files using \`pre-commit run -a\` command.${TERMINAL_COLOR_NC}"
