#!/bin/bash
#
# Copyright 2021 Denis Stogl (Stogl Robotics Consulting)
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


usage='setup-ci-config.bash "repo_name" "repo_namespace" ["first_package second_package ..."]'
#

# Load Framework defines
script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $script_own_dir/../setup.bash
check_and_set_ros_distro_and_version ${ROS_DISTRO}

repo_name=$1
if [ -z "$1" ]; then
  print_and_exit "You should provide repository name!" "$usage"
fi

repo_namespace=$2
if [ -z "$2" ]; then
  print_and_exit "You should provide repository namespace!" "$usage"
fi

package_names=$3
if [ -z "$3" ]; then
  echo -e "${TERMINAL_COLOR_YELLOW}Packages should be added manually because none is defined${TERMINAL_COLOR_NC}"
  package_names="<none>"
fi

read -p "${RAW_TERMINAL_COLOR_BROWN}Are you setting CI for private repository?${RAW_TERMINAL_COLOR_NC} (yes/no) [no]: " private
private=${private:="no"}

CI_FORMAT="ci-format"
if  [[ "$private" == "yes" ]]; then
  CI_FORMAT="ci-format-private"
fi

# File-suffix definitions
CI_ROS_LINT="ci-ros-lint"
CI_COVERAGE="ci-coverage-build"
CI_REUSABLE_INDUSTRIAL_CI="reusable-industrial-ci-with-cache"
CI_REUSABLE_ROS_TOOLING="reusable-ros-tooling-source-build"
PRE_RELEASE="prerelease-check"

CI_ABI_COMPATIBILITY="abi-compatibility"
CI_BINARY_BUILD_MAIN="binary-build-main"
CI_BINARY_BUILD_TESTING="binary-build-testing"
CI_SEMI_BINARY_BUILD_MAIN="semi-binary-build-main"
CI_SEMI_BINARY_BUILD_TESTING="semi-binary-build-testing"
CI_SOURCE_BUILD="source-build"
CI_RHEL_BINARY_BUILD="rhel-binary-build"


# Lists for different generation stages
CI_GENERAL_FILES=(
  $CI_FORMAT
  $CI_ROS_LINT
  $CI_COVERAGE
  $CI_REUSABLE_INDUSTRIAL_CI
  $CI_REUSABLE_ROS_TOOLING
  $PRE_RELEASE
)

CI_DISTRIBUTION_FILES=(
  $CI_ABI_COMPATIBILITY
  $CI_BINARY_BUILD_MAIN
  $CI_BINARY_BUILD_TESTING
  $CI_SEMI_BINARY_BUILD_MAIN
  $CI_SEMI_BINARY_BUILD_TESTING
  $CI_SOURCE_BUILD
  $CI_RHEL_BINARY_BUILD
)

read -p "${RAW_TERMINAL_COLOR_BROWN}Name of the default ROS distro?${RAW_TERMINAL_COLOR_NC} [default: rolling] " default_ros_distro
default_ros_distro=${default_ros_distro:="rolling"}
read -p "${RAW_TERMINAL_COLOR_BROWN}Name of the repository default branch?${RAW_TERMINAL_COLOR_NC} [default: master]: " default_branch
default_branch=${default_branch:="master"}

# Create workflow folder if it does not exists
mkdir -p .github/workflows

# Coping and SEDing general CI files
for CI_FILE in "${CI_GENERAL_FILES[@]}"; do
  cp -n ${PACKAGE_TEMPLATES}/CI-github_${CI_FILE}.yml .github/workflows/${CI_FILE}.yml
  sed -i 's/\$ros_distro\$/'${default_ros_distro}'/g' .github/workflows/${CI_FILE}.yml
  sed -i 's/\$branch\$/'${default_branch}'/g' .github/workflows/${CI_FILE}.yml
  sed -i 's/\$NAME\$/'${repo_name}'/g' .github/workflows/${CI_FILE}.yml
  if [[ "$package_names" != "<none>" ]]; then
    sed -i 's/\$PKG_NAME\$//g' .github/workflows/${CI_FILE}.yml
    for PKG_NAME in ${package_names}; do
      append_to_string='          package-name:'
      sed -i "s/$append_to_string/$append_to_string\\n            ${PKG_NAME}/g" .github/workflows/${CI_FILE}.yml
      if ! grep -q ${PKG_NAME} .github/workflows/${CI_FILE}.yml; then
        append_to_string='        package-name:'
        sed -i "s/$append_to_string/$append_to_string\\n          ${PKG_NAME}/g" .github/workflows/${CI_FILE}.yml
      fi
    done
  fi

done


some_ros_distro=${default_ros_distro}
some_branch=${default_branch}

TMP_FILE=".f_tmp"
TMP_FILE_2=".f_tmp2"
touch $TMP_FILE
touch $TMP_FILE_2

while true; do
  for CI_FILE in "${CI_DISTRIBUTION_FILES[@]}"; do
    output_file_name="${some_ros_distro}-${CI_FILE}"
    cp -n ${PACKAGE_TEMPLATES}/CI-github_${CI_FILE}.yml .github/workflows/${output_file_name}.yml
    sed -i 's/\$ros_distro\$/'${some_ros_distro}'/g' .github/workflows/${output_file_name}.yml
    sed -i 's/\$Ros_distro\$/'${some_ros_distro^}'/g' .github/workflows/${output_file_name}.yml
    sed -i 's/\$branch\$/'${some_branch}'/g' .github/workflows/${output_file_name}.yml
    sed -i 's/\$NAME\$/'${repo_name}'/g' .github/workflows/${output_file_name}.yml
    if [[ "$package_names" != "<none>" ]]; then
      sed -i 's/\$PKG_NAME\$//g' .github/workflows/${output_file_name}.yml
      for PKG_NAME in ${package_names}; do
        append_to_string='          package-name:'
        sed -i "s/$append_to_string/$append_to_string\\n            ${PKG_NAME}/g" .github/workflows/${output_file_name}.yml
        if ! grep -q ${PKG_NAME} .github/workflows/${output_file_name}.yml; then
          append_to_string='        package-name:'
          sed -i "s/$append_to_string/$append_to_string\\n          ${PKG_NAME}/g" .github/workflows/${output_file_name}.yml
        fi
      done
    fi
  done
  cp -n ${PACKAGE_TEMPLATES}/pkg_name.repos ${repo_name}-not-released.${some_ros_distro}.repos
  cp -n ${PACKAGE_TEMPLATES}/pkg_name.repos ${repo_name}.${some_ros_distro}.repos

  cp -n ${PACKAGE_TEMPLATES}/README.md.github README.md

  if ! grep -q "## Build status" README.md; then
    cat ${PACKAGE_TEMPLATES}/_append_to_README_build_status.md >> README.md
  fi

  # Get line with "### Explanation of different build types"
  TEST_LINE=`awk '$1 == "###" && $2 == "Explanation" && $5 == "build" { print NR }' README.md`
  let CUT_LINE=$TEST_LINE-1
  head -$CUT_LINE README.md >> $TMP_FILE
  cat ${PACKAGE_TEMPLATES}/_append_to_README_build_status_table.md >> $TMP_FILE

  # Update .github/workflows/README.md file
  cat ${PACKAGE_TEMPLATES}/_append_to_workflows_README_build_status_table.md > $TMP_FILE_2

  # SED temp files
  FILES_TO_UPDATE=("$TMP_FILE" "$TMP_FILE_2")
  for SED_FILE in "${FILES_TO_UPDATE[@]}"; do
    sed -i 's/\$branch\$/'${some_branch}'/g' $SED_FILE
    sed -i 's/\$ros_distro\$/'${some_ros_distro}'/g' $SED_FILE
    sed -i 's/\$Ros_distro\$/'${some_ros_distro^}'/g' $SED_FILE
    sed -i 's/\$NAME\$/'${repo_name}'/g' $SED_FILE
    sed -i 's/\$NAMESPACE\$/'${repo_namespace}'/g' $SED_FILE
  done

  # append to readme files
  tail -n +$CUT_LINE README.md >> $TMP_FILE
  mv $TMP_FILE README.md

  cat $TMP_FILE_2 >> .github/workflows/README.md
  rm $TMP_FILE_2

  echo ""
  echo -e "${TERMINAL_COLOR_BLUE}Added setup for ros distro '${RAW_TERMINAL_COLOR_BROWN}${some_ros_distro}${TERMINAL_COLOR_BLUE}' on branch '${RAW_TERMINAL_COLOR_BROWN}${some_branch}${TERMINAL_COLOR_BLUE}'.${TERMINAL_COLOR_NC}"
  echo ""

  read -p "${RAW_TERMINAL_COLOR_BROWN}Do you want to configure ci for another ros distro?${RAW_TERMINAL_COLOR_NC} (yes/no) [no] " another
  another=${another:="no"}

  if [[ "$another" == "no" ]]; then
    break
  fi

  echo "" > $TMP_FILE
  echo "" > $TMP_FILE_2

  read -p "${RAW_TERMINAL_COLOR_BROWN}Name of the ROS distro? ${RAW_TERMINAL_COLOR_NC}" some_ros_distro
  read -p "${RAW_TERMINAL_COLOR_BROWN}Name of the repository branch?: ${RAW_TERMINAL_COLOR_NC}" some_branch
done

# Setting up formatting
read -p "${RAW_TERMINAL_COLOR_BROWN}Do you want to setup formatting using pre-commit?${RAW_TERMINAL_COLOR_NC} (yes/no) [no]: " formatting
formatting=${formatting:="no"}

if  [[ "$formatting" == "yes" ]]; then
  cp -n ${PACKAGE_TEMPLATES}/.clang-format .
  cp -n ${PACKAGE_TEMPLATES}/.pre-commit-config.yaml .
  pre-commit install
  pre-commit autoupdate
fi

echo ""
echo -e "${TERMINAL_COLOR_BLUE}FINISHED: Please check the generated file and setup package names.${TERMINAL_COLOR_NC}"
