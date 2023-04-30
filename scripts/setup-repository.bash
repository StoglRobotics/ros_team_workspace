#!/bin/bash
#
# Copyright 2021 Stogl Denis Stogl (Stogl Robotics Consulting)
# Copyright 2017-2020 Denis Stogl (Institute for Anthropomatics and Robotics (IAR) -
#  Intelligent Process Control and Robotics (IPR) of Karlsruhe Institute of Technology (KIT))
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


usage='setup-repository.bash PKG_NAME PKG_DESCRIPTION LICENSE'
#

echo ""
echo "Your path is `pwd`. Is this your package folder to setup repository?"
read -p "If so press <ENTER> otherwise <CTRL>+C and start the script again from your source folder."

# Load Framework defines
script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $script_own_dir/../setup.bash
check_and_set_ros_distro_and_version ${ROS_DISTRO}

PKG_NAME=$1
if [ -z "$1" ]; then
  echo "You should provide package name!"
  exit 0
fi

PKG_DESCRIPTION=$2
if [ -z "$2" ]; then
  echo "You should provide package description!"
  exit 0
fi

LICENSE=$3
if [ -z "$LICENSE" ]; then
  echo "No package LICENSE defined! Using 'Propriatery' as default."
  exit 0
fi


git init
git checkout -b $ros_distro
git add .
git commit -m "RosTeamWS: package created with initial files"

# TODO: Check if a user want to configure CI
# Build type
echo "Which type of repository server are you using [1]:"
echo "(1) GitHub"
echo "(2) Gitlab (not available yet - TBD)"
read choice
choice=${choice:=1}

#TODO: Add automatically git remote

repository=""
case "$choice" in
"1")
   mkdir -p .github/workflows
   CI_FILES_TO_COPY_AND_SED=("ci-build" "ci-format") # TODO(denis): Check if also "ci-lint" ?

   for CI_FILE in "${CI_FILES_TO_COPY_AND_SED[@]}"; do
     cp -n $PACKAGE_TEMPLATES/CI-github_${CI_FILE}.yml .github/workflows/${CI_FILE}.yml
     sed -i 's/\$NAME\$/'${PKG_NAME}'/g' .github/workflows/${CI_FILE}.yml
     sed -i 's/\$ROS_DISTRO\$/'${ros_distro}'/g' .github/workflows/${CI_FILE}.yml
   done

   cp -n $PACKAGE_TEMPLATES/pkg_name.repos $PKG_NAME.repos
   ln -s $PKG_NAME.repos $PKG_NAME.ci.repos
   echo "NOTE: To enable CI from source, uncomment it manually in '.github/workflows/ci-build.yml'"
   cp -n $PACKAGE_TEMPLATES/README.md.github README.md
   repository="github"
   ;;
"2")
  cp -n $PACKAGE_TEMPLATES/.gitlab-ci.yml .
  cp -n $PACKAGE_TEMPLATES/.ci.repos .
  repository="gitlab"
   ;;
*)
  echo "Invalid input! Exiting..."
  exit 0
esac

cp -n $PACKAGE_TEMPLATES/.clang-format .
cp -n $PACKAGE_TEMPLATES/.pre-commit-config.yaml .
pre-commit install
pre-commit autoupdate


# This functionality is not provided in all framework versions
if [[ -f "$PACKAGE_TEMPLATES/_append_to_README_ROS_Intro.md" ]]; then
  # Ask if add How-to-use and ROS-Intro
  read -p "Do you want to append description on 'How-to-use and ROS-Intro' to the README? (y/n) [n]" choice
  choice=${choice="n"}

  case "$choice" in
  "y")
    cat $PACKAGE_TEMPLATES/_append_to_README_ROS_Intro.md >> README.md
    echo "Description is appended."
    ;;
  "n")
    echo "Description not appended."
  esac
fi

read -p "Enter namespace of the repository [default: $PKG_NAME]: " NAMESPACE
NAMESPACE=${NAMESPACE:=$PKG_NAME}

license_web="${LICENSE// /%20}"
sed -i 's/\$NAME\$/'${PKG_NAME}'/g' README.md
sed -i 's/\$ROS_DISTRO\$/'${ros_distro}'/g' README.md
sed -i 's/\$Ros_distro\$/'${ros_distro^}'/g' README.md
sed -i 's/\$DESCRIPTION\$/'"${PKG_DESCRIPTION}"'/g' README.md
sed -i 's/\$LICENSE\$/'${license_web}'/g' README.md
sed -i 's/\$NAMESPACE\$/'${NAMESPACE}'/g' README.md

# Check if it is metapackage
if [[ ! -f "package.xml" ]]; then
  echo "" >> README.md
  echo "" >> README.md
  echo "### Packages in \`${PKG_NAME}\` metapackage" >> README.md
  echo "" >> README.md
  echo "" >> README.md
fi

git add .
git commit -m "RosTeamWS: added CI configuration"


read -p "Does repository hold open source project (y/n) [n]: " open_source
open_source=${open_source="n"}

case "$open_source" in
"y")
  # TODO: Add here License choice
  license="Apache-2.0"
  echo "Adding 'LICENSE' file for '$license' license."
  # TODO: maybe use "-i,  --input-file=DATEI  in local or external FILE" option?
  wget -O LICENSE https://www.apache.org/licenses/LICENSE-2.0.txt
  # TODO: Add contributing file
#   OS-Apache-CONTRIBUTING.md
  ;;
"n")
  echo "Not open source repository"
esac

read -p "Does repository hold documentation (y/n) [n]: " documentation
documentation=${documentation="n"}

case "$documentation" in
"y")
  DOCS_FOLDER='docs'
  mkdir $DOCS_FOLDER
  cp -n $PACKAGE_TEMPLATES/CI-github_docs-sphinx-build-check.yml .github/workflows/docs-sphinx-build-check.yml
  sed -i 's/\$DOCS_FOLDER\$/'${DOCS_FOLDER}'/g' .github/workflows/docs-sphinx-build-check.yml
  cp -n $PACKAGE_TEMPLATES/CI-github_docs-sphinx-make-page.yml .github/workflows/docs-sphinx-make-page.yml
  sed -i 's/\$DOCS_FOLDER\$/'${DOCS_FOLDER}'/g' .github/workflows/docs-sphinx-make-page.yml

  cd $DOCS_FOLDER
  sphinx-quickstart

  # Use RTD template. https://sphinx-tutorial.readthedocs.io/start/
  ;;
"n")
  echo "Docs folder not added"
esac


echo ""
echo "FINISHED: Please create $repository repository manually on $TEAM_REPOSITORY_SERVER/$NAMESPACE/$PKG_NAME add follow the explanation to push existing repository from command line."
