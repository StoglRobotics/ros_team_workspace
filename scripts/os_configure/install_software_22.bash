#!/usr/bin/env bash
# Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

# install_software_22.bash [computer_type:{office, robot, default:basic}]

script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
SCRIPT_PATH=$script_own_dir
source "${SCRIPT_PATH}"/../../setup.bash

if [ -z "$supported_computer_types" ]; then
  readonly supported_computer_types=("basic" "office" "robot")
fi

computer_type=$1
# check if given computer_type is supported. If this is not the case, then inform user about supported types and
# let user chose which typ he would like.
if ! [[ " ${supported_computer_types[*]} " =~ " ${computer_type} " ]]; then
  notify_user "The computer type ${computer_type} you have given is not supported."
  echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION} Chose one of the following:"
  echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION} basic    -Standard pc. Basic utilities like development utilities (git, vscode, ...) and some additional tools."
  echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION} office   -Used for office pcs. Basic utilities, additional tools and office related stuff."
  echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION} robot    -Used for robot platforms. Basic utilities like development utilities (git, vscode, ...)."
  select computer_type in basic office robot;
  do
    case "$computer_type" in
          basic)
              computer_type="basic"
              break
            ;;
          office)
              computer_type="office"
              break
            ;;
          robot)
              computer_type="robot"
              break
            ;;
    esac
  done
  echo -n -e "${TERMINAL_COLOR_NC}"
fi

echo "Installing software for $computer_type computers. Press <ENTER> to continue."
read

ROS2_VERSIONS=( "humble" "rolling" )

### CORE TOOLS ###
sudo apt update
sudo apt -y install ca-certificates curl gnupg gnupg2 lsb-release

# Nala - better apt frontend
echo "deb http://deb.volian.org/volian/ scar main" | sudo tee /etc/apt/sources.list.d/volian-archive-scar-unstable.list
wget -qO - https://deb.volian.org/volian/scar.key | sudo tee /etc/apt/trusted.gpg.d/volian-archive-scar-unstable.gpg > /dev/null
sudo apt update
sudo apt -y install nala
nala --install-completion bash

# git stable upstream version
sudo add-apt-repository -y ppa:git-core/ppa
sudo apt update

### BASIC TOOLS ###
sudo apt -y install neovim ssh git qgit trash-cli htop unrar yakuake screen finger ksshaskpass kompare filelight tldr thefuck ranger tree pre-commit

# Useful libraries
sudo apt -y install libxml2-dev libvlc-dev libmuparser-dev libudev-dev

### DEVELOPMENT TOOLS ###
# gh - Github CLI (it seems that gh needs to be install via apt to get access to .ssh keys)
sudo apt -y install gh
gh completion -s bash | tee "$HOME"/.local/share/bash-completion/completions/gh.bash > /dev/null

# visual studio code
sudo snap install --classic code
# install all plugins for visual studio code
vs_code_plugin_file=$RosTeamWS_FRAMEWORK_OS_CONFIGURE_PATH/vs-code_plugins.txt
while read extension; do
  # ignore empty lines or lines starting with "#"
  [[ $extension =~ ^#.* ]] || [ -z "$extension" ] && continue
  code --install-extension "${extension}"
done < "${vs_code_plugin_file}"

# Docker
sudo apt-get update
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt -y install docker-ce docker-ce-cli containerd.io docker-compose-plugin
sudo groupadd docker
sudo usermod -aG docker "$(whoami)"

# VirtualBox
sudo apt -y install virtualbox dkms virtualbox-guest-utils virtualbox-ext-pack

## ROS
# ROS2 Packages
for ROS2_VERSION in "${ROS2_VERSIONS[@]}"
do
  bash $SCRIPT_PATH/install_software_ros2.bash $ROS2_VERSION
done

sudo rosdep init
rosdep update

# ! They need to come after ros installation !
# Python tools
sudo apt -y install python3-pip \
  python3-colcon-common-extensions \
  python3-colcon-cd \
  python3-colcon-argcomplete \
  python3-flake8 \
  python3-flake8-blind-except \
  python3-flake8-builtins \
  python3-flake8-class-newline \
  python3-flake8-comprehensions \
  python3-flake8-deprecated \
  python3-flake8-docstrings \
  python3-flake8-import-order \
  python3-flake8-quotes \
  python3-pytest \
  python3-pytest-cov \
  python3-pytest-repeat \
  python3-pytest-rerunfailures \
  python3-rosdep \
  python3-rosdep2 \
  python3-setuptools \
  python3-vcstool \
  python3-pip \
  python3-virtualenv \
  python3-virtualenvwrapper \
  python3-notebook \
pip3 install --upgrade pip

# setup bash
cat "$OS_CONFIGURE_TEMPLATES/extend_to_bashrc" >> "$HOME/.bashrc"
cat "$OS_CONFIGURE_TEMPLATES/extend_to_bash_aliases" >> "$HOME/.bash_aliases"
cat "$OS_CONFIGURE_TEMPLATES/extend_to_bash_commands" >> "$HOME/.bash_commands"

# setup git
commit_template_path="$HOME/.config/git"
template_name="commit-template.txt"
mkdir -p "$commit_template_path"
cp "$OS_CONFIGURE_TEMPLATES/$template_name" "$commit_template_path/."
git config --global core.editor "vim"
git config --global commit.template "$commit_template_path/$template_name"

### NONE DEVELOPMENT RELATED TOOLS ###

# Dolphin Plugins
sudo apt -y install kdesdk-scripts

# Nextcloud
sudo apt -y install nextcloud-desktop

### BACKPORTS ###
# KDE Backports
sudo apt-add-repository -y ppa:kubuntu-ppa/backports
sudo apt update
sudo apt -y dist-upgrade
sudo apt -y autoremove

########################## END BASIC SETUP ##########################

if [[ "${computer_type}" != "robot" ]]
then
  sudo apt -y install recordmydesktop peek rdesktop gimp meshlab inkscape pdfposter unrar
fi

if [[ "${computer_type}" == "office" ]]
then

  ### CRYPTOPGRAPHY AND PASSWORD MANAGEMENT ###
  # Cryptography
  sudo apt -y install kleopatra scdaemon
  #  Passwordmanager
  sudo apt -y install pass

  ### NETWORKING ###
  # Network manager
  sudo apt -y install network-manager-openvpn network-manager-vpnc network-manager-ssh network-manager-openconnect

  # Remote desktop
  sudo apt -y install krdc

  ##  Network monitoring and debugging
  # Wireshark
  sudo DEBIAN_FRONTEND=noninteractive apt -y install wireshark
  # debugging
  sudo apt install nethogs nload net-tools

  ### ADDITIONAL DEVELOPMENT TOOLS ###
  # Code optimization and profiling
  sudo apt -y install valgrind kcachegrind hotspot heaptrack-gui
  sudo apt -y install linux-tools-generic linux-cloud-tools-generic  # Kernel tools

  sudo apt -y install flac

  sudo add-apt-repository -y ppa:freecad-maintainers/freecad-daily
  sudo apt update
  sudo apt -y install freecad-daily spacenavd

  # Tracing
  sudo apt -y install lttng-tools lttng-modules-dkms liblttng-ust-dev
  sudo apt -y install python3-babeltrace python3-lttng python3-lttngust
  sudo usermod -aG tracing "$(whoami)"

  ### MANAGE PERSONAL INFORMATION
  # KDE-PIM
  sudo apt -y install kontact korganizer kmail kjots kaddressbook kdepim*

  ### OFFICE TOOLS ###
  # Notes-taking
  sudo add-apt-repository ppa:pbek/qownnotes
  sudo apt update
  sudo apt -y install qownnotes

  ### LANGUAGES ###
  # Language packs
  sudo apt -y install language-pack-de language-pack-de-base language-pack-kde-de aspell-de hunspell-de-de hyphen-de wogerman
  sudo apt -y install language-pack-hr language-pack-hr-base language-pack-kde-hr aspell-hr hunspell-hr hyphen-hr

fi

sudo apt update
sudo apt -y dist-upgrade
sudo apt -y autoremove
# log is created somehow
trash "${RosTeamWS_FRAMEWORK_OS_CONFIGURE_PATH}/log/"
