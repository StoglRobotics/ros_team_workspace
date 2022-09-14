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

# install_software_20.bash [computer_type:{office, robot, default:basic}]

script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
SCRIPT_PATH=$script_own_dir
source "$SCRIPT_PATH"/../../setup.bash

if [ -z "$supported_computer_types" ]; then
  readonly supported_computer_types=("basic" "office" "robot")
fi

computer_type=$1
# check if given computer_type is supported. If this is not the case, then inform user about supported types and
# let user chose which typ he would like.
if ! [[ " ${supported_computer_types[*]} " =~ " ${computer_type} " ]]; then
  notify_user "The computer type ${computer_type} you have given is not supported."
  echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION} Chose one of the following:"
  echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION} basic    -TODO(Add description)normal standard pc. Basic utilities like: TODO are installed"
  echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION} office   -TODO(Add description)pc used for "
  echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION} robot    -TODO(Add description)"
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

ROS_VERSION=melodic
ROS2_VERSIONS=( "galactc" "rolling" )

# Core tools
sudo apt update && sudo apt -y install ca-certificates curl gnupg2 lsb-release

# KDE Backports
sudo apt-add-repository -y ppa:kubuntu-ppa/backports
sudo apt update && sudo apt -y dist-upgrade && sudo apt -y autoremove

# Nala - better apt frontend
echo "deb http://deb.volian.org/volian/ scar main" | sudo tee /etc/apt/sources.list.d/volian-archive-scar-unstable.list
wget -qO - https://deb.volian.org/volian/scar.key | sudo tee /etc/apt/trusted.gpg.d/volian-archive-scar-unstable.gpg > /dev/null
sudo apt update
sudo apt -y install nala
nala --install-completion bash

# Dolphin Plugins
sudo apt -y install kdesdk-kio-plugins kdesdk-scripts

## Useful tools
sudo apt -y install vim ssh git qgit trash-cli htop unrar yakuake screen finger ksshaskpass kompare filelight tldr thefuck ranger

# Python tools
sudo apt -y install python3-pip
sudo pip3 install --upgrade pip
sudo pip3 install pre-commit virtualenv virtualenvwrapper notebook

# Useful libraries
sudo apt -y install libxml2-dev libvlc-dev libmuparser-dev libudev-dev

## Development tools
# gh - Github CLI
curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg \
&& sudo chmod go+r /usr/share/keyrings/githubcli-archive-keyring.gpg \
&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null \
&& sudo apt update \
&& sudo apt -y  install gh
gh completion -s bash | tee "$HOME"/.local/share/bash-completion/completions/gh.bash > /dev/null

# visual studio code
sudo snap install --classic code

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

# ROS Packages
bash $SCRIPT_PATH/install_software_ros.bash $ROS_VERSION

# ROS2 Packages
for ROS2_VERSION in "${ROS2_VERSIONS[@]}"
do
  bash $SCRIPT_PATH/install_software_ros2.bash $ROS2_VERSION
done

sudo rosdep init
rosdep update

if [[ $computer_type != "robot" ]]
then
  sudo apt -y install recordmydesktop rdesktop gimp meshlab inkscape pdfposter unrar
  sudo DEBIAN_FRONTEND=noninteractive apt -y install wireshark
fi

if [[ $computer_type == "office" ]]
then

  # Latex
  sudo apt -y install kile texlive-full texlive-lang-german kbibtex ktikz

  sudo apt -y install pass

  # Network manager
  sudo apt -y install network-manager-openvpn network-manager-vpnc network-manager-ssh network-manager-openconnect

  # Cryptography
  sudo apt -y install kleopatra scdaemon

  # Code optimization and profiling
  sudo apt -y install valgrind kcachegrind hotspot heaptrack-gui
  sudo apt -y install linux-tools-generic linux-cloud-tools-generic  # Kernel tools

  sudo apt -y install flac

  sudo add-apt-repository -y ppa:freecad-maintainers/freecad-daily
  sudo apt update
  sudo apt -y install freecad-daily spacenavd

  # Nextcloud
  sudo apt -y install nextcloud-desktop

  # VirtualBox
  sudo sh -c 'echo "deb [arch=amd64] https://download.virtualbox.org/virtualbox/debian $(lsb_release -sc) contrib" > /etc/apt/sources.list.d/virtualbox.list'
  wget -q https://www.virtualbox.org/download/oracle_vbox_2016.asc -O- | sudo apt-key add -
  sudo apt update
  sudo apt -y install virtualbox-6.1 dkms virtualbox-guest-utils virtualbox-ext-pack

  ## Messgengers

  # Telegram
  sudo add-apt-repository ppa:atareao/telegram
  sudo apt update
  sudo apt -y install -f telegram

  # Hamsket
  sudo apt -y install libappindicator3-1
  wget https://github.com/TheGoddessInari/hamsket/releases/download/0.6.2/hamsket_0.6.2_amd64.deb
  sudo dpkg -i hamsket_0.6.2_amd64.deb
  rm hamsket_0.6.2_amd64.deb

  # Spotify
  curl -sS https://download.spotify.com/debian/pubkey_0D811D58.gpg | sudo apt-key add -
  echo "deb http://repository.spotify.com stable non-free" | sudo tee /etc/apt/sources.list.d/spotify.list
  sudo apt update
  sudo apt -y install spotify-client

  # Noson
  sudo add-apt-repository ppa:jlbarriere68/noson-app
  sudo apt update
  sudo apt -y install noson-app

  # KDE-PIM
  sudo apt -y install kontact korganizer kmail kjots kaddressbook kdepim*

  # Tools
  sudo apt -y install krdc

  # Notes-taking
  sudo add-apt-repository ppa:pbek/qownnotes
  sudo apt update
  sudo apt -y install qownnotes

  # Language packs
  sudo apt -y install language-pack-de language-pack-de-base language-pack-kde-de aspell-de hunspell-de-de hyphen-de wogerman
  sudo apt -y install language-pack-hr language-pack-hr-base language-pack-kde-hr aspell-hr hunspell-hr hyphen-hr

  ## Development Tools

  # Tracing
  sudo apt-add-repository ppa:lttng/stable-2.12
  sudo apt update
  sudo apt -y install lttng-tools lttng-modules-dkms liblttng-ust-dev
  sudo apt -y install python3-babeltrace python3-lttng python3-lttngust
  sudo usermod -aG tracing "$(whoami)"

  # Tuxedo repositories
  sudo sh -c 'echo "deb https://deb.tuxedocomputers.com/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/tuxedocomputers.list'
  sudo sh -c 'echo "deb https://oibaf.tuxedocomputers.com/ubuntu $(lsb_release -sc) main" >> /etc/apt/sources.list.d/tuxedocomputers.list'
  sudo sh -c 'echo "deb https://graphics.tuxedocomputers.com/ubuntu $(lsb_release -sc) main" >> /etc/apt/sources.list.d/tuxedocomputers.list'
  sudo sh -c 'echo "deb https://kernel.tuxedocomputers.com/ubuntu $(lsb_release -sc) main" >> /etc/apt/sources.list.d/tuxedocomputers.list'
  wget -O - http://deb.tuxedocomputers.com/0x54840598.pub.asc | sudo apt-key add -
  sudo apt update
  sudo apt -y install tuxedo-tomte tuxedo-control-center tuxedo-*

fi

sudo apt update && sudo apt -y dist-upgrade && sudo apt -y autoremove


# Configs
# Yakuake
