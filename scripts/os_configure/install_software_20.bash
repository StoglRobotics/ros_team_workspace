#!/usr/bin/bash
#
# install_software_20.bash [computer_type:{office, robot, default:lab}]
#

script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
SCRIPT_PATH=$script_own_dir

computer_type=$1
if [ -z "$1" ]
then
  computer_type="lab"
  echo "Computer type not provided using default $computer_type"
fi

echo "Installing software for $computer_type computers. Press <ENTER> to continue."
read

ROS_VERSION=melodic
ROS2_VERSION=foxy

#KDE Backports
sudo apt-add-repository -y ppa:kubuntu-ppa/backports
sudo apt update && sudo apt -y dist-upgrade && sudo apt -y autoremove

### Useful tools
sudo apt -y install vim ssh git trash-cli htop unrar yakuake screen finger ksshaskpass kompare
# Dolphin Plugins
sudo apt -y install kdesdk-dolphin-plugins kdesdk-kio-plugins kdesdk-scripts

if ([[ $computer_type != "robot" ]])
then
  sudo apt -y install recordmydesktop rdesktop gimp gimp-ufraw meshlab inkscape pdfposter unrar
  sudo DEBIAN_FRONTEND=noninteractive apt -y install wireshark
  sudo apt -y install gvfs-bin gvfs-fuse gvfs-backends
fi

# Useful libraries
sudo apt -y install libxml2-dev

if ([[ $computer_type != "robot" ]])
then
  # Latex
  sudo apt -y install kile texlive-full texlive-lang-german kbibtex ktikz
fi

# Libraries
sudo apt -y install libvlc-dev libmuparser-dev

# Python tools
sudo apt -y install python3-pip
sudo pip3 install --upgrade pip
sudo pip3 install virtualenv virtualenvwrapper notebook


# ROS Packages
bash $SCRIPT_PATH/install_software_ros.bash $ROS_VERSION

# ROS2 Packages
bash $SCRIPT_PATH/install_software_ros2.bash $ROS2_VERSION

sudo rosdep init
rosdep update

# if ([[ $computer_type == "lab" ]] || [[$computer_type == "robot" ]])
# then
#   bash $SCRIPT_PATH/configure_ROSTeamWS.bash $ROS_VERSION
# fi
#
# if ([[ $computer_type == "robot" ]])
# then
#   bash $SCRIPT_PATH/configure_robots.bash $ROS_VERSION
# fi

if ([[ $computer_type == "office" ]])
then
  sudo apt -y install redshift

  #Nvidia drivers
#   sudo add-apt-repository -y ppa:graphics-drivers/ppa
#   sudo apt update
#   sudo apt -y install nvidia-396

  # Network manager
  sudo apt -y install network-manager-openvpn network-manager-vpnc network-manager-ssh network-manager-openconnect

  sudo apt -y install kleopatra scdaemon
  #see: #13 at https://bugs.launchpad.net/kubuntu-ppa/+bug/684902
#   echo 'use-agent' >> .gnupg/gpg.conf

  # Datensicherung
#   sudo apt -y install kup bup

  sudo apt -y install flac

  sudo add-apt-repository -y ppa:freecad-maintainers/freecad-daily
  sudo apt update
  sudo apt -y install freecad-daily spacenavd

  #NextCloud
  sudo add-apt-repository -y ppa:nextcloud-devs/client
  sudo apt update
  sudo apt -y install nextcloud-client

  #VirtualBox
  sudo sh -c 'echo "deb [arch=amd64] https://download.virtualbox.org/virtualbox/debian $(lsb_release -sc) contrib" > /etc/apt/sources.list.d/virtualbox.list'
  wget -q https://www.virtualbox.org/download/oracle_vbox_2016.asc -O- | sudo apt-key add -
  sudo apt update
  sudo apt -y install virtualbox-6.1 dkms virtualbox-guest-utils virtualbox-ext-pack

  # Hamsket
  sudo apt -y install libappindicator3-1
  wget https://github.com/TheGoddessInari/hamsket/releases/download/0.6.2/hamsket_0.6.2_amd64.deb
  sudo dpkg -i hamsket_0.6.2_amd64.deb

  # KDE-PIM
  sudo apt -y install kontact korganizer kmail kjots kaddressbook kdepim*

  # Tools
  sudo apt -y install  kdrc

fi

sudo apt update && sudo apt -y dist-upgrade && sudo apt -y autoremove


# Configs
# Yakuake
