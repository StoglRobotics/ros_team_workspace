#!/usr/bin/bash
#
# /vol64/IPR-Framework/IPR_ros_ws_melodic/src/intelligent_robotic_automation/scripts/os_configure/configure_ipr-framework.bash
#

echo "You will configure ROS Team Workspace for '$ros_distro'. Are you sure you want to continue?"
echo "Press <ENTER> if yes!"
read

git config --global credential.helper cache
git config --global credential.helper 'cache --timeout=600'

main_dir="/opt/RosTeamWS/ros_ws_foxy/src"

sudo mkdir -p $main_dir

# Create maintainers group and add current user to it
sudo groupadd -r -g 700 rws-maintainers
sudo usermod -a -G rws-maintainers `whoami`

sudo mkdir -p $FRAMEWORK_REPO_PATH/src
sudo chown -R root:rws-maintainers $FRAMEWORK_REPO_PATH

cd $FRAMEWORK_REPO_PATH/src

## TODO: Correct this form here to use configuration from the GitHub repository

vcs import src < $REMOTE_FRAMEWORK_PATH/src/.rosinstall
bash $SCRIPT_PATH/update_IPR_Framework.bash $ros_distro

cd $FRAMEWORK_BASE_PATH
ln -s $REMOTE_FRAMEWORK_BASE_PATH/IPR_ros_ws/src/intelligent_robotic_automation/scripts/environment/setup.bash
