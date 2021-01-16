## BEGIN: definitions
DEFAULT_ROS_DISTRO="foxy"
DEFAULT_ROS_VERSION=2


# We have two example teams. On is working with industrial and other with mobile robots
TEAM_TEAM_NAMES=("Industrial" "Mobile")

## END: definitions


## BEGIN: Team setup

setup_ros1_exports () {

export ROSCONSOLE_FORMAT='[${severity}] [${walltime}: ${logger}] [${node}@${file}.${function}:${line}]: ${message}'
export ROSCONSOLE_CONFIG_FILE='~/workspace/ros_ws/rosconsole.config'

}

setup_ros1_aliases () {

  alias cb="catkin build"

}


setup_ros2_exports ()  {

export RTI_LICENSE_FILE=/opt/rti.com/rti_connext_dds-5.3.1/rti_license.dat

#  export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
# export RCUTILS_COLORIZED_OUTPUT=1
# export RCUTILS_LOGGING_USE_STDOUT=1
#export RCUTILS_LOGGING_BUFFERED_STREAM=

}


setup_ros2_aliases () {

    alias cba="colcon build --symlink-install"
    alias cbap="colcon build --symlink-install --packages-select"
    #colcon_build()
    #{
    #    local pkg=\$1
    #    colcon build --symlink-install --packages-select \${pkg}
    #}
    #alias colcon_build="colcon_build \$@"

    # Author: Jordan Palacios
    #colcon_run_test()
    #{
    #    local pkg=\$1
    #    colcon build --symlink-install --packages-select \${pkg} && \
    #        colcon test --packages-select \${pkg} && \
    #        colcon test-result
    #}
    #alias colcon_run_test=colcon_run_test \$@

}

## END: Team setup


## BEGIN: Framework functions

framework_default_paths () {
    ros_distro=$1

    FRAMEWORK_NAME="ros_team_workspace"
    FRAMEWORK_BASE_PATH="/opt/RosTeamWS"
    FRAMEWORK_REPO_PATH="$FRAMEWORK_BASE_PATH/ros_ws_$ros_distro"
    REMOTE_FRAMEWORK_BASE_PATH="/vol64_remote/IPR-Framework"
    REMOTE_FRAMEWORK_PATH="$REMOTE_FRAMEWORK_BASE_PATH/IPR_ros_ws_$ros_distro"
    #TODO: use this in the future
    SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"
    if [ ! -d "$FRAMEWORK_REPO_PATH" ]; then
        echo "FRAMEWORK_REPO_PATH: local not found, set to remote..."
        FRAMEWORK_REPO_PATH="$REMOTE_FRAMEWORK_PATH/src/intelligent_robotic_automation/scripts"
    fi 
    FRAMEWORK_REPO_PATH="$REMOTE_FRAMEWORK_PATH/src/intelligent_robotic_automation/scripts"
}

check_ros_distro () {
    ros_distro=$1
    if [ -z "$1" ]; then
        ros_distro=$DEFAULT_ROS_DISTRO
        echo "No ros_distro defined. Using default: '$ros_distro'"
        if [ ! -d "/opt/ros/$ros_distro" ]; then
            echo "FATAL: ROS '$ros_distro' not installed on this computer! Exiting..."
            exit
        fi
        echo "Press <ENTER> to continue or <CTRL>+C to exit."
        read
    fi

    if [ ! -d "/opt/ros/$ros_distro" ]; then
        echo "FATAL: ROS '$ros_distro' not installed on this computer! Exiting..."
        exit
    fi

    ros_version=$DEFAULT_ROS_VERSION
    if [[ $ros_distro == "foxy" ]]; then
      ros_version=2
    elif [[ $ros_distro == "noetic" ]]; then
      ros_version=1
    fi

    framework_default_paths $ros_distro
}

# END: Framework functions
