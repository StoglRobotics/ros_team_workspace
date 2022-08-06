setup_script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"

# Load RosTeamWS defines
source $setup_script_own_dir/scripts/_RosTeamWs_Defines.bash

# Set default paths
framework_default_paths $DEFAULT_ROS_DISTRO

# Load Team defines
source $setup_script_own_dir/scripts/_Team_Defines.bash

# Load Docker defines
source $setup_script_own_dir/scripts/_RosTeamWs_Docker_Defines.bash

# Set main path where source.bash is defined
RosTeamWS_FRAMEWORK_MAIN_PATH="$(RosTeamWS_script_own_dir)/../"
