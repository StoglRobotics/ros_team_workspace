script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"

# Load RosTeamWS defines
source $script_own_dir/scripts/_RosTeamWs_Defines.bash

# Load Team defines
source $script_own_dir/scripts/_Team_Defines.bash


# Set main path where source.bash is defined
RosTeamWS_FRAMEWORK_MAIN_PATH=$(RosTeamWS_script_own_dir)
