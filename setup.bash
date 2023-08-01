setup_script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"

# Load RosTeamWS defines
source $setup_script_own_dir/scripts/_RosTeamWs_Defines.bash

# setup default versions and paths for rtw
set_supported_versions
set_framework_default_paths
# Get color definitions
RosTeamWS_setup_exports

# Load Team defines
source $setup_script_own_dir/scripts/_Team_Defines.bash

# Load Docker defines
source $setup_script_own_dir/scripts/_RosTeamWs_Docker_Defines.bash

# Set main path where source.bash is defined
RosTeamWS_FRAMEWORK_MAIN_PATH="$(RosTeamWS_script_own_dir)/../"

# Source autocompletion for rtwcli
source $setup_script_own_dir/rtwcli/rtwcli/completion/rtw-argcomplete.bash

# rtwcli: export ros workspace variables if chosen (rtw workspace use)
export ROS_WS_CACHE_SOURCED_TIME=0
function update_ros_ws_variables {
  local file_name="/tmp/ros_team_workspace/wokspace_$$.bash"
  # If file exists
  if [[ -f $file_name ]]; then
    local file_mod_time=$(stat -c %Y $file_name)

    # If file was modified after the last source operation
    if (( file_mod_time > ROS_WS_CACHE_SOURCED_TIME )); then
        source $file_name
        ROS_WS_CACHE_SOURCED_TIME=$file_mod_time
        export ROS_WS_CACHE_SOURCED_TIME
    fi
  fi
}
# run this command before every prompt, similar to PS1 variable
PROMPT_COMMAND=update_ros_ws_variables
