setup_script_own_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

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
  local python_constants_file="$setup_script_own_dir/rtwcli/rtwcli/rtwcli/constants.py"
  local ws_use_bash_file_path_format
  ws_use_bash_file_path_format=$(python3 -c "
import os; import pathlib; import sys
sys.path.insert(0, os.path.dirname('$python_constants_file'))
from rtwcli.constants import WS_USE_BASH_FILE_PATH_FORMAT
print(WS_USE_BASH_FILE_PATH_FORMAT)")

  if [[ -z $ws_use_bash_file_path_format ]]; then
    echo "Error: Could not get WS_USE_BASH_FILE_PATH_FORMAT from $python_constants_file"
  else
    local file_name="${ws_use_bash_file_path_format//\{ppid\}/$$}"
    if [[ -f $file_name ]]; then # If file exists
      local file_mod_time
      file_mod_time=$(stat -c %Y $file_name)

      # If file was modified after the last source operation
      if ((file_mod_time > ROS_WS_CACHE_SOURCED_TIME)); then
        source "$file_name"
        ROS_WS_CACHE_SOURCED_TIME=$file_mod_time
        export ROS_WS_CACHE_SOURCED_TIME
      fi
    fi
  fi
}
# run this command before every prompt, similar to PS1 variable
PROMPT_COMMAND=update_ros_ws_variables
