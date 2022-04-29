#!/bin/bash

# Load Framework defines
setup_ws_script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $setup_ws_script_own_dir/_RosTeamWs_Defines.bash
source $setup_ws_script_own_dir/docker/_RosTeamWs_Docker_Defines.bash

check_user_input () {
  # ros distribution name will be set in ${ros_distro}
  check_ros_distro $1

  ws_folder="$2"
  if [ -z "$2" ]; then
    ws_folder="workspace"
    echo "Using default '~/workspace' folder to setup ros workspace"
  elif [ "$use_docker" == true ] && [[ "$2" == *"-"* ]]; then
    print_and_exit "Workspace names for docker cannot contain \"-\"."
  fi


  ros_ws_prefix="$3"
  if [ -z "$3" ]; then
    ros_ws_prefix="-"
    echo "No ros_ws_prefix used..."
  fi

  ros_ws_suffix="$4"
  if [ -z "$4" ]; then
    ros_ws_suffix="-"
    echo "No ros_ws_suffix used..."
  fi
}

setup_new_workspace () {
    # TODO: Write this automatically from the user's definitions
  echo "Please choose which workspace should be basis for yours:"
  echo "(0) <Use current sourced workspace>"
  read choice

  if [ -z "$choice" ]; then
    print_and_exit "No workspace is chosen!" "$usage"
  fi

  case "$choice" in
  # "1")
  #    base_ws=Industrial
  #    ;;
  # "2")
  #    base_ws=Mobile
  #    ;;
  "0")
    base_ws="<current>"
    ;;
  *)
    print_and_exit "No workspace chosen! Exiting..."
  esac

  # Form here the full name
  ws_full_name=${ros_distro}

  if [ "$ros_ws_prefix" != "-" ]; then
    ws_full_name=${ros_ws_prefix}_${ws_full_name}
  fi
  if [ "$ros_ws_suffix" != "-" ]; then
    ws_full_name=${ws_full_name}_${ros_ws_suffix}
  fi

  # TODO: Add here output of the <current> WS
  echo ""
  read -p "ATTENTION: Creating a new workspace in folder '${ws_folder}' for ROS '${ros_distro}' (ROS$ros_version) with suffix '${ros_ws_suffix}' (full path: '${ws_folder}/${ws_full_name}') using '${base_ws}' as base workspace. Press <ENTER> to continue..."

  # Create and initialise ROS-Workspace
  if [[ ${base_ws} != "<current>" ]]; then
    if [[ $ros_version == 1 ]]; then
      source $FRAMEWORK_BASE_PATH/${base_ws}_ros_ws_${ros_distro}/devel/setup.bash
    else
      source $FRAMEWORK_BASE_PATH/${base_ws}_ros_ws_${ros_distro}/install/setup.bash
    fi
  fi

  mkdir -p ~/${ws_folder}/${ws_full_name}
  cd ~/${ws_folder}/${ws_full_name} || print_and_exit "Could not change dir to workspace folder. Something went wrong."

  if [ "$use_docker" = false ]; then # only build if not in docker, to avoide wrong dependencies
    if [[ $ros_version == 1 ]]; then
      wstool init src
      catkin config -DCMAKE_BUILD_TYPE=RelwithDebInfo
      catkin build
    elif [[ $ros_version == 2 ]]; then
      mkdir src
    #   cb
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    fi
  fi

  # Go to Home folder of the user
  cd || print_and_exit "Could not change back to home dir. This should never happen..."
}

setup_ros_team_ws_file() {
  if [ -z "$1" ]; then
    print_and_exit "No ros_team_ws_file given. Cannot setup workspace defines."
  fi
  local ros_team_ws_file=$1
  if [ -z "$2" ]; then
    print_and_exit "Not specified if docker should be used. Cannot setup workspace defines."
  fi
  local use_docker=$2
    if [ -z "$2" ]; then
    print_and_exit "Not specified if ros_teamws_file is for docker. Cannot setup workspace defines."
  fi
  local is_docker_rtw_file=$3

  # Create a function name
  fun_name_suffix=${chosen_ros_distro}

  if [ "$ros_ws_prefix" != "-" ]; then
    fun_name_suffix=${fun_name_suffix}_${ros_ws_prefix}
  fi
  if [ "$ros_ws_suffix" != "-" ]; then
    fun_name_suffix=${fun_name_suffix}_${ros_ws_suffix}
  fi

  alias_name=_${ws_full_name}
  fun_name="RosTeamWS_setup_${fun_name_suffix}"

  cp "$ros_team_ws_file" "$ros_team_ws_file".bkp
  # Comment out the old configuration if such exists - this is hard if using functions...
  sed -i -e '/'"$fun_name"'/ s/^#*/OLD_/' "$ros_team_ws_file"
  sed -i -e '/alias st_ros'"$ros_version=$fun_name"'/ s/^#*/#/' "$ros_team_ws_file"

  if [ "$use_docker" = true ]; then
    docker_image_tag=$(sed "s/-//g" <(echo "ubuntu_20_04_${ros_ws_prefix}_${ws_folder}_${ros_ws_suffix}_${chosen_ros_distro}"))      
  else
    docker_image_tag="-"
  fi

  local docker_support="$use_docker"
  if [ "$is_docker_rtw_file" = true ]; then
    source_path_rtw="  source /opt/RosTeamWS/ros_ws_$chosen_ros_distro/src/ros_team_workspace/setup.bash \"\$RosTeamWS_DISTRO\" \"\$RosTeamWS_WS_FOLDER\" \"\$RosTeamWS_WS_PREFIX\" \"\$RosTeamWS_WS_SUFFIX\"" 
    docker_support=false # don't use docker in docker
  else
    source_path_rtw="  source $FRAMEWORK_BASE_PATH/ros_ws_\"\$RosTeamWS_DISTRO\"/src/$FRAMEWORK_NAME/scripts/environment/setup.bash \"\$RosTeamWS_DISTRO\" \"\$RosTeamWS_WS_FOLDER\" \"\$RosTeamWS_WS_PREFIX\" \"\$RosTeamWS_WS_SUFFIX\""
  fi 

  echo "" >> "$ros_team_ws_file"
  echo "$fun_name () {" >> "$ros_team_ws_file"
  echo "  RosTeamWS_BASE_WS=\"${base_ws}\"" >> "$ros_team_ws_file"
  echo "  RosTeamWS_DISTRO=\"${chosen_ros_distro}\"" >> "$ros_team_ws_file"
  echo "  RosTeamWS_WS_FOLDER=\"${ws_folder}\"" >> "$ros_team_ws_file"
  echo "  RosTeamWS_WS_PREFIX=\"${ros_ws_prefix}\"" >> "$ros_team_ws_file"
  echo "  RosTeamWS_WS_SUFFIX=\"${ros_ws_suffix}\"" >> "$ros_team_ws_file"
  echo "  RosTeamWS_WS_DOCKER_SUPPORT=\"${docker_support}\"" >> "$ros_team_ws_file"
  echo "  RosTeamWS_DOCKER_TAG=\"${docker_image_tag}\"" >> "$ros_team_ws_file"
  echo "$source_path_rtw" >> "$ros_team_ws_file"
  echo "}" >> "$ros_team_ws_file"
  echo "alias $alias_name=$fun_name" >> "$ros_team_ws_file"

}

update_config()
{
  local rtw_file=~/.ros_team_ws_rc

  setup_ros_team_ws_file "$rtw_file" "$use_docker" "$is_docker_rtw_file"

  # source new workspace
  source "$rtw_file"

  # Update rosdep definitions
  rosdep update

  if [[ $ros_version == 1 ]]; then
    rospack profile
  fi
}

setup_docker_files () {
  echo "Not implemented"
}

# workspace folder is relative to your home
create_workspace () {
  local usage="setup-ros-workspace.bash create_workspace ROS_DISTRO WS_FOLDER WS_PREFIX WS_SUFFIX"
  local use_docker=false

  check_user_input "$@"
  local chosen_ros_distro=$ros_distro
  setup_new_workspace
  
  local is_docker_rtw_file=false
  update_config

  echo "------------------------------------------------------"
  echo "Finished creating new workspace: Please open a new terminal and execute '$alias_name'"
}

create_workspace_docker () {
  local usage="setup-ros-workspace.bash create_workspace_docker ROS_DISTRO WS_FOLDER WS_PREFIX WS_SUFFIX"
  local use_docker=true

  # create new workspace locally
  check_user_input "$@"
  local chosen_ros_distro=$ros_distro # need to store, ros_distro gets overwritten while creating workspace...
  setup_new_workspace
  
  local is_docker_rtw_file=false
  update_config

  ############################################################################
  # DOCKER PART: create corresponding docker image and map worksapce and rtw #
  ############################################################################

  # first get name for creating correct ros distro in docker
  local docker_ros_distro_name
  if [[ -v "map_to_docker_ros_distro_name[$chosen_ros_distro]" ]];then
    docker_ros_distro_name=${map_to_docker_ros_distro_name["$chosen_ros_distro"]}
  else
    print_and_exit "For $chosen_ros_distro does no docker container exist."
  fi

  # setup Dockerfile basrc and .ros_team_ws_rc
  # and copy needed files to workspace dir
  ws_docker_folder="${ws_folder}/.rtw_docker_defines"
  mkdir -p "$ws_docker_folder"

  # setup .bashrc
  cp "$DOCKER_TEMPLATES/bashrc" "$ws_docker_folder/."
  echo "$alias_name" >> "$ws_docker_folder/bashrc"

  # setup Dockerfile: set correct docker version
  cp "$DOCKER_TEMPLATES/Dockerfile" "$ws_docker_folder/."
  sed -i "s/ROS_DUMMY_VERSION/${docker_ros_distro_name}/g" "$ws_docker_folder/Dockerfile"

  # setup ros_team_ws
  cp "$DOCKER_TEMPLATES/ros_team_ws_rc_docker" "$ws_docker_folder/."
  # make copied .ros_team_ws match to source
  local rtw_file="$ws_docker_folder/ros_team_ws_rc_docker"
  sed -i "s/RTW_DISTRO/${chosen_ros_distro}/g" "$rtw_file"
  setup_ros_team_ws_file "$rtw_file" "$use_docker" "true"

  # now we are all set for building the container
  build_docker_container "$docker_image_tag" "$ws_docker_folder/Dockerfile" || { print_and_exit "Build of docker container failed."; }

  echo ""
  echo "######################################################################################################################"
  echo "Finished creating new workspace with docker support: Going to switch to docker. Next time simply run '$alias_name'"
  echo "######################################################################################################################"
  sleep 2 # give user time to read above message before switching to docker container

  create_docker_image "$docker_image_tag" "$ws_folder" "$chosen_ros_distro"
}

# needed for expanding the arguments
# DO NOT REMOVE!
"$@"