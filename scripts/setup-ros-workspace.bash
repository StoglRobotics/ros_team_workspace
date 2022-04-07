#!/bin/bash

# Load Framework defines
script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $script_own_dir/_RosTeamWs_Defines.bash
source $script_own_dir/docker/_RosTeamWs_Docker_Defines.bash

check_user_input () {
  # ros distribution name will be set in ${ros_distro}
  check_ros_distro $1

  ws_folder="$2"
  if [ -z "$2" ]; then
    ws_folder="workspace"
    echo "Using default '~/workspace' folder to setup ros workspace"
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

  if [[ $ros_version == 1 ]]; then
    wstool init src
    catkin config -DCMAKE_BUILD_TYPE=RelwithDebInfo
    catkin build
  elif [[ $ros_version == 2 ]]; then
    mkdir src
  #   cb
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
  fi

  # Go to Home folder of the user
  cd || print_and_exit "Could not change back to home dir. This should never happen..."
}

setup_ros_team_ws_file() {
  if [ -z "$1" ]; then
    print_and_exit "No docker image (tag) specified. Can not instantiate image."
  fi
  local ros_team_ws_file=$1

  # Create a function name
  fun_name_suffix=${ros_distro}

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
  sed -i -e '/alias st_ros'"$ros_version=$fun_name"'/ s/^#*/#/' "$ros_team_ws_file"c

  if [ "$use_docker" = true ]; then
    docker_image_tag="${ros_ws_prefix}_${ws_folder}_${ros_ws_suffix}-${ros_distro}"
  else
    docker_image_tag="-"
  fi

  echo "" >> "$ros_team_ws_file"
  echo "$fun_name () {" >> "$ros_team_ws_file"
  echo "  RosTeamWS_BASE_WS=\"${base_ws}\"" >> "$ros_team_ws_file"
  echo "  RosTeamWS_DISTRO=\"${ros_distro}\"" >> "$ros_team_ws_file"
  echo "  RosTeamWS_WS_FOLDER=\"${ws_folder}\"" >> "$ros_team_ws_file"
  echo "  RosTeamWS_WS_PREFIX=\"${ros_ws_prefix}\"" >> "$ros_team_ws_file"
  echo "  RosTeamWS_WS_SUFFIX=\"${ros_ws_suffix}\"" >> "$ros_team_ws_file"
  echo "  RosTeamWS_WS_DOCKER_SUPPORT=\"$use_docker\"" >> "$ros_team_ws_file"
  echo "  RosTeamWS_DOCKER_TAG=\"${docker_image_tag}\"" >> "$ros_team_ws_file"
  echo "  source $FRAMEWORK_BASE_PATH/ros_ws_\"\$RosTeamWS_DISTRO\"/src/$FRAMEWORK_NAME/scripts/environment/setup.bash \"\$RosTeamWS_DISTRO\" \"\$RosTeamWS_WS_FOLDER"\" \"\$RosTeamWS_WS_PREFIX\" \"\$RosTeamWS_WS_SUFFIX\" >> "$ros_team_ws_file"
  echo "}" >> "$ros_team_ws_file"
  echo "alias $alias_name=$fun_name" >> "$ros_team_ws_file"

}

update_config()
{
  local rtw_file=~/.ros_team_ws_rc
  setup_ros_team_ws_file "$rtw_file"

  # source new workspace
  source "$rtw_file"

  # Update rosdep definitions
  rosdep update

  if [[ $ros_version == 1 ]]; then
    rospack profile
  fi
}

clean_up ()
{
  rm "$DOCKER_TEMPLATES/Dockerfile"
  rm "$DOCKER_TEMPLATES/bashrc"
  rm "$DOCKER_TEMPLATES/.ros_team_ws_rc_docker"
    # invalidate docker support for workspace
  sed -i "s/RosTeamWS_WS_DOCKER_SUPPORT=\"true\"/RosTeamWS_WS_DOCKER_SUPPORT=\"false\"/g" "$ros_team_ws_file"
  sed -i "s/RosTeamWS_DOCKER_TAG=\"${docker_image_tag}\"/RosTeamWS_DOCKER_TAG=\"-\"/g" "$ros_team_ws_file"
}

# workspace folder is relative to your home
create_workspace () {
  local usage="setup-ros-workspace.bash create_workspace ROS_DISTRO WS_FOLDER WS_PREFIX WS_SUFFIX"
  use_docker=false

  check_user_input "$@"
  setup_new_workspace
  update_config

  echo "------------------------------------------------------"
  echo "Finished creating new workspace: Please open a new terminal and execute '$alias_name'"
}

create_workspace_docker () {
  local usage="setup-ros-workspace.bash create_workspace_docker ROS_DISTRO WS_FOLDER WS_PREFIX WS_SUFFIX"
  use_docker=true

  # create new workspace locally
  check_user_input "$@"
  setup_new_workspace
  update_config

  ############################################################################
  # DOCKER PART: create corresponding docker image and map worksapce and rtw #
  ############################################################################

  # first get name for creating correct ros distro in docker
  local docker_ros_distro_name
  if [[ -v "map_to_docker_ros_distro_name[$ros_distro]" ]];then
    docker_ros_distro_name=${map_to_docker_ros_distro_name["$ros_distro"]}
  else
    print_and_exit "For $ros_distro does no docker container exist."
  fi

  # trap interrupts so we can clean copied files only needed from now on, because bevor no files have been copied
  trap clean_up INT

  # setup Dockerfile basrc and .ros_team_ws_rc
  # and temporary copy needed files to docker dir
  docker_script_dir="$script_own_dir"/docker/
  cp "$DOCKER_TEMPLATES/bashrc" "$docker_script_dir/."
  cp "$DOCKER_TEMPLATES/Dockerfile" "$docker_script_dir/."
  cp "$DOCKER_TEMPLATES/.ros_team_ws_rc_docker" "$docker_script_dir/."

  # setup Dockerfile: set correct docker version
  sed -i "s/ROS_DUMMY_VERSION/${docker_ros_distro_name^^}/g" "$docker_script_dir/Dockerfile"

  # setup ros_team_ws
  # make copied .ros_team_ws match to source
  sed -i "s/RTW_DISTRO/${ros_distro^^}/g" "$docker_script_dir/Dockerfile"
  local rtw_file="$docker_script_dir/.ros_team_ws_rc_docker"
  setup_ros_team_ws_file "$rtw_file"

  # now we are all set for building the container
  build_docker_container "$docker_image_tag" || { clean_up; print_and_exit "Build of docker container failed."; }
  echo "Finished creating new workspace with docker support: Going to switch to docker. Next time simply run '$alias_name'"
  sleep 1 # give user time to read above message before switching to docker container

  create_docker_image "$docker_image_tag" "$ws_folder" "$ros_distro"
}

# needed for expanding the arguments
# DO NOT REMOVE!
"$@"
