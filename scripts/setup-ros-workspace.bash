#!/bin/bash

# Load Framework defines
setup_ws_script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $setup_ws_script_own_dir/../setup.bash

check_user_input () {
  # ros distribution name will be set in ${ros_distro}
  check_and_set_ros_distro_and_version "$1"

  ws_folder="$2"
  if [ -z "$2" ]; then
    ws_folder="workspace"
    echo "Using default '~/workspace' folder to setup ros workspace"
  elif [ "$use_docker" == true ] && [[ "$2" == *"-"* ]]; then
    print_and_exit "Workspace names for docker cannot contain \"-\"."
  fi

  # Todo Manuel make this more generic
  if [ "$use_docker" == true ]; then
    if [ "$ros_distro" == "rolling" ]; then
      echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}'$ros_distro' is currently supported on multiple versions of Ubuntu. Which version of ubuntu would you like to choose:"
      select ubuntu_version in Ubuntu_20_04 Ubuntu_22_04;
      do
        case "$ubuntu_version" in
              Ubuntu_20_04)
                  ubuntu_version="ubuntu:20.04"
                  ubuntu_version_tag="ubuntu_20_04"
                  break
                ;;
              Ubuntu_22_04)
                  ubuntu_version="ubuntu:22.04"
                  ubuntu_version_tag="ubuntu_22_04"
                  break
                ;;
        esac
      done
      echo -n -e "${TERMINAL_COLOR_NC}"
    else
      ubuntu_version="ubuntu:20.04"
      ubuntu_version_tag="ubuntu_20_04"
    fi
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
  ros_distro_base_workspace_source_path="/opt/ros/$ros_distro/setup.bash"

#   # TODO: Write this automatically from the user's definitions
#   echo "Please choose which workspace should be basis for yours:"
#   echo "(0) <Use current sourced workspace> (or if non sourced, source '${ros_distro_base_workspace_source_path}')"
#   read choice

  # TODO(destogl): This is only temporarily solution until we offer different base-workspaces support (above commented lines)
  choice="0"
  echo -e "${RTW_COLOR_NOTIFY_USER} The new workspace will base on currently sourced workspace or if none is sourced, '${ros_distro_base_workspace_source_path}' will be sourced."

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

  base_ws_path=""

  if [ -z "$ROS_DISTRO" ]; then
    echo -e "${TERMINAL_COLOR_YELLOW}No workspace is sourced, sourcing: '${ros_distro_base_workspace_source_path}'${TERMINAL_COLOR_NC}"
    source ${ros_distro_base_workspace_source_path}
    base_ws_path="${ros_distro_base_workspace_source_path}"
  fi

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
  echo -e "${TERMINAL_COLOR_USER_NOTICE}ATTENTION: Creating a new workspace in folder '${ws_folder}' for ROS '${ros_distro}' (ROS$ros_version) with suffix '${ros_ws_suffix}' (full path: '${ws_folder}/${ws_full_name}') using '${base_ws}' (${base_ws_path}) as base workspace.${TERMINAL_COLOR_NC}"
  echo ""
  echo -e "${TERMINAL_COLOR_USER_CONFIRMATION}If correct press <ENTER>, otherwise <CTRL>+C and start the script again from the package folder and/or with correct controller name.${TERMINAL_COLOR_NC}"
  read

  # TODO(destogl): This part with base workspaces should be updated!!! - this is obsolete logic
  # Create and initialise ROS-Workspace
  if [[ ${base_ws} != "<current>" ]]; then
    if [[ $ros_version == 1 ]]; then
      source $FRAMEWORK_BASE_PATH/${base_ws}_ros_ws_${ros_distro}/devel/setup.bash
    else
      source $FRAMEWORK_BASE_PATH/${base_ws}_ros_ws_${ros_distro}/install/setup.bash
    fi
  fi

  mkdir -p ~/${ws_folder}/${ws_full_name}
  cd ~/${ws_folder}/${ws_full_name} || { print_and_exit "Could not change dir to workspace folder. Something went wrong."; }

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

  if [ -f "$ros_team_ws_file" ]; then
    new_rtw_file_name="${ros_team_ws_file_name}.bkp-$(ls ${ros_team_ws_file}* | wc -l)"
    echo ""
    cp "$ros_team_ws_file" "$new_rtw_file_name"
    notify_user "${ros_team_ws_file_name} already exists. Copied it to ${new_rtw_file_name}."
    # Comment out the old configuration if such exists - this is hard if using functions...
    sed -i -e '/'"$fun_name"'/ s/^#*/OLD_/' "$ros_team_ws_file"
    sed -i -e '/alias st_ros'"$ros_version=$fun_name"'/ s/^#*/#/' "$ros_team_ws_file"
  fi

  if [ "$use_docker" = true ]; then
    docker_image_tag=$(sed "s/[\/]/-/g" <(echo "ros-team-ws_${ubuntu_version_tag}__${ws_folder}__${ws_full_name}"))
    docker_host_name="rtw-${ws_full_name}-docker"
  else
    docker_image_tag="-"
    docker_host_name="-"
  fi

  local docker_support="$use_docker"
  if [ "$is_docker_rtw_file" = true ]; then
    docker_support=false # don't use docker in docker
  fi

  source_path_rtw="  source $FRAMEWORK_BASE_PATH/scripts/environment/setup.bash \"\$RosTeamWS_DISTRO\" \"\$RosTeamWS_WS_FOLDER\" \"\$RosTeamWS_WS_PREFIX\" \"\$RosTeamWS_WS_SUFFIX\""

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
  local ros_team_ws_file_name=".ros_team_ws_rc"
  local ros_team_ws_file="$HOME/$ros_team_ws_file_name"

  setup_ros_team_ws_file "$ros_team_ws_file" "$use_docker" "$is_docker_rtw_file"

  # source new workspace
  source "$ros_team_ws_file"

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

  echo -e "${RTW_COLOR_NOTIFY_USER}Finished creating new workspace: Please open a new terminal and execute '$alias_name'${TERMINAL_COLOR_NC} (if you have setup auto sourcing). Otherwise you first have to source your ~/.ros_team_ws_rc file."
}

create_workspace_docker () {
  local usage="setup-ros-workspace.bash create_workspace_docker ROS_DISTRO WS_FOLDER WS_PREFIX WS_SUFFIX"
  local use_docker=true

  # create new workspace locally
  check_user_input "$@"
  local chosen_ros_distro=$ros_distro # need to store, ros_distro gets overwritten while creating workspace...
  setup_new_workspace

  # TODO(destogl): we should remove updating of configs if docker is used - can we do this already?
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
  local rtw_branch_for_ros_distro
  if [[ -v "ros_distro_to_rtw_branch[$docker_ros_distro_name]" ]];then
    rtw_branch_for_ros_distro=${ros_distro_to_rtw_branch["$docker_ros_distro_name"]}
  else
    print_and_exit "For $docker_ros_distro_name does no RosTeamWorkspace branch exist."
  fi

  # setup Dockerfile bashrc and .ros_team_ws_rc
  # and copy needed files to workspace dir
  ws_docker_folder="${ws_folder}/${ws_full_name}/.rtw_docker_defines"
  mkdir -p "$ws_docker_folder"

  # setup .bashrc
  cp "$DOCKER_TEMPLATES/bashrc" "$ws_docker_folder/."
  # add the name of the workspace folder in docker, so that it can be inited in bashrc the first time used.
  local docker_ws_folder
  docker_ws_folder="${ws_folder}/${ws_full_name}"
  sed -i "s|DUMMY_WS_FOLDER|${docker_ws_folder}|g" "$ws_docker_folder/bashrc"
  echo "" >> "$ws_docker_folder/bashrc"
  echo "$alias_name" >> "$ws_docker_folder/bashrc"

  # setup Dockerfile: set correct docker version
  cp "$DOCKER_TEMPLATES/Dockerfile" "$ws_docker_folder/."
  sed -i "s/UBUNTU_DUMMY_VERSION/${ubuntu_version}/g" "$ws_docker_folder/Dockerfile"
  sed -i "s/ROS_DUMMY_VERSION/${docker_ros_distro_name}/g" "$ws_docker_folder/Dockerfile"
  sed -i "s/ROS_TEAM_WS_DUMMY_BRANCH/${rtw_branch_for_ros_distro}/g" "$ws_docker_folder/Dockerfile"

  # setup ros_team_ws
  cp "$DOCKER_TEMPLATES/ros_team_ws_rc_docker" "$ws_docker_folder/."
  # make copied .ros_team_ws match to source
  local rtw_file="$ws_docker_folder/ros_team_ws_rc_docker"
  sed -i "s/ROS_DUMMY_VERSION/${chosen_ros_distro}/g" "$rtw_file"
  setup_ros_team_ws_file "$rtw_file" "$use_docker" "true"

  # copy file for recreating docker
  cp "$DOCKER_TEMPLATES/recreate_docker.sh" "$ws_docker_folder/."
  sed -i "s/DUMMY_DOCKER_IMAGE_TAG/${docker_image_tag}/g" "$ws_docker_folder/recreate_docker.sh"
  sed -i "s/DUMMY_DOCKER_HOSTNAME/${docker_host_name}/g" "$ws_docker_folder/recreate_docker.sh"
  sed -i "s|DUMMY_WS_FOLDER|${docker_ws_folder}|g" "$ws_docker_folder/recreate_docker.sh"

  # now we are all set for building the container
  build_docker_image "$docker_image_tag" "$ws_docker_folder/Dockerfile" || { print_and_exit "Build of docker container failed."; }

  echo ""
  echo "######################################################################################################################"
  echo -e "${RTW_COLOR_NOTIFY_USER}Finished creating new workspace with docker support: Going to switch to docker. Next time simply run '$alias_name'${TERMINAL_COLOR_NC}"
  echo "######################################################################################################################"
  sleep 2 # give user time to read above message before switching to docker container

  create_docker_container "$docker_image_tag" "${docker_ws_folder}" "$chosen_ros_distro" "$docker_host_name"
}
