#!/bin/bash

# Load Framework defines
setup_ws_script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $setup_ws_script_own_dir/../setup.bash
source $setup_ws_script_own_dir/docker/_RosTeamWs_Docker_Defines.bash

# All the possible supported ros distributions supported by rtw
if [ -z "$ros_distributions_20_04" ]; then
  readonly ros_distributions_20_04=("noetic" "foxy" "galactic")
fi
if [ -z "$ros_distributions_22_04" ]; then
  readonly ros_distributions_22_04=("humble" "iron")
fi
if [ -z "$ros_distributions_20_and_22_04" ]; then
  readonly ros_distributions_20_and_22_04=("rolling")
fi

ubuntu_20_04_version="ubuntu:20.04"
ubuntu_20_04_tag="ubuntu_20_04"
ubuntu_22_04_version="ubuntu:22.04"
ubuntu_22_04_tag="ubuntu_22_04"

select_normal_or_nvidia_docker() {
  # setup Dockerfile: set correct docker version
  echo -e "Do you want to setup a 'standard' or 'nvidia-based' docker container? [1]"
  echo "(1) standard"
  echo "(2) nvidia-based"
  echo -n -e ""
  read choice
  choice=${choice:="1"}

  docker_file=""
  case "$choice" in
  "1")
    docker_file="standard.dockerfile"
    ;;
  "2")
    # The version naming differs for nvidia-dockers e.g. instead of ubuntu:22.04->ubuntu22.04
    if [[ "$ubuntu_version" == "${ubuntu_20_04_version}" ]]; then
      ubuntu_version="ubuntu20.04"
    elif [[ "$ubuntu_version" == "${ubuntu_22_04_version}" ]]; then
      ubuntu_version="ubuntu22.04"
    else
      print_and_exit "Something went wrong. The Ubuntu version ${ubuntu_version} should be supported by nvidia but somehow it's not."
    fi
    ubuntu_version_tag=${ubuntu_version_tag}_nvidia
    docker_file="nvidia.dockerfile"
    notify_user "NOTE: Make sure that you have setup nvidia-drivers to support this!"
    notify_user "To abort press <CTRL>+<C>, to continue press <ENTER>."
    read
  esac


}

check_user_input () {
  ws_path="$1"
  if [ -z "$ws_path" ] || [[ "$ws_path" == "-" ]]; then
    ws_path="workspace"
    notify_user "Using default:\"${ws_path}\" as folder name to setup ros workspace"
  elif [ "$use_docker" == true ] && [[ "$(basename "${ws_path}")" == *"-"* ]]; then
    print_and_exit "Workspace names for docker cannot contain \"-\"."
  fi

  # if user passes a path then the true new workspace name is only the basename of the passed path
  ws_name=$(basename ${ws_path})

  # ros distribution name will be set in ${ros_distro}
  check_and_set_ros_distro_and_version "$2" "$use_docker"

  if [ "$use_docker" == true ]; then
    # according to selected ros distro the ubuntu version is selected.
    if [[ " ${ros_distributions_20_04[*]} " =~ " ${ros_distro} " ]]; then
      ubuntu_version=${ubuntu_20_04_version}
      ubuntu_version_tag=${ubuntu_20_04_tag}
      user_decision "The ros distribution ${ros_distro} is currently only on ${ubuntu_version} supported! Continue?"
      if [[ " ${negative_answers[*]} " =~ " ${user_answer} " ]]; then
        print_and_exit "Aborting creation of new docker workspace. Exiting..."
      fi
      select_normal_or_nvidia_docker
    elif [[ " ${ros_distributions_22_04[*]} " =~ " ${ros_distro} " ]]; then
      ubuntu_version=${ubuntu_22_04_version}
      ubuntu_version_tag=${ubuntu_22_04_tag}
      user_decision "The ros distribution ${ros_distro} is currently only on ${ubuntu_version} supported! Continue?"
      if [[ " ${negative_answers[*]} " =~ " ${user_answer} " ]]; then
        print_and_exit "Aborting creation of new docker workspace. Exiting..."
      fi
      select_normal_or_nvidia_docker
    elif [[ " ${ros_distributions_20_and_22_04[*]} " =~ " ${ros_distro} " ]]; then
      echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}'$ros_distro' is currently supported on multiple versions of Ubuntu. Which version of ubuntu would you like to choose:"
      select ubuntu_version in Ubuntu_20_04 Ubuntu_22_04;
      do
        case "$ubuntu_version" in
              Ubuntu_20_04)
                  ubuntu_version=${ubuntu_20_04_version}
                  ubuntu_version_tag=${ubuntu_20_04_tag}
                  break
                ;;
              Ubuntu_22_04)
                  ubuntu_version=${ubuntu_22_04_version}
                  ubuntu_version_tag=${ubuntu_22_04_tag}
                  break
                ;;
        esac
      done
      echo -n -e "${TERMINAL_COLOR_NC}"
      select_normal_or_nvidia_docker
    else
      print_and_exit "The selected ros distribution ${ros_distro} is not supported for docker workspaces!"
    fi
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
  notify_user "The new workspace will based on currently sourced workspace or if none is sourced, '${ros_distro_base_workspace_source_path}' will be sourced."

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
    notify_user "No workspace is sourced, sourcing: '${ros_distro_base_workspace_source_path}'"
    source ${ros_distro_base_workspace_source_path}
    base_ws_path="${ros_distro_base_workspace_source_path}"
  fi

  # Check if workspace which should be created is relative to $HOME workspace.
  # If this is the case user probably wants home as base of workspace otherwise current pwd is used.
  if [[ ${ws_path} = ${HOME}/* ]]; then
    # needs to be empty, since ws_path contains path with $HOME, otherwise we create something like:
    # $(pwd)/home/user/working_dir/new_workspace instead of /home/user/working_dir/new_workspace
    new_workspace_base_location=""
  else
    new_workspace_base_location="$(pwd)/"
  fi
  new_workspace_location=${new_workspace_base_location}${ws_path}

  # TODO: Add here output of the <current> WS
  echo ""
  notify_user "ATTENTION: Creating a new workspace in folder \"${ws_path}\" for ROS \"${ros_distro}\" (ROS$ros_version) (full path: ${new_workspace_location}) using \"${base_ws}\" (${base_ws_path}) as base workspace."
  echo ""
  user_confirmation "If correct press <ENTER>, otherwise <CTRL>+C and start the script again from the package folder and/or with correct controller name."

  # TODO(destogl): This part with base workspaces should be updated!!! - this is obsolete logic
  # Create and initialise ROS-Workspace
  if [[ ${base_ws} != "<current>" ]]; then
    if [[ $ros_version == 1 ]]; then
      source $FRAMEWORK_BASE_PATH/${base_ws}_ros_ws_${ros_distro}/devel/setup.bash
    else
      source $FRAMEWORK_BASE_PATH/${base_ws}_ros_ws_${ros_distro}/install/setup.bash
    fi
  fi

  mkdir -p "${new_workspace_location}" || { print_and_exit "Could not create new workspace folder \"${new_workspace_location}\" in ${new_workspace_base_location}. Something went wrong."; }
  current_pwd=$(pwd)
  cd "${new_workspace_location}" || { print_and_exit "Could not change dir to workspace folder. Something went wrong."; }

  if [ "$use_docker" = false ]; then # only build if not in docker, to avoide wrong dependencies
    if [[ $ros_version == 1 ]]; then
      wstool init src
      catkin config -DCMAKE_BUILD_TYPE=RelwithDebInfo
      catkin build
    elif [[ $ros_version == 2 ]]; then
      mkdir src
      # cb
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    fi
  fi

  # Go back to where we started
  cd "${current_pwd}" || print_and_exit "Could not change back to ${new_workspace_base_location} (working directory we created the new workspace in). This should never happen..."
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

  # if user creates a workspace like new_folder/my_new_workspace
  # alias should probably be _my_new_workspace instead of _new_folder/my_new_workspace
  alias_name=_${ws_name}
  fun_name="RosTeamWS_setup_${ws_name}"

  if [ "$is_docker_rtw_file" = false ]; then
    if [ -f "$ros_team_ws_file" ]; then
      new_rtw_file_name="${ros_team_ws_file_name}.bkp-$(ls ${ros_team_ws_file}* | wc -l)"
      echo ""
      cp "$ros_team_ws_file" "$HOME/$new_rtw_file_name"
      notify_user "${ros_team_ws_file_name} already exists. Copied it to ${new_rtw_file_name}."
      # move fun_name to OLD_fun_name if exists
      sed -i -e "s|$fun_name|OLD_${fun_name}|g" "$ros_team_ws_file"
      # and alias_name to OLD_alias
      sed -i -e "s|${alias_name}=OLD_${fun_name}|OLD_${alias_name}=OLD_${fun_name}|g" "$ros_team_ws_file"
      # uncomment alias line
      old_alias_line_number=$(grep -n "OLD_${alias_name}=OLD_${fun_name}" "$ros_team_ws_file" | grep -Eo '^[^:]+')
      # uncomment
      if ! [ -z "$old_alias_line_number" ]; then
          sed -i "${old_alias_line_number},${old_alias_line_number}s|^|#|g" "$ros_team_ws_file"
      fi
    else
      print_and_exit "No $ros_team_ws_file found! Please first setup auto sourcing with the \"setup-auto-sourcing\" command."
    fi
  fi

  if [ "$use_docker" = true ]; then
    docker_image_tag=$(sed "s/[\/]/-/g" <(echo "ros-team-ws_${ubuntu_version_tag}__${ws_name}"))
    docker_host_name="rtw-${ws_name}-docker"
  else
    docker_image_tag="-"
    docker_host_name="-"
  fi

  local docker_support="$use_docker"
  if [ "$is_docker_rtw_file" = true ]; then
    docker_support=false # don't use docker in docker
    source_path_rtw=" source /opt/RosTeamWS/ros_ws_$chosen_ros_distro/src/ros_team_workspace/scripts/environment/setup.bash \"\$RosTeamWS_DISTRO\" \"\$RosTeamWS_WS_FOLDER\""
  else
    source_path_rtw=" source $FRAMEWORK_BASE_PATH/scripts/environment/setup.bash \"\$RosTeamWS_DISTRO\" \"\$RosTeamWS_WS_FOLDER\""
  fi

  echo "" >> "$ros_team_ws_file"
  echo "$fun_name () {" >> "$ros_team_ws_file"
  echo "  RosTeamWS_BASE_WS=\"${base_ws}\"" >> "$ros_team_ws_file"
  echo "  RosTeamWS_DISTRO=\"${chosen_ros_distro}\"" >> "$ros_team_ws_file"
  echo "  RosTeamWS_WS_FOLDER=\"${new_workspace_location}\"" >> "$ros_team_ws_file"
  echo "  RosTeamWS_WS_DOCKER_SUPPORT=\"${docker_support}\"" >> "$ros_team_ws_file"
  echo "  RosTeamWS_DOCKER_TAG=\"${docker_image_tag}\"" >> "$ros_team_ws_file"
  echo "$source_path_rtw" >> "$ros_team_ws_file"
  echo "}" >> "$ros_team_ws_file"
  echo "alias $alias_name=$fun_name" >> "$ros_team_ws_file"

}

source_and_update_ws()
{
  if [ -z "$1" ]; then
    print_and_exit "No ros_team_ws_file given."
  fi
  local ros_team_ws_file=$1

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
  local usage="setup-ros-workspace create_workspace WS_FOLDER ROS_DISTRO"
  local use_docker=false

  check_user_input "$@"
  local chosen_ros_distro=$ros_distro
  setup_new_workspace

  local is_docker_rtw_file=false
  local ros_team_ws_file_name=".ros_team_ws_rc"
  local ros_team_ws_file="$HOME/$ros_team_ws_file_name"
  setup_ros_team_ws_file "$ros_team_ws_file" "$use_docker" "$is_docker_rtw_file"
  source_and_update_ws "$ros_team_ws_file"

  echo -e "${RTW_COLOR_NOTIFY_USER}Finished creating new workspace: Please open a new terminal and execute '$alias_name'${TERMINAL_COLOR_NC} (if you have setup auto sourcing)."
}

create_workspace_docker () {
  local usage="setup-ros-workspace-docker WS_FOLDER ROS_DISTRO"
  local use_docker=true

  # create new workspace locally
  check_user_input "$@"
  local chosen_ros_distro=$ros_distro # need to store, ros_distro gets overwritten while creating workspace...
  # check if selected chosen_ros_distro is supported, before we start creating a new workspace
  local docker_ros_distro_name
  if [[ -v "map_to_docker_ros_distro_name[$chosen_ros_distro]" ]];then
    # first get name for creating correct ros distro in docker
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

  # everything looks fine, now we can create a worksapce
  setup_new_workspace

  local is_docker_rtw_file=false
  local ros_team_ws_file_name=".ros_team_ws_rc"
  local ros_team_ws_file="$HOME/$ros_team_ws_file_name"
  setup_ros_team_ws_file "$ros_team_ws_file" "$use_docker" "$is_docker_rtw_file"

  ############################################################################
  # DOCKER PART: create corresponding docker image and map worksapce and rtw #
  ############################################################################

  # setup Dockerfile bashrc and .ros_team_ws_rc
  # and copy needed files to workspace dir
  ws_docker_folder="${new_workspace_location}/.rtw_docker_defines"
  mkdir -p "$ws_docker_folder"

  # setup .bashrc
  cp "$DOCKER_TEMPLATES/bashrc" "$ws_docker_folder/."
  # add the name of the workspace folder in docker, so that it can be inited in bashrc the first time used.
  local docker_ws_path
  docker_ws_path="${new_workspace_location}"
  sed -i "s|DUMMY_WS_FOLDER|${docker_ws_path}|g" "$ws_docker_folder/bashrc"
  echo "" >> "$ws_docker_folder/bashrc"
  echo "$alias_name" >> "$ws_docker_folder/bashrc"

  cp "$DOCKER_TEMPLATES/$docker_file" "$ws_docker_folder/Dockerfile"
  sed -i "s/UBUNTU_DUMMY_VERSION/${ubuntu_version}/g" "$ws_docker_folder/Dockerfile"
  sed -i "s/ROS_DUMMY_VERSION/${docker_ros_distro_name}/g" "$ws_docker_folder/Dockerfile"
  sed -i "s/ROS_TEAM_WS_DUMMY_BRANCH/${rtw_branch_for_ros_distro}/g" "$ws_docker_folder/Dockerfile"
  if [[ "${ubuntu_version}" == *20.04* ]]; then
    DOCKER_FILE="$ws_docker_folder/Dockerfile"
    TMP_FILE="$ws_docker_folder/.tmp_Dockerfile"

    # Add ROS 1 repositories
    mv $DOCKER_FILE "$TMP_FILE"
    TEST_LINE=`awk '$1 == "#" && $2 == "ROS2" && $3 == "repository" { print NR }' $TMP_FILE`  # get line before `# ROS2 repository`
    let CUT_LINE=$TEST_LINE-1
    head -$CUT_LINE $TMP_FILE > $DOCKER_FILE

    echo "# ROS repository" >> $DOCKER_FILE
    echo "RUN echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu \$(lsb_release -sc) main\" | tee /etc/apt/sources.list.d/ros.list > /dev/null" >> $DOCKER_FILE
    echo "" >> $DOCKER_FILE

    # Add last part
    let CUT_LINE=$TEST_LINE+0
    tail -n +$CUT_LINE $TMP_FILE >> $DOCKER_FILE

    # Add repositories for Nala
    mv $DOCKER_FILE "$TMP_FILE"
    TEST_LINE=`awk '$1 == "#" && $2 == "install" && $3 == "nala" { print NR }' $TMP_FILE`  # get line before `# install nala and upgrade` dependency
    let CUT_LINE=$TEST_LINE-0
    head -$CUT_LINE $TMP_FILE > $DOCKER_FILE

    echo "RUN apt update -y && apt install -y wget" >> $DOCKER_FILE
    echo "RUN echo \"deb [arch=amd64,arm64,armhf] http://deb.volian.org/volian/ scar main\" | tee /etc/apt/sources.list.d/volian-archive-scar-unstable.list" >> $DOCKER_FILE
    echo "RUN wget -qO - https://deb.volian.org/volian/scar.key | tee /etc/apt/trusted.gpg.d/volian-archive-scar-unstable.gpg > /dev/null" >> $DOCKER_FILE
    echo "RUN apt update -y && apt install -y nala-legacy" >> $DOCKER_FILE

    # Add last part
    let CUT_LINE=$TEST_LINE+2
    tail -n +$CUT_LINE $TMP_FILE >> $DOCKER_FILE

    # Cleanup temp files
    rm $TMP_FILE
  fi

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
  sed -i "s|DUMMY_WS_FOLDER|${docker_ws_path}|g" "$ws_docker_folder/recreate_docker.sh"

  # now we are all set for building the container
  RTW_Docker_build_docker_image "$docker_image_tag" "$ws_docker_folder/Dockerfile" || { print_and_exit "Build of docker container failed."; }

  echo ""
  echo "######################################################################################################################"
  echo -e "${RTW_COLOR_NOTIFY_USER}Finished creating new workspace with docker support: Going to switch to docker. Next time simply run '$alias_name'${TERMINAL_COLOR_NC}"
  echo "######################################################################################################################"
  sleep 2 # give user time to read above message before switching to docker container

  RTW_Docker_create_docker_container "$docker_image_tag" "${docker_ws_path}" "$chosen_ros_distro" "$docker_host_name"
}
