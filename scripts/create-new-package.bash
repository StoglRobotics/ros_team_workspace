#!/bin/bash
#
usage="create-new-package <PKG_NAME> <\"PKG_DESCRIPTION\">"

# Load Framework defines
script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $script_own_dir/../setup.bash
check_and_set_ros_distro_and_version "${ROS_DISTRO}"

PKG_NAME=$1
if [ -z "$1" ]; then
  print_and_exit "Package name is not defined. Nothing to do 😯" "$usage"
fi

PKG_DESCRIPTION=$2
if [ -z "$2" ]; then
  print_and_exit "Package description is not defined. Nothing to do 😯" "$usage"
fi

echo ""
echo -e "${TERMINAL_COLOR_USER_NOTICE}Your path is `pwd`. Is this your workspace source folder?${TERMINAL_COLOR_NC}"
echo -e "${TERMINAL_COLOR_USER_CONFIRMATION}If so press <ENTER> otherwise <CTRL>+C and start the script again from your source folder.${TERMINAL_COLOR_NC}"
read


# PKG_TYPE options
package_type_standard_option="standard"
package_type_metapackage_option="metapackage"
package_type_subpackage_option="subpackage"
packae_type_options=("$package_type_standard_option" "$package_type_metapackage_option" "$package_type_subpackage_option")

echo -n -e ""
echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}What type of package you want to create?${TERMINAL_COLOR_NC}"
select PKG_TYPE in "${packae_type_options[@]}";
do
  case "$PKG_TYPE" in
        "$package_type_standard_option")
            echo -e "${TERMINAL_COLOR_USER_NOTICE}Standard package '$PKG_NAME' will be created!${TERMINAL_COLOR_NC}"
            if [[ -d "$PKG_NAME" ]]; then
              print_and_exit "ERROR: Directory '$PKG_NAME' already exists. Nothing to do 😯" "$usage"
            fi
            CREATE_PARAMS=""
            break
          ;;
        "$package_type_metapackage_option")
            echo -e "${TERMINAL_COLOR_USER_NOTICE}Meta-package '$PKG_NAME' will be created!${TERMINAL_COLOR_NC}"
            if [[ -d "$PKG_NAME" ]]; then
              print_and_exit "ERROR: Directory '$PKG_NAME' already exists. Nothing to do 😯" "$usage"
            fi
            CREATE_PARAMS="--meta"
            mkdir $PKG_NAME
            cd $PKG_NAME
            break
          ;;
        "$package_type_subpackage_option")
            echo -e "${TERMINAL_COLOR_USER_NOTICE}Subpackage '$PKG_NAME' will be created!${TERMINAL_COLOR_NC}"
            read -p "To create a subpackage, enter the name of metapackage: " META_NAME
            if [ -z "$META_NAME" ]; then
              print_and_exit "ERROR: You have to enter the name of metapackage! Exiting..." "$usage"
            fi
            if [[ ! -d $META_NAME ]]; then
              print_and_exit "ERROR: metapackage with the name '$META_NAME' does not exist! Exiting..." "$usage"
            fi
            CREATE_PARAMS=""
            cd $META_NAME
            # TODO: read licence of the meta-package
            break
          ;;
  esac
done


# MAINTAINER_NAME and MAINTAINER_EMAIL options for a multiple choice
maintainer_info_user_input_option="user input"
maintainer_info_options=("$maintainer_info_user_input_option")

global_git_name=`git config --global user.name`
global_git_email=`git config --global user.email`
if [ -n "$global_git_name" ] && [ -n "$global_git_email" ]; then
  maintainer_info_global_git_option="global git: $global_git_name <$global_git_email>"
  maintainer_info_options+=("$maintainer_info_global_git_option")
fi

local_git_name=""; local_git_email=""
if [[ -d ".git" ]]; then
  local_git_name=`git config user.name`
  local_git_email=`git config user.email`
  if [ -n "$local_git_name" ] && [ -n "$local_git_email" ]; then
    maintainer_info_local_git_option="local git: $local_git_name <$local_git_email>"
    maintainer_info_options+=("$maintainer_info_local_git_option")
  fi
fi

function get_maintainer_name_from_input() {
  read -p "Enter the maintainer's name: " name
  while [[ -z $name ]]; do
      read -p "Name cannot be empty, please enter your name: " name
  done
  echo "$name"
}

function get_maintainer_email_from_input() {
  read -p "Enter the maintainer's email address: " email
  while [[ ! $email =~ ^[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Z|a-z]{2,}$ ]]; do
      if [[ -z $email ]]; then
          read -p "Email cannot be empty, please enter your email: " email
      else
          read -p "Invalid email format, please enter a valid email: " email
      fi
  done
  echo "$email"
}

echo ""
echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}Who will maintain the package you want to create? Please provide the info:${TERMINAL_COLOR_NC}"

# If there is only user input option ask for maintainer info directly
if [[ -z "$maintainer_info_global_git_option" && -z "$maintainer_info_local_git_option" ]]; then
  echo "No local or global git name and email found"
  MAINTAINER_NAME=$(get_maintainer_name_from_input)
  MAINTAINER_EMAIL=$(get_maintainer_email_from_input)
else
  select maintainer_info_option in "${maintainer_info_options[@]}";
  do
    case "$maintainer_info_option" in
          "$maintainer_info_user_input_option")
              MAINTAINER_NAME=$(get_maintainer_name_from_input)
              MAINTAINER_EMAIL=$(get_maintainer_email_from_input)
              break
            ;;
          "$maintainer_info_global_git_option")
              MAINTAINER_NAME=$global_git_name
              MAINTAINER_EMAIL=$global_git_email
              break
            ;;
          "$maintainer_info_local_git_option")
              MAINTAINER_NAME=$local_git_name
              MAINTAINER_EMAIL=$local_git_email
              break
            ;;
    esac
  done
fi
echo -e "${TERMINAL_COLOR_USER_NOTICE}The name '$MAINTAINER_NAME' and email address '$MAINTAINER_EMAIL' will be used as maintainer info!${TERMINAL_COLOR_NC}"


# License options for a multiple choice
license_user_input_option="user input"
licence_team_option="Current team license standard: ['$TEAM_LICENSE']"

license_apache_option="Apache-2.0"
license_apache_desc="A permissive license with an added patent grant."

license_bsl_option="BSL-1.0"
license_bsl_desc="A permissive license with a requirement to display the license."

license_bsd2_option="BSD-2.0"
license_bsd2_desc="A permissive license with a requirement to display the license and copyright."

license_bsd2c_option="BSD-2-Clause"
license_bsd2c_desc="A permissive license with a requirement to display the license and copyright."

license_bsd3c_option="BSD-3-Clause"
license_bsd3c_desc="A permissive license with a requirement to display the license and copyright."

license_gpl3_option="GPL-3.0-only"
license_gpl3_desc="A copyleft license that requires derivative works to be licensed under the same license."

license_lgpl3_option="LGPL-3.0-only"
license_lgpl3_desc="A copyleft license that allows for linking with proprietary software."

license_mit_option="MIT"
license_mit_desc="A permissive license with a requirement to display the license and copyright."

license_mit0_option="MIT-0"
license_mit0_desc="A permissive license with no requirements."

license_options=("$license_user_input_option" "$licence_team_option" "$license_apache_option" "$license_bsl_option" "$license_bsd2_option" "$license_bsd2c_option")
license_options+=("$license_bsd3c_option" "$license_gpl3_option" "$license_lgpl3_option" "$license_mit_option" "$license_mit0_option")

echo ""
echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}How do you want to licence your package? ${TERMINAL_COLOR_NC}"
select licence_option in "${license_options[@]}";
do
  case "$licence_option" in
        "$license_user_input_option")
            read -p "Enter your licence: " LICENSE
            licence_description=""
            break
          ;;
        "$licence_team_option")
            LICENSE="$TEAM_LICENSE"
            licence_description=""
            break
          ;;
        "$license_apache_option")
            LICENSE="$license_apache_option"
            licence_description="$license_apache_desc"
            break
          ;;
        "$license_bsl_option")
            LICENSE="$license_bsl_option"
            licence_description="$license_bsl_desc"
            break
          ;;
        "$license_bsd2_option")
            LICENSE="$license_bsd2_option"
            licence_description="$license_bsd2_desc"
            break
          ;;
        "$license_bsd2c_option")
            LICENSE="$license_bsd2c_option"
            licence_description="$license_bsd2c_desc"
            break
          ;;
        "$license_bsd3c_option")
            LICENSE="$license_bsd3c_option"
            licence_description="$license_bsd3c_desc"
            break
          ;;
        "$license_gpl3_option")
            LICENSE="$license_gpl3_option"
            licence_description="$license_gpl3_desc"
            break
          ;;
        "$license_lgpl3_option")
            LICENSE="$license_lgpl3_option"
            licence_description="$license_lgpl3_desc"
            break
          ;;
        "$license_mit_option")
            LICENSE="$license_mit_option"
            licence_description="$license_mit_desc"
            break
          ;;
        "$license_mit0_option")
            LICENSE="$license_mit0_option"
            licence_description="$license_mit0_desc"
            break
          ;;
  esac
done
echo -e "${TERMINAL_COLOR_USER_NOTICE}The licence '$LICENSE' will be used! ($) $licence_description ${TERMINAL_COLOR_NC}"


# BUILD_TYPE
ros2_build_type_ament_cmake_option="ament_cmake"
ros2_build_type_ament_python_option="ament_python"
ros2_build_type_cmake_option="cmake"
ros2_build_type_options=("$ros2_build_type_ament_cmake_option" "$ros2_build_type_ament_python_option" "$ros2_build_type_cmake_option")

if [[ $ros_version == 1 ]]; then
    BUILD_TYPE="catkin"
elif [[ $ros_version == 2 ]]; then
  echo ""
  echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}Please choose your package build type:${TERMINAL_COLOR_NC}"
  select build_type in "${ros2_build_type_options[@]}";
  do
    case "$build_type" in
          "$ros2_build_type_ament_cmake_option")
              BUILD_TYPE="ament_cmake"
              break
            ;;
          "$ros2_build_type_ament_python_option")
              BUILD_TYPE="ament_python"
              break
            ;;
          "$ros2_build_type_cmake_option")
              BUILD_TYPE="cmake"
              break
            ;;
    esac
  done
fi

echo ""
echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}ATTENTION: Creating package '$PKG_NAME' in '`pwd`' with description '$PKG_DESCRIPTION', licence '$LICENSE', build type '$BUILD_TYPE' and maintainer '$MAINTAINER_NAME <$MAINTAINER_EMAIL>'${TERMINAL_COLOR_NC}"
echo -e "${TERMINAL_COLOR_USER_CONFIRMATION}If correct press <ENTER>, otherwise <CTRL>+C and start the script again from your source folder.${TERMINAL_COLOR_NC}"
read

if [[ $ros_version == 1 ]]; then
  catkin_create_pkg $CREATE_PARAMS $PKG_NAME -V 0.0.1 -l "$LICENSE" -D "$PKG_DESCRIPTION"
  catkin b -c
  source ~/.bashrc
elif [[ $ros_version == 2 ]]; then
  ros2 pkg create --package-format 3 --description "$PKG_DESCRIPTION" --license "$LICENSE" --build-type "$BUILD_TYPE" --maintainer-email "$MAINTAINER_EMAIL" --maintainer-name "$MAINTAINER_NAME" $PKG_NAME

  ## Until it is corrected upstream
  if [[ "$PKG_TYPE" == "$package_type_metapackage_option" ]]; then
    cd $PKG_NAME
    rm -r include
    rm -r src
    head -3 CMakeLists.txt > CMakeLists1.txt
    mv CMakeLists1.txt CMakeLists.txt
    echo "find_package(ament_cmake REQUIRED)" >> CMakeLists.txt
    echo "ament_package()" >> CMakeLists.txt
    sed -i '/test_depend/s/.*/ /' package.xml
    cd ../..
  fi
fi

echo ""
user_decision "Do you want to setup/update repository with the new package configuration?"
if [[ " ${positive_answers[*]} " =~ " ${user_answer} " ]]; then
  choice="y"
else
  choice="n"
fi

case "$choice" in
"y")
  if [[ "$PKG_TYPE" != "$package_type_subpackage_option" ]]; then

    if [[ $ros_version == 1 ]]; then
      roscd $PKG_NAME
    elif [[ $ros_version == 2 ]]; then
      cd $PKG_NAME
    fi

    $RosTeamWS_FRAMEWORK_SCRIPTS_PATH/setup-repository.bash $PKG_NAME "$PKG_DESCRIPTION" $LICENSE

  else

    if [[ $ros_version == 1 ]]; then

      roscd $META_NAME
      sed -i 's/<buildtool_depend>catkin<\/buildtool_depend>/<buildtool_depend>catkin<\/buildtool_depend>\n  <depend>'${PKG_NAME}'<\/depend>/g' package.xml
      cd ..

    elif [[ $ros_version == 2 ]]; then

      cd $META_NAME
      append_to_string="<buildtool_depend>ament_cmake<\/buildtool_depend>"
      sed -i "s/$append_to_string/$append_to_string\\n\\n  <exec_depend>${PKG_NAME}<\/exec_depend>/g" package.xml
      cd ..

    fi

    ## Move this from here into setup-repository.bash script

    # Add package to CI
    append_to_string="          #package-name:"
    sed -i "s/$append_to_string/$append_to_string\\n            #${PKG_NAME}/g" .github/workflows/ci-build.yml
    # Add also if source build is uncommented
    append_to_string='          package-name:'
    sed -i "s/$append_to_string/$append_to_string\\n            ${PKG_NAME}/g" .github/workflows/ci-build.yml
    append_to_string='        package-name:'
    sed -i "s/$append_to_string/$append_to_string\\n          ${PKG_NAME}/g" .github/workflows/ci-lint.yml

    # Add short description to README.md
    append_to_string='### Packages in `'"${META_NAME}"'` metapackage'
    sed -i "s/$append_to_string/$append_to_string\\n* **${PKG_NAME}** - ${PKG_DESCRIPTION}/g" README.md

    # Setup also subpackage README.md
    head -4 $PACKAGE_TEMPLATES/README.md.github >> ${PKG_NAME}/README.md
    sed -i 's/\$NAME\$/'${PKG_NAME}'/g' ${PKG_NAME}/README.md
    sed -i 's/\$DESCRIPTION\$/'"${PKG_DESCRIPTION}"'/g' ${PKG_NAME}/README.md

    read -p "Do you want to create a commit with your changes? (y/n) [n]: " choice
    choice=${choice:="n"}

    case "$choice" in
    "y")
      git add .
      git commit -m "RosTeamWS: Created sub-package $PKG_NAME."
      ;;
    #"n")
    esac

  fi
  ;;
"n")
  echo -e "${TERMINAL_COLOR_USER_NOTICE}Repository configuration is _not_ updated!${TERMINAL_COLOR_NC}"
esac
