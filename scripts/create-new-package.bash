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

echo -n -e ""
echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}What type of package you want to create? (1 - standard, 2 - metapackage, 3 - subpackage):${TERMINAL_COLOR_NC}"
select META in standard metapackage subpackage;
do
  case "$META" in
        standard)
            META="1"
            break
          ;;
        metapackage)
            META="2"
            break
          ;;
        subpackage)
            META="3"
            break
          ;;
  esac
done

CREATE_PARAMS=""

case "$META" in
"2")
   echo -e "${TERMINAL_COLOR_USER_NOTICE}Meta-package '$PKG_NAME' will be created!${TERMINAL_COLOR_NC}"
    if [[ -d "$PKG_NAME" ]]; then
      print_and_exit "ERROR: Directory '$PKG_NAME' already exists. Nothing to do 😯" "$usage"
    fi
    CREATE_PARAMS="--meta"
    mkdir $PKG_NAME
    cd $PKG_NAME
   ;;
"3")
   read -p "To create a subpackage, enter the name of metapackage: " META_NAME
   if [ -z "$META_NAME" ]; then
     print_and_exit "ERROR: You have to enter the name of metapackage! Exiting..." "$usage"
   fi
   if [[ ! -d $META_NAME ]]; then
     print_and_exit "ERROR: metapackage with the name '$META_NAME' does not exist! Exiting..." "$usage"
   fi
   echo -e "${TERMINAL_COLOR_USER_NOTICE}Subpackage '$PKG_NAME' will be created in the metapackage '$META_NAME'!${TERMINAL_COLOR_NC}"
   cd $META_NAME
   # TODO: read licence of the meta-package
   ;;
*)
  echo -e "${TERMINAL_COLOR_USER_NOTICE}Package '$PKG_NAME' will be created!${TERMINAL_COLOR_NC}"
esac


# MAINTAINER_NAME and MAINTAINER_EMAIL options for a multiple choice
maintainer_info_user_input_option="user input"
maintainer_info_options=("$maintainer_info_user_input_option")

global_git_name=`git config --global user.name`
global_git_email=`git config --global user.email`
if [ -n "$global_git_name" ] && [ -n "$global_git_email" ]; then
  maintainer_info_global_git_option="global git: $global_git_name, $global_git_email"
  maintainer_info_options+=("$maintainer_info_global_git_option")
fi

local_git_name=""; local_git_email=""
if [[ -d ".git" ]]; then
  local_git_name=`git config user.name`
  local_git_email=`git config user.email`
  if [ -n "$local_git_name" ] && [ -n "$local_git_email" ]; then
    maintainer_info_local_git_option="local git: $local_git_name, $local_git_email"
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


# License
echo -n -e "${TERMINAL_COLOR_USER_INPUT_DECISION}How do you want to licence your package? Current team license standard:['$TEAM_LICENSE']: ${TERMINAL_COLOR_NC}"
read LICENSE
LICENSE=${LICENSE:=$TEAM_LICENSE}

if [[ $ros_version == 1 ]]; then
    BUILD_TYPE="catkin"

elif [[ $ros_version == 2 ]]; then
  # Build type
  echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}Please choose your package build type (1 - ament_cmake, 2 - ament_python, 3 - cmake):${TERMINAL_COLOR_NC}"
  select build_type in ament_cmake ament_python cmake;
  do
    case "$build_type" in
          ament_cmake)
              BUILD_TYPE="ament_cmake"
              break
            ;;
          ament_python)
              BUILD_TYPE="ament_python"
              break
            ;;
          cmake)
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
  if [[ $META == 2 ]]; then
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

user_decision "Do you want to setup/update repository with the new package configuration?"
if [[ " ${positive_answers[*]} " =~ " ${user_answer} " ]]; then
  choice="y"
else
  choice="n"
fi

case "$choice" in
"y")
  if [[ $META != 3 ]]; then

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
