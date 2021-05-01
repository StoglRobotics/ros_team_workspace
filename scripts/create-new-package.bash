#!/bin/bash
#
usage='create-new-package PKG_NAME PKG_DESCRIPTION'

# Load Framework defines
script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $script_own_dir/../setup.bash
check_ros_distro ${ROS_DISTRO}

PKG_NAME=$1
if [ -z "$1" ]; then
  print_and_exit "Package name is not defined. Nothing to do, exiting..." "$usage"
fi

PKG_DESCRIPTION=$2
if [ -z "$2" ]; then
  print_and_exit "Package description is not defined. Nothing to do, exiting..." "$usage"
fi

echo ""
echo "Your path is `pwd`. Is this your workspace source folder?"
read -p "If so press <ENTER> otherwise <CTRL>+C and start the script again from your source folder."

read -p "What type of package you want to create? (0 - standard, 1 - metapackage, 2 - subpackage) [0]:" META
META=${META:=0}

CREATE_PARAMS=""

case "$META" in
"1")
   echo "Meta-package '$PKG_NAME' will be created!"
    CREATE_PARAMS="--meta"
    mkdir $PKG_NAME
    cd $PKG_NAME
   ;;
"2")
   read -p "To create a subpackage, enter the name of metapackage: " META_NAME
   if [ -z "$META_NAME" ]; then
    echo "ERROR: You have to enter the name of metapackage! Exiting..."
    exit;
   fi
   if [[ ! -d $META_NAME ]]; then
     echo "ERROR: metapackage with the name '$META_NAME' does not exist! Exiting..."
   fi
   echo "Subpackage '$PKG_NAME' will be created in the metapackage '$META_NAME'!"
   cd $META_NAME
   # TODO: read licence of the meta-package
   ;;
*)
  echo "Package '$PKG_NAME' will be created!"
esac


read -p "Do you want to enter name and email address of the maintainer? If not, data from git configuration will be used. (y/n) [n]: " choice
choice=${choice:="n"}

case "$choice" in
"y")
  read -p "Enter the maintainer's name: " MAINTAINER_NAME
  read -p "Enter the maintainer's email address: " MAINTAINER_EMAIL
  ;;
"n")
  MAINTAINER_NAME=`git config user.name`
  MAINTAINER_EMAIL=`git config user.email`
  if [[ ! -d ".git" ]]; then
    cd ..
    git init >> /dev/null # init git to get data from git command
    MAINTAINER_NAME=`git config user.name`
    MAINTAINER_EMAIL=`git config user.email`
    rm -r .git/
    cd - >> /dev/null
  fi
esac

# License
read -p "How do you want to licence your package? ['$TEAM_LICENSE']: " LICENSE
LICENSE=${LICENSE:=$TEAM_LICENSE}

# Build type
read -p "Please choose your package build type (1 - ament_cmake, 2 - ament_python, 3 - cmake) [1]:" choice

case "$choice" in
"2")
   BUILD_TYPE="ament_python"
   ;;
"3")
   BUILD_TYPE="cmake"
   ;;
*)
  BUILD_TYPE="ament_cmake"
esac

echo ""
echo "ATTENTION: Creating package '$PKG_NAME' in '`pwd`' with description '$PKG_DESCRIPTION', licence '$LICENSE', build type '$BUILD_TYPE' and maintainer '$MAINTAINER_NAME <$MAINTAINER_EMAIL>'"
read -p "If correct press <ENTER>, otherwise <CTRL>+C and start the script again from your source folder."


if [[ $ros_version == 1 ]]; then
  catkin_create_pkg $CREATE_PARAMS $PKG_NAME -V 0.0.1 -l "$LICENSE" -D "$PKG_DESCRIPTION"
  catkin b -c
  source ~/.bashrc
elif [[ $ros_version == 2 ]]; then
  ros2 pkg create --package-format 3 --description "$PKG_DESCRIPTION" --license "$LICENSE" --build-type "$BUILD_TYPE" --maintainer-email "$MAINTAINER_EMAIL" --maintainer-name "$MAINTAINER_NAME" $PKG_NAME

  ## Until it is corrected upstream
  if [[ $META == 1 ]]; then
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

read -p "Do you want to setup/update repository with the new package configuration? (y/n) [n]: " choice
choice=${choice:="n"}

case "$choice" in
"y")
  if [[ $META != 2 ]]; then

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
  echo "Repository configuration is _not_ updated!"
esac
