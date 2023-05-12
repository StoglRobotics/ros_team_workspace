
# Quick Start Instructions

If you are familiar with ROS 2, here are the quick-and-dirty build instructions.

  ```
  cd $COLCON_WS
  git clone git@github.com:$NAMESPACE$/$NAME$.git src/$NAME$
  vcs import src --input src/$NAME$/$NAME$.<ros-distro>.repos
  rosdep install --ignore-src --from-paths src -y -r
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release    # Faster and more efficient build type
  cd ..
  ```
If you end up with missing dependencies, install them using commands from [Setup ROS Workspace](#setup-ros-workspace) section.

# How to use this Package and ROS Introduction

 - [Install and Build](#install-and-build)
   * [Install ROS $Ros_distro$ and Development Tooling](#install-ros-$ros_distro$-and-development-tooling)
   * [Setup ROS Workspace](#setup-ros-workspace)
   * [Configure and Build Workspace](#configure-and-build-workspace)
 - [Running Executables](#running-executables)
   * [Using the Local Workspace](#using-the-local-workspace)
 - [Testing and Linting](#testing-and-linting)
 - [Creating a new ROS 2 Package](#creating-a-new-ros2-package)
 - [References](#references)

## Install and Build

### Install ROS $Ros_distro$ and Development Tooling

These instructions assume you are running Ubuntu 20.04:

1. [Install ROS 2 $Ros_distro$](https://index.ros.org/doc/ros2/Installation/$Ros_distro$/Linux-Install-Debians/).
   You can stop following along with the tutorial after you complete the section titled: [Environment setup](https://index.ros.org/doc/ros2/Installation/$Ros_distro$/Linux-Install-Debians/#environment-setup).
   Make sure you setup your environment with:
   ```
   source /opt/ros/$ros_distro$/setup.bash
   ```

   > **NOTE:** You may want to add that line to your `~/.bashrc`

   > **NOTE:** There is also a `zsh` version of the setup script.

1. [Install ROS 2 Build Tools](https://index.ros.org/doc/ros2/Installation/$Ros_distro$/Linux-Development-Setup/#install-development-tools-and-ros-tools).
   You do not need to build ROS 2 from source.
   Simply install the tooling under the section titled "Install development tools and ROS tools".

1. Install `ccache`:
   ```
   sudo apt install ccache
   ```

1. Setup `colcon mixin` [Reference](https://github.com/colcon/colcon-mixin-repository) for convenience commands.
   ```
   sudo apt install python3-colcon-mixin
   colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
   colcon mixin update default
   ```

### Setup ROS Workspace

1. Create a colcon workspace:
   ```
   export COLCON_WS=~/workspace/ros_ws_foxy
   mkdir -p $COLCON_WS/src
   ```

   > **NOTE:** Feel free to change `~/workspace/ros_ws_foxy` to whatever absolute path you want.

   > **NOTE:** Over time you will probably have multiple ROS workspaces, so it makes sense to them all in a subfolder.
     Also, it is good practice to put the ROS version in the name of the workspace, for different tests you could just add a suffix to the base name `ros_ws_foxy`.

1. Download the required repositories and install package dependencies:
   ```
   cd $COLCON_WS
   git clone git@github.com:$NAMESPACE$/$NAME$.git src/$NAME$
   vcs import src --input src/$NAME$/$NAME$.<ros_distro>.repos
   rosdep install --ignore-src --from-paths src -y -r       # install also is there are unreleased packages
   ```

   Sometimes packages do not list all their dependencies so `rosdep` will not install everything.
   If you are getting missing dependency errors, try manually install the following packages:
   ```
   sudo apt install ros2-foxy-forward_command_controller ros2-foxy-joint_state_broadcaster ros2-foxy-joint_trajectory_controller ros2-foxy-xacro
   ```

### Configure and Build Workspace:
To configure and build workspace execute following commands:
  ```
  cd $COLCON_WS
  colcon build --symlink-install --mixin rel-with-deb-info compile-commands ccache
  ```

## Running Executable

See `README.md` files of the packages for information regarding running executables.

<Add here some concrete data about current repository>

### Using the Local Workspace

To use the local workspace you have to source it by using local setup script:
  ```
  source $COLCON_WS/install/local_setup.bash
  ```
Since there are many errors one unintentionally do with wrong sourcing, please check also [Notes on Sourcing ROS Workspace](#notes-on-sourcing-ros-workspace).

#### Notes on Sourcing ROS Workspace

Sourcing of a workspace appends the binary and resource directories to appropriate environment variables.
It is important that you do not run the build command in the same terminal that you have previously sourced your local workspace.
This can cause dependency resolution issues.
Here is some advice copied from [Official ROS Workspace Tutorial](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/) on this:

Before sourcing the overlay, it is very important that you open a new terminal, separate from the one where you built the workspace.
Sourcing an overlay in the same terminal where you built, or likewise building where an overlay is sourced, may create complex issues.

Sourcing the local_setup of the overlay will only add the packages available in the overlay to your environment.
`setup` sources the overlay as well as the underlay it was created in, allowing you to utilize both workspaces.

So, sourcing your main ROS 2 installation’s setup and then the dev_ws overlay’s local_setup, like you just did, is the same as just sourcing dev_ws’s setup, because that includes the environment of the underlay it was created in.


## Testing and Linting

To test the packages packages built from source, use the following command with [colcon](https://colcon.readthedocs.io/en/released/).
In order to run tests and linters you will have had to already built the workspace.
To run the tests use following commands:
  ```
  cd $COLCON_WS
  colcon test
  colcon test-result
  ```

There are `--mixin` arguments that can be used to control testing with linters, specifically `linters-only` and `linters-skip`.

## Creating a new ROS 2 Package

If you need to create a new ROS 2 package it is helpful to start with the official boilerplate for a ROS 2 package.
The command `ros2 pkg` can be used to generate the boilerplate details.
For example to create a new ROS 2 package called `example_package` with a node called `example_node` and library called `example_library` use this command:
  ```
  ros2 pkg create --build-type ament_cmake --node-name example_node --library-name example_library example_package
  ```

## References

Here are some useful references for developing with ROS 2:

 - [Official ROS 2 Tutorials](https://index.ros.org/doc/ros2/Tutorials/)
   * [Luanchfile](https://index.ros.org/doc/ros2/Tutorials/Launch-Files/Creating-Launch-Files/)
   * [Package](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/)
   * [Parameters](https://index.ros.org/doc/ros2/Tutorials/Parameters/Understanding-ROS2-Parameters/)
   * [Workspace](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/)
 - [Example ROS packages](https://github.com/ros2/examples)
 - [Colcon Documentation](https://colcon.readthedocs.io/en/released/#)
 - [ROS 2 Design Documentation](https://design.ros2.org/)
 - [ROS 2 Launch Architecture](https://github.com/ros2/launch/blob/master/launch/doc/source/architecture.rst)
