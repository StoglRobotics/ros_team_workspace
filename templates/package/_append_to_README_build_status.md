
## Build status



### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `src/$NAME$/$NAME$-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `src/$NAME$/$NAME$.repos`

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.


## Using the repository
Skip any of below steps is not applicable.

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
   vcs import src --input src/$NAME$/$NAME$.<ros-distro>.repos
   rosdep install --ignore-src --from-paths src -y -r       # install also is there are unreleased packages
   cd ..
   ```

### Configure and Build Workspace:
To configure and build workspace execute following commands:
  ```
  cd $COLCON_WS
  colcon build --symlink-install --mixin rel-with-deb-info compile-commands ccache
  ```

## Running Executable
```
ros2 launch $NAME$
```
