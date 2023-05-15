============================
ROS-Robot Package structure
============================

This document proposes guidelines for the structure of ROS packages in larger details than provided by `ROS Enhancement Proposals (REPs) <https://github.com/ros-infrastructure/rep>`_

**NOTE**: All the proposal here are the resuls of authors' personal experiences. Saying that, if you don't like some of it, you are free to change what you want and need (and hopefully propose it as PR).

Package structure for Robot support in ROS
------------------------------------------

The here down proposed architecture tries to split the robot's files to minimize per-package dependencies.

.. code:: text

  <manufacturer>_<robot_name>[_<type>]/                     # Repository
  ├── <manufacturer>[_<robot_name>[_<type>]|_manipulators]/       # Meta-package of the robot
  │   ├── CMakeLists.txt
  │   └── package.xml
  ├── <robot_name>_bringup/                              # Launch and config files for starting the robot using ros2_control
  │   ├── [CMakeLists.txt]                               # if ament_cmake is used (recommended)
  │   ├── package.xml
  │   ├── [setup.py]                                     # if ament_python is used
  │   ├── [setup.cfg]                                    # if ament_python is used
  │   ├── config/
  │   │   ├── <robot_name>_controllers.yaml              # Controllers' configuration for ros2_control
  │   │   ├── <robot_name>_forward_position_publisher.yaml  # Setup test publisher for forward position controller
  │   │   └── <robot_name>_joint_trajectory_publisher.yaml  # Setup test publisher for joint trajectory controller
  │   └── launch/
  │       ├── <robot_name>.launch.py                     # Robot's default launch file
  │       ├── test_forward_position_controller.launch.py # Start test publisher for forward position controller
  │       └── test_joint_trajectory_controller.launch.py # Start test publisher for joint trajectory controller
  ├── <manufacturer|robot_name>_description/             # Robot's description files
  │   ├── [CMakeLists.txt]                               # if ament_cmake is used (recommended)
  │   ├── package.xml
  │   ├── [setup.py]                                     # if ament_python is used
  │   ├── [setup.cfg]                                    # if ament_python is used
  │   ├── config/                                        # general YAML files for a robot
  │   │   └── <robot_name>_<someting_specific>.yaml
  │   ├── launch/                                        # launch files related to testing robots' description
  │   │   └── test_<robot_name>_description.launch.py
  │   ├── meshes/                                        # meshes used in <robot_name>_macro.urdf.xacro
  │   │   ├── collision
  │   │   │   └── <robot_name|robot_model>               # meshes are sorted by robot name or model
  │   │   │       ├── <link_xy>.stl
  │   │   │       └── ...
  │   │   └── visual
  │   │       └── <robot_name|robot_model>
  │   │           ├── <link_xy>.dae
  │   │           └── ...
  │   ├── rviz/                                          # rviz display configurations
  │   │   └── <robot_name>_default.rviz
  │   └── urdf/                                          # URDF file for the robot
  │       ├── common.xacro                               # Common XACRO definitions
  │       ├── <robot_name>.urdf.xacro                    # Main URDF for a robot - loads macro and other files
  │       └── <robot_name|robot_model>
  │           ├── <robot_name>_macro.xacro               # Macro file of the robot - can add prefix, define origin, etc.
  │           └── <robot_name>_macro.ros2_control.xacro  # URDF-part used to configure ros2_control
  └── <robot_name>_hardware_interface/                # Implementation of the ros2_control interface
  │   ├── CMakeLists.txt
  │   ├── package.xml
  │   ├── include/
  │   │   └── <robot_name>_hardware_interface/
  │   │       └── <robot_name>_<something_specific>.hpp
  │   └── src/
  │       └── <robot_name>_<something_specific>.cpp
  └── <manufacturer>_controllers/                 # Implementation of hardware specific controllers
      ├── CMakeLists.txt
      ├── package.xml
      ├── include/
      │   └── <manufacturer>_controllers/
      │       └── <robot_specific_controller>.hpp
      └── src/
          └── <robot_specific_controller>.cpp
