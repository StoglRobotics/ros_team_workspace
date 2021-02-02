=====================
ROS Package structure
=====================

This documents proposes guidelines for structure of ROS packages in larger detail than provided by `ROS Enhancement Proposals (REPs) <https://github.com/ros-infrastructure/rep>`_

**NOTE**: All the proposal here are the resuls of authors' personal experiences. Saying that, if you don't like some of it, you are free to change what you want and need (and hopefully propose it as PR).

Package structure for Robot support in ROS
------------------------------------------

Here proposed architecture try to split the robot's files to minimize per-package dependencies.

.. code:: text

  <manufacturer>_<robot_name>[_<type>]/                     # Repository
  ├── <manufacturer>[_<robot_name>[_<type>]|_manipulators]/       # Meta-package of the robot
  │   ├── CMakeLists.txt
  │   └── package.xml
  ├── <robot_name>_bringup/                     # Launch and config files of the robot
  │   ├── [CMakeLists.txt]
  │   ├── package.xml
  │   ├── [setup.py]
  │   ├── config/
  │   │   └── <config>.yaml
  │   └── launch/
  │       └── <something>.launch.py
  ├── <manufacturer>_description/                 # Robot's description files
  │   ├── [CMakeLists.txt]
  │   ├── package.xml
  │   ├── [setup.py]
  │   ├── config/
  │   │   └──
  │   ├── meshes/
  │   │   └── collision
  │   │       ├── <link_xy>.stl
  │   │       └── ...
  │   │   └── visual
  │   │       ├── <link_xy>.dae
  │   │       └── ...
  │   ├── rviz/
  │   │   └── <robot_name>_default.rviz
  │   └── urdf/
  │       ├── <robot_name>.urdf.xacro
  │       ├── <robot_name>_macro.ros2_control.xacro
  │       └── <robot_name>_macro.xacro
  └── <robot_name>_hardware_interface/          # Implementation of the ros2_control interface
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
