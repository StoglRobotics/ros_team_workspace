==================================
Quick start with RosTeamWorkspace
==================================
.. _tutorial-quick-start:

This tutorial shows use of RosTeamWorkspace for very common use-cases that can be done without permanent changes to your environment.
First you have to clone and source the workspace and then continue with the use-case you are interested in:

.. toctree::
   :maxdepth: 1


Clone and source the RosTeamWorkspace
---------------------------------------
.. code-block:: bash

   git clone https://github.com/StoglRobotics/ros_team_workspace.git
   source ros_team_workspace/setup.bash
   setup-auto-sourcing  # Make RosTeamWorkspace automatically sourced when open a new terminal (The best experience)


Create new package in an existing workspace
--------------------------------------------------------
For more details check :ref:`use-case description <uc-new-package>`.


.. code-block:: bash

   source <path to your ROS workspace>/install/setup.bash
   cd <src folder of your ROS workspace>

   create-new-package <my_new_package_name> <"Some cool description of the package.">  # follow the instructions and remember to set a license

   cd .. && colcon build --symlink-install  # to compile your newly created package


Create robot description package
-------------------------------------------------
For more details check :ref:`use-case description <uc-setup-robot-description>`.

requirement: have a <my_cool_robot_description_package_name> package to hold the robot description

.. code-block:: bash

   source <path to your ROS workspace>/install/setup.bash
   cd <src folder of your ROS workspace>/<my_cool_robot_description_package_name>

   setup-robot-description <my_cool_robot_name>


Create robot bringup package
-----------------------------------------------
For more details check :ref:`use-case description <uc-setup-robot-bringup>`.

requirement: have a <my_cool_robot_bringup_package_name> package to hold the robot bringup

.. code-block:: bash

   source <path to your ROS workspace>/install/setup.bash
   cd <src folder of your ROS workspace>/<my_cool_robot_bringup_package_name>

   setup-robot-bringup <my_cool_robot_name> <my_cool_robot_description_package_name>


Setup  ros2_control control hardware
-------------------------------------------------
For more details check :ref:`use-case description <uc-setup-ros2-control-hardware>`.

requirement: have a <my_cool_robot_control_package_name> package to hold the robot's ros2_control hardware interface

.. code-block:: bash

   source <path to your ROS workspace>/install/setup.bash
   cd <src folder of your ROS workspace>/<my_cool_robot_control_package_name>

   ros2_control_setup-hardware-interface-package <my_cool_robot_hardware> [<MyCoolRobotHW>]


Setup  ros2_control controller
-----------------------------------------------
For more details check :ref:`use-case description <uc-setup-ros2-controller>`.

requirement: have a <my_cool_robot_controller_package_name> package to hold the robot's ros2_control controller

.. code-block:: bash

   source <path to your ROS workspace>/install/setup.bash
   cd <src folder of your ROS workspace>/<my_cool_robot_controller_package_name>

   ros2_control_setup-controller-package <my_controller_file_name>
