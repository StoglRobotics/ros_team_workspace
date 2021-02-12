=======================================================
Setup robot's ros2_control hardware package
=======================================================
.. _uc-setup-ros2-control-hardware:

This use-case describes how to set up a robot's hardware interface for the ros2_control framework using scripts from ROS Team Workspace (RosTeamWS) framework.

.. contents:: Table of Contents
   :depth: 2

Script for Setting up ros2_control hardware package
====================================================

``setup-robot-ros2-control-hardware.bash`` script accepts the file name of the robot's hardware interfacea and, optionally, class name and the package name.
The file name should use standard ROS format <my_cool_robot_hardware>.
A ``.cpp`` and ``.hpp`` files will added using this name.
If the class name is not set, it is guessed by camel-casing the file name.
If the package name is not set, it is guessed from the current path using the folder's name.
The script **has to be executed** from the folder where the package should be generated.

**Note**: it is recomended to setup your package using :ref:`set-new-package.bash <uc-new-package>` scritpt.

The scripts copies template files from the ``templates/ros2_control/hardware`` folder, rename the files, and replaces the placeholders.
The scripts adds also a plugin description and simple test checking if the plugin can be loaded.

.. code-block:: bash
   :caption: Usage of script for setting up the robot bringup.
   :name: setup-ros2-control-hardware

   setup-robot-ros2-control-hardware.bash FILE_NAME [CLASS_NAME] [PKG_NAME]


After all files are copied and placeholders set, a commit is automatically created.
