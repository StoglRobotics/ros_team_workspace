==========================================
Setup robot bringup
==========================================
.. _uc-setup-robot-bringup:

This use-case describes how to set up a robot bringup package using scripts from ROS Team Workspace (RosTeamWS) framework.
The package follows as far as possible best practices for `robot support packages <http://wiki.ros.org/Industrial/Tutorials/WorkingWithRosIndustrialRobotSupportPackages>`_ from ROS-Industrial consortia. (**This has to be verified!**)


Script for Setting up Description Package
============================================

``setup-robot-bringup`` script accepts the robot name and, optionally, the package name.
If the package name is not set, it is guessed from the current path using the folder's name.
The script **has to be executed** from the folder where the bringup should be generated.

**Note**: it is recommended to setup your package using :ref:`setup-new-package <uc-new-package>` script.

The scripts copies template files from the ``templates/robot_bringup`` folder, rename the files, and replaces the placeholders.

.. code-block:: bash
   :caption: Usage of script for setting up the robot bringup.
   :name: setup-robot-bringup

   setup-robot-bringup ROBOT_NAME [PKG_NAME]


After all files are copied and placeholders set, a commit is automatically created.
