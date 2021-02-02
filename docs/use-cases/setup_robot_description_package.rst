==========================================
Setup robot description
==========================================
.. _uc-setup-robot-description:

This use-case describes how to set up a robot description package using scripts from ROS Team Workspace (RosTeamWS) framework.
The package follows as far as possible best practices for `robot support packages <http://wiki.ros.org/Industrial/Tutorials/WorkingWithRosIndustrialRobotSupportPackages>`_ from ROS-Industrial consortia. (**This has to be verified!**)

.. contents:: Table of Contents
   :depth: 2

Script for Setting up Description Package
============================================

``setup-robot-description.bash`` script accepts the robot name and, optionally, the package name.
If the package name is not set, it is guessed from the current path using the folder's name.
The script **has to be executed** from the folder where the description should be generated.

**Note**: it is recomended to setup your package using :ref:`set-new-package.bash <uc-new-package>` scritpt.

The scripts copies template files from the ``templates/robot_description`` folder, rename the files, and replaces the placeholders.

.. code-block:: bash
   :caption: Usage of script for setting up the robot description.
   :name: setup-robot-description

   setup-robot-description.bash ROBOT_NAME [PKG_NAME]


After all files are copied and placeholders set, a commit is automatically created.
