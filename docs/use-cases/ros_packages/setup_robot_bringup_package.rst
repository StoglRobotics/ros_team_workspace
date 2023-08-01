==========================================
Setup robot bringup
==========================================
.. _uc-setup-robot-bringup:

This use-case describes how to set up a robot bringup package using scripts from ROS Team Workspace (RosTeamWS) framework.
The package follows as far as possible best practices for `robot support packages <http://wiki.ros.org/Industrial/Tutorials/WorkingWithRosIndustrialRobotSupportPackages>`_ from ROS-Industrial consortia. (**This has to be verified!**)


Script for Setting up a Bringup Package
============================================

``setup-robot-bringup`` script accepts the robot name and the robot description package name.
The package name is obtained from the 'package.xml' file.

  .. note:: it is recommended to setup your package using :ref:`setup-new-package <uc-new-package>` script.

  .. warning:: The script **has to be executed** from the folder where the package should be generated.

The scripts copies template files from the ``templates/robot_bringup`` folder, renames the files, and replaces the placeholders.

.. code-block:: bash
   :caption: Usage of script for setting up the robot bringup.
   :name: setup-robot-bringup

   setup-robot-bringup ROBOT_NAME DESCRIPTION_PKG_NAME


After all files are copied and placeholders set, changes are automatically staged in git.
