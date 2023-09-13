==========================================
Setup robot description
==========================================
.. _uc-setup-robot-description:

This use-case describes how to set up a robot description package using scripts from ROS Team Workspace (RosTeamWS) framework.
The package follows as far as possible best practices for `robot support packages <http://wiki.ros.org/Industrial/Tutorials/WorkingWithRosIndustrialRobotSupportPackages>`_ from ROS-Industrial consortia. (**This has to be verified!**)


Script for Setting up Description Package
============================================

``setup-robot-description`` script accepts the robot name.
The package name is obtained from the 'package.xml' file.

  .. note:: it is recommended to setup your package using :ref:`create-new-package <uc-new-package>` script.

  .. warning:: The script **has to be executed** from the folder where the package should be generated.

The scripts copies template files from the ``templates/robot_description`` folder, renames the files, and replaces the placeholders.

.. code-block:: bash
   :caption: Usage of script for setting up the robot description.
   :name: setup-robot-description

   setup-robot-description ROBOT_NAME


After all files are copied and placeholders set, changes are automatically staged in git.

To test the generated files compile and source your workspace and execute:

.. code-block:: bash
   :caption: Test generated files.
   :name: test-generated-files

   ros2 launch <PKG_NAME>  view_<ROBOT_NAME>.launch.py
