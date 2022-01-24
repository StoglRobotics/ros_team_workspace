==========================================
Setup robot description
==========================================
.. _uc-setup-robot-description:

This use-case describes how to set up a robot description package using scripts from ROS Team Workspace (RosTeamWS) framework.
The package follows as far as possible best practices for `robot support packages <http://wiki.ros.org/Industrial/Tutorials/WorkingWithRosIndustrialRobotSupportPackages>`_ from ROS-Industrial consortia. (**This has to be verified!**)


Script for Setting up Description Package
============================================

``setup-robot-description`` script accepts the robot name and, optionally, the package name.
If the package name is not set, it is guessed from the current path using the folder's name.
The script **has to be executed** from the package folder where the description should be generated.

**Note**: it is recommended to setup your package using :ref:`create-new-package <uc-new-package>` script.

The scripts copies template files from the ``templates/robot_description`` folder, rename the files, and replaces the placeholders.

.. code-block:: bash
   :caption: Usage of script for setting up the robot description.
   :name: setup-robot-description

   setup-robot-description ROBOT_NAME [PKG_NAME]


After all files are copied and placeholders set, a commit is automatically created.

To test the generated files compile and source your workspace and execute:

.. code-block:: bash
   :caption: Test generated files.
   :name: test-generated-files

   ros2 launch <PKG_NAME>  view_<ROBOT_NAME>.launch.py
