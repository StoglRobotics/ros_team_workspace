==========================================
Setup ROS 2 node
==========================================
.. _uc-setup-ros-node:

This use-case describes how to set up a new ROS 2 node in an existing package using scripts from ROS Team Workspace (RosTeamWS) framework.

  .. note:: currently only C++ nodes and ``ament_cmake`` packages are supported.


Script for Setting up a ROS 2 Node
============================================

``setup-ros-node`` script accepts the file name, and optionally, list of dependencies and class name.
Dependencies are added to ``package.xml`` and ``CMakeLists.txt`` file.

  .. note:: it is recommended to setup your package using :ref:`setup-new-package <uc-new-package>` script.

  .. warning:: The script **has to be executed** from inside the package where the files should be generated.

The scripts copies template files from the ``templates/nodes`` folder, renames the files, and replaces the placeholders.

.. code-block:: bash
   :caption: Usage of script for setting up the robot bringup.
   :name: setup-robot-bringup

   setup-ros-node FILE_NAME [PKG_DEPS] [CLASS_NAME]


After the files are successfully generated follow the instruction for test-launch of the node.
