=======================================================
ros2_control: Setup robot's hardware package
=======================================================
.. _uc-setup-ros2-control-hardware:

This use-case describes how to set up a robot's hardware interface for the ros2_control framework using scripts from ROS Team Workspace (RosTeamWS) framework.

``ros2_control_setup-hardware-interface-package`` script accepts the file name of the robot's hardware interfacea and, optionally, class name.
The file name should use standard ROS format <my_cool_robot_hardware>.
A ``.cpp`` and ``.hpp`` files will added using this name.
If the class name is not set, it is guessed by camel-casing the file name.
The package name is obtained from the 'package.xml' file.

  .. note:: it is recommended to setup your package using :ref:`setup-new-package <uc-new-package>` script.

  .. warning:: The script **has to be executed** from the folder where the package should be generated.

The scripts copies template files from the ``templates/ros2_control/hardware`` folder, renames the files, and replaces the placeholders.
The scripts adds also a plugin description and simple test checking if the plugin can be loaded.

.. code-block:: bash
   :caption: Usage of script for setting up the robot bringup.
   :name: ros2_control_setup-hardware-interface-package

   ros2_control_setup-hardware-interface-package FILE_NAME [CLASS_NAME]


After all files are copied and placeholders set, changes are automatically staged in git.
