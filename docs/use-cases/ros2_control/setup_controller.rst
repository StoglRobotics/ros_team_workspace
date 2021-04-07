=======================================================
ros2_control: Setup controller package
=======================================================
.. _uc-setup-ros2-controller:

This use-case describes how to set up a controller for the ros2_control framework using scripts from ROS Team Workspace (RosTeamWS) framework.

``ros2_control_setup-controller-package`` script accepts the file name of the controller and, optionally, class name and the package name.
The file name should use standard ROS format <my_cool_controller>.
A ``.cpp`` and ``.hpp`` files will added using this name.
If the class name is not set, it is guessed by camel-casing the file name.
If the package name is not set, it is guessed from the current path using the folder's name.
The script **has to be executed** from the folder where the package should be generated.

**Note**: it is recomended to setup your package using :ref:`setup-new-package <uc-new-package>` scritpt.

The scripts copies template files from the ``templates/ros2_control/controller`` folder, rename the files, and replaces the placeholders.
The scripts adds also a plugin description and simple test checking if the plugin can be loaded.

.. code-block:: bash
   :caption: Usage of script for setting up a controller.
   :name: ros2_control_setup-controller-package

   ros2_control_setup-controller-package FILE_NAME [CLASS_NAME] [PKG_NAME]


After all files are copied and placeholders set, a commit can be automatically created.
