=======================================================
ros2_control: Setup controller package
=======================================================
.. _uc-setup-ros2-controller:

This use-case describes how to set up a controller for the ros2_control framework using scripts from ROS Team Workspace (RosTeamWS) framework.
The scripts uses template files from ``templates/ros2_control/controller`` folder.
The script creates a full skeleton of a controller with plugin description and tests for loading controller and checking its basic functionality.

  .. note:: it is recommended to setup your package using :ref:`setup-new-package <uc-new-package>` script.

  .. warning:: The script **has to be executed** from the folder where the package should be generated.

Usage
------

.. code-block:: bash
   :caption: Usage of script for setting up a controller.
   :name: ros2_control_setup-controller-package

   ros2_control_setup-controller-package FILE_NAME [CLASS_NAME]


Parameters:

  - ``FILE_NAME`` file name used for controller's ``.cpp`` and ``.hpp`` files.
    It assumes standard ROS format, e.g, "my_cool_controller".

  - ``CLASS_NAME`` optional name used for controller class.
    If not set, it is guessed by camel-casing the file name.


The script will ask for some additional input.

The package name is obtained from the 'package.xml' file.

After all files are copied and placeholders set, changes are automatically staged in git.
