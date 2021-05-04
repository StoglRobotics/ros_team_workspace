=======================================================
ros2_control: Setup controller package
=======================================================
.. _uc-setup-ros2-controller:

This use-case describes how to set up a controller for the ros2_control framework using scripts from ROS Team Workspace (RosTeamWS) framework.
The scripts uses template files from ``templates/ros2_control/controller`` folder.
The script creates a complete skeleton of a controller with plugin description and tests for loading controller and checking its basic functionality.

**Note**: it is recomended to setup your package using :ref:`setup-new-package <uc-new-package>` scritpt.

**IMPORTANT**: The script has to be executed from the folder where files should be generated.

Usage
------

.. code-block:: bash
   :caption: Usage of script for setting up a controller.
   :name: ros2_control_setup-controller-package

   ros2_control_setup-controller-package FILE_NAME [CLASS_NAME] [PKG_NAME]


Parameters:

  - ``FILE_NAME`` file name used for controller's ``.cpp`` and ``.hpp`` files.
    It assumes standard ROS format, e.g, "my_cool_controller".

  - ``CLASS_NAME`` optional name used for controller class.
    If not set, it is guessed by camel-casing the file name.

  - ``PKG_NAME`` name of the controller's package.
    If not set, it is guessed from the current path using the folder's name.
