===========================
Setup a new Workspace
===========================
.. _uc-setup-workspace:

This use-case describes how to setup a new ROS workspace using the ``setup-ros-workspace`` command from the ROS Team Workspace (RosTeamWS) framework. The command is used as follows:

.. code-block:: bash
   :caption: Usage of the script for setting up a new ROS workspace.
   :name: setup-workspace

   setup-ros-workspace ROS_DISTRO "WS_FOLDER" WS_PREFIX WS_SUFFIX


The script creates in the ``$HOME/<WS_FOLDER>`` a new ROS workspace with name ``ros_ws_<WS_PREFIX>_<ROS_DISTRO>_<WS_SUFFIX>``.
If you want to omit any of input parameters use ``"-"`` as argument.
Default value for ``<WS_FOLDER>`` is "workspace".

Parameter description:
***********************
.. csv-table:: setup-ros-workspace parameters overview
   :header: "Parameter", "Description", "Optional", "Default"
   :widths: 10 70 10 10

   "ROS_DISTRO", "The ros distribution you want to use.", "No. Needs to be selected.", ""
   "WS_FOLEDER", "Workspace folder relative to `$Home`.", "Yes. Omitted with -", "``$Home\workspace``"
   "WS_PREFIX", "Prefix for the workspace name.", "Yes. Omitted with -", ""
   "WS_SUFFIX", "Suffix for the workspace name.", "Yes. Omitted with -", ""

The distribution you pass as ``ROS_DISTRO`` needs to be installed under ``/opt/ros/<distribition>``.
If you want to create nested folders you have to pass them in double quotes like:

.. code-block:: bash
   :caption: Example usage with nested workspace folder.
   :name: setup-workspace-nested

   setup-ros-workspace rolling "my_workspace/folder/nested" - -
