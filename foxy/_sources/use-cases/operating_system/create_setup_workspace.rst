===========================
Setup a new Workspace
===========================
.. _uc-setup-workspace:

This use-case describes how to setup a new ROS workspace using scripts from the ROS Team Workspace (RosTeamWS) framework.

``setup-ros-workspace`` accepts ros distro name workspace suffix and workspace folder as parameters.
All three parameters are optional.

.. code-block:: bash
   :caption: Usage of the script for setting up a new ROS workspace.
   :name: setup-workspace

   setup-ros-workspace ROS_DISTRO WS_FOLDER WS_PREFIX WS_SUFFIX

The script creates in the ``$HOME/<WS_FOLDER>`` a new ROS workspace with name ``ros_ws_<WS_PREFIX>_<ROS_DISTRO>_<WS_SUFFIX>``.
If you want to omit any of input parameters use ``"-"`` as argument.
Default value for ``<WS_FOLDER>`` is "workspace".
