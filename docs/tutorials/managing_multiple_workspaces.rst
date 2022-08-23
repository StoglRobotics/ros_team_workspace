=============================
Managing multiple workspaces
=============================
.. _tutorial-managing-multiple-workspaces:

Before learning how to manage multiple workspace with RosTeamWorkspace be sure that you have set everything up as described in this :ref:`tutorial <tutorial-setting-up-rtw>`.

Also be sure that you opened a new terminal after you setup RosTeamWorkspace to be configured permanently.


First setup a new workspace called ``ws_rolling_local`` inside a ``~/workspace`` folder.

.. code-block:: bash

   setup-ros-workspace rolling ~/workspace ws ros2c_demos

When asked for confirmation just press <ENTER>.
After a workspace is created open a new terminal and execute ``_ws_rolling_ros2c_demos`` alias for sourcing your new workspace.
Now you can use :ref:`aliases <uc-aliases>` to interact with your workspace.
Those can be used out of any folder you are in.

Let's now add a test package into your workspace.

1. Enter a workspace source folder using ``rosds`` alias.
2. Clone ``ros2_control_demos`` repository for testing:

.. code-block:: bash

   git clone https://github.com/ros-control/ros2_control_demos.git


As next let's add another workspace

<repeat the above, but add gz_ros2_control repository for testing and execute a demo there>


Now each time you open a new terminal you can use either ``_ws_rolling_ros2c_demos`` or ``_ws_rolling_gz_demos`` to source needed workspace and use the same :ref:`aliases <uc-aliases>` without constantly thinking about exact workspace/folder you are working in.
