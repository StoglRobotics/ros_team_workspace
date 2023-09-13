=============================
Managing multiple workspaces
=============================
.. _tutorial-managing-multiple-workspaces:

Before learning how to manage multiple workspace with RosTeamWorkspace be sure that you have set everything up as described in the :ref:`setting up RosTeamWorkspace tutorial <tutorial-setting-up-rtw>`.

Also be sure that you opened a new terminal after you setup RosTeamWorkspace to be configured permanently.


First setup a new workspace called ``ws_rolling_ros2c_demos`` inside a ``~/workspace`` folder.

.. code-block:: bash

   setup-ros-workspace ~/workspace/ws_rolling_ros2c_demos rolling

When asked for confirmation just press <ENTER>.
After a workspace is created, open a new terminal and execute ``_ws_rolling_ros2c_demos`` alias for sourcing your new workspace. You can then switch to the new sourced workspace with ``rosd``.
Now you can use :ref:`aliases <uc-aliases>` to interact with your workspace.
Those can be used out of any folder you are in.

Let's now add a test package into your workspace.

1. Make sure your workspace is sourced by executing ``_ws_rolling_ros2c_demos``. Then enter the source folder using ``rosds`` alias.
2. Clone ``ros2_control_demos`` repository for testing either:

.. code-block:: bash

   git clone git@github.com:ros-controls/ros2_control_demos.git # SSH
   # or if no ssh key is setup:
   git clone https://github.com/ros-controls/ros2_control_demos.git # HTTPS

3. Go to the base of your workspace:

.. code-block:: bash

   rosd

4. Install dependencies to be sure we got everything:

.. code-block:: bash

    rosdep install --from-paths src -y -i -r

.. note:: if ``rosdep`` command fails with a comment that binary packages can not be found by apt, try to update your rosdep index using ``rosdep update`` command or even your package index using ``sudo apt update``.

5. You can then build your workspace using the alias for colcon build:

.. code-block:: bash

    cb

Everything should now have been built successfully!

Next let's add another workspace

.. code-block:: bash

   setup-ros-workspace ~/workspace/ws_rolling_gz_demos rolling

Now repeat the above steps to add `gz_ros2_control <https://github.com/ros-controls/gz_ros2_control>`_ repository for testing and execute a demo from there.

.. note:: if ``rosdep`` commands fails with a comment on ros-rolling-ros-gz-sim not being installed successfully, maybe force the desired gazebo version with e.g. ``export GZ_VERSION=fortress``

Now each time you open a new terminal you can use either ``_ws_rolling_ros2c_demos`` or ``_ws_rolling_gz_demos`` to source needed workspace and use the same :ref:`aliases <uc-aliases>` without constantly thinking about exact workspace/folder you are working in.
