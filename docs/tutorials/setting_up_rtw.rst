============================
Setting up RosTeamWorkspace
============================
.. _tutorial-setting-up-rtw:

To start using RosTeamWS framework clone the repository to any location using:

.. code-block:: bash

   git clone https://github.com/StoglRobotics/ros_team_workspace.git


Source the ``setup.bash``` in the top folder of RosTeamWorkspace:

.. code-block:: bash

   source ros_team_workspace/setup.bash


Execute the following to configure the ROS Team Workspace permanently.

Add auto-sourcing of configuration to your ``.bashrc`` file by adding following lines to its end using your favorite text editor (e.g., ``vim`` or ``nano``):

.. code-block:: bash

   if [ -f ~/.ros_team_ws_rc ]; then
       . ~/.ros_team_ws_rc
   fi

Copy ``templates/.ros_team_ws_rc`` file to your home folder using

.. code-block:: bash

   cp ros_team_workspace/templates/.ros_team_ws_rc ~/


and adjust the following values using your favorite text editor:

- ``<PATH TO ros_team_workspace>`` - with a path to the framework folder

Now you are ready to

- :ref:`setup your first workspace <uc-setup-workspace>`
- :ref:`check next tutorial <tutorial-managing-multiple-workspaces>`
- or :ref:`other use-cases <uc-index>`.
