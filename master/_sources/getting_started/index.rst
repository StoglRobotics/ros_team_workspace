=================
Getting Started
=================

To start using RosTeamWS framework clone the repository to any location using:

.. code-block:: bash

   git clone https://github.com/StoglRobotics/ros_team_workspace.git


Than source the ``setup.bash``` in the top folder:

.. code-block:: bash

   source ros_team_workspace/setup.bash


Execute the following to configure the ROS Team Workspace permanently.

Add auto-sourcing of configuration to your ``.bashrc`` file by adding following lines to its end:

.. code-block:: bash

   if [ -f ~/.ros_team_ws_rc ]; then
       . ~/.ros_team_ws_rc
   fi

Copy ``templates/.ros_team_ws_rc`` file to your home folder and adjust the following values:

- ``<PATH TO ros_team_workspace>`` - with a path to the framework folder
- ``ADD_HERE_INTERFACE_NAME`` - add name of usually-used network interface on your computer


Now checkout :ref:`use-cases <uc-index>` for how-to-use descriptions.
