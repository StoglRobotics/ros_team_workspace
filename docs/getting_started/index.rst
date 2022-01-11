=================
Getting Started
=================

Configuration of your ros_team_workspace
******************************************
To start using RosTeamWS framework clone the repository to any location using:

.. code-block:: bash

   git clone https://github.com/StoglRobotics/ros_team_workspace.git


Than source the ``setup.bash``` in the top folder:

.. code-block:: bash

   source ros_team_workspace/setup.bash


This is going to configure the ROS Team Workspace permanently.

Auto-sourcing
**************
If you want to add auto-sourcing of the configuration copy the ``ros_team_workspace/templates/.ros_team_ws_rc`` file to your home folder and adjust the following values:

- ``<PATH TO ros_team_workspace>`` - with a path to the framework folder
- ``ADD_HERE_INTERFACE_NAME`` - add name of usually-used network interface on your computer


You then have to append the following lines in to your ``.bashrc`` file:

.. code-block:: bash

   if [ -f ~/.ros_team_ws_rc ]; then
       . ~/.ros_team_ws_rc
   fi
This will look for the ``.ros_team_ws_rc`` inside your home directory and if present source it.

What next?
************   
Now checkout :ref:`use-cases <uc-index>` for how-to-use descriptions.
