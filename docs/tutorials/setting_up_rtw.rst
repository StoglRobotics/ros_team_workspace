============================
Setting up RosTeamWorkspace
============================
.. _tutorial-setting-up-rtw:

Installation of RosTeamWS
""""""""""""""""""""""""""
To start using RosTeamWS framework clone the repository to any location using:

.. code-block:: bash

   git clone https://github.com/StoglRobotics/ros_team_workspace.git


Source the ``setup.bash``` in the top folder of RosTeamWorkspace:

.. code-block:: bash

   source ros_team_workspace/setup.bash

That's all. You are now set to use RosTeamWS. If you want to add auto-sourcing you can simply execute the following command:

.. code-block:: bash

   setup-auto-sourcing


This is going to configure the RosTeamWorkspace permanently. If you want to revert those changes or prefer to do them by yourself simply follow the next few steps.

Manual auto-sourcing
"""""""""""""""""""""

Add auto-sourcing of configuration to your ``.bashrc`` file by adding the following lines to its end using your favorite text editor (e.g., ``vim`` or ``nano``):

.. code-block:: bash

   if [ -f ~/.ros_team_ws_rc ]; then
       . ~/.ros_team_ws_rc
   fi

Copy ``<PATH TO ros_team_workspace>/templates/.ros_team_ws_rc`` file to your home folder using

.. code-block:: bash

   cp <PATH TO ros_team_workspace>/templates/.ros_team_ws_rc ~/


In ``~/.ros_team_ws_rc`` adjust the following values using your favorite text editor:

- ``<PATH TO ros_team_workspace>`` to point to the path to the framework folder

.. note::
  If you have ROS installed at another location than /opt/ros/<ros-distro> , please adjust the ALTERNATIVE_ROS_<ROS-DISTRO>_LOCATION variable in the ``~/.ros_team_ws_rc`` to the according location.

Now you are ready to:

- :ref:`quick-start <tutorial-quick-start>`,
- :ref:`setup your first workspace <uc-setup-workspace>`,
- check how to :ref:`manage multiple workspaces <tutorial-managing-multiple-workspaces>`,
- or check out one of the :ref:`use-cases <uc-index>`.
