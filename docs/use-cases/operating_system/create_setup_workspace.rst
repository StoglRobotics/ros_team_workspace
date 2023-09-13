===========================
Setup a new Workspace
===========================
.. _uc-setup-workspace:

This use-case describes how to setup a new ROS workspace using scripts from the ROS Team Workspace (RosTeamWS) framework. Besides the creation of a local workspace, creation of a Ubuntu-based docker container is supported. For more details have a look below in the `Docker workspace`_ section.

Local workspace
----------------

The ``setup-ros-workspace`` command creates a new workspace in the current folder. If you supply a path relative to your home directory, the workspace is created there.

.. code-block:: bash
   :caption: Usage of the script for setting up a new ROS workspace.
   :name: setup-workspace

   setup-ros-workspace WS_FOLDER ROS_DISTRO

All parameters are optional but they are positional. If you want to omit any of the input parameters use ``"-"`` as argument.
The script creates a new ROS workspace with name ``<WS_FOLDER>`` in ``<CURRENT_WORKING_DIRECTORY>/<WS_FOLDER>``. However if you pass a folder name relative to your ``$HOME`` directory, like for example: ``setup-ros-workspace ~/<WS_FOLDER> ROS_DISTRO``, then a new workspace is created inside ``$HOME/<WS_FOLDER>``. The passed ROS distribution ``<ROS_DISTRO>`` is sourced as base if no other workspace is sourced. After the workspace has been created, you have to open a new terminal and can then type the new created alias ``_<WS_FOLDER>``. This is going to source the new workspace and you can then switch to its root|source|install|build folders by executing respectively ``rosd|rosds|rosdi|rosdb``.

Example:

.. code-block:: bash
   :caption: Example usage of the script for setting up a new ROS workspace.
   :name: example-setup-workspace

   setup-ros-workspace subfolder/my_new_workspace rolling

Creates in the current folder a subfolder ``subfolder`` and then creates a new workspace called ``my_new_workspace``.  Opening a new terminal and executing ``_my_new_workspace`` is going to source this workspace.

Default value for ``<WS_FOLDER>`` is "workspace".

.. _uc-setup-docker-workspace:

Docker workspace
------------------

.. code-block:: bash
   :caption: How to setup a workspace inside a docker container.
   :name: setup docker workspace

   setup-ros-workspace-docker WS_FOLDER ROS_DISTRO

Like the ``setup-ros-workspace`` script, the ``setup-ros-workspace-docker`` script creates a new local ROS workspace. Then a Ubuntu docker container is built and the new created workspace is mounted inside the docker container under the same directory as in the host. The first time a docker container is built can take quite a while. When the first build is finished you are directly connected as user inside the container. You can verify this by checking if your hostname has changed. To exit a container simply type the ``exit`` command.

Reconnect to a container
""""""""""""""""""""""""""

If you exited a container and want to reconnect as a user, you have to run the ``rtw_switch_to_docker`` command. However before executing this, you have to source your workspace with the alias  ``_<WS_FOLDER>``. If you want to connect as a root user, you can execute ``rtw_switch_to_docker_root``.

Recreate a container
""""""""""""""""""""""

If you removed an image,  you can recreate it by switching into the ``.rtw_docker_defines`` folder inside your workspace and then executing the ``./recreate_docker.sh`` command. After the process is finished you should be able to normally start, reconnect and stop the container.

.. note::
  For more general questions on the usage of docker and the limitations of RosTeamWS in interacting with docker, have a look at our :ref:`docker related docs<docker-overview>`.

CLI overview
"""""""""""""

.. list-table:: Overview of the cli
   :widths: auto
   :header-rows: 1
   :stub-columns: 1

   * - command
     - description
   * - ``setup-ros-workspace-docker``
     - Creates a new workspace and maps the workspace inside a docker container. You can then switch to docker using ``rtw_switch_to_docker``.
   * - ``rtw_switch_to_docker``
     - Starts the docker container if it has been stopped and connects to the container as user.
   * - ``rtw_switch_to_docker_root``
     - Starts the docker container if it has been stopped and connects to the container as root-user.
   * - ``rtw_stop_docker``
     - Stops the docker container.
