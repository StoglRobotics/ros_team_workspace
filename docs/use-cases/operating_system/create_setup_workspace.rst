===========================
Setup a new Workspace
===========================
.. _uc-setup-workspace:

This use-case describes how to setup a new ROS workspace using scripts from the ROS Team Workspace (RosTeamWS) framework. Besides the creation of a local workspace, creation of a Ubuntu based docker container is supported. For more details have a look below in the `Docker workspace`_ section.

Local workspace
----------------

``setup-ros-workspace`` accepts ros distro name workspace suffix and workspace folder as parameters.
All three parameters are optional.

.. code-block:: bash
   :caption: Usage of the script for setting up a new ROS workspace.
   :name: setup-workspace

   setup-ros-workspace ROS_DISTRO WS_FOLDER WS_PREFIX WS_SUFFIX

The script creates in the ``$HOME/<WS_FOLDER>`` a new ROS workspace with name ``ros_ws_<WS_PREFIX>_<ROS_DISTRO>_<WS_SUFFIX>``.
If you want to omit any of input parameters use ``"-"`` as argument.
Default value for ``<WS_FOLDER>`` is "workspace".

.. _uc-setup-docker-workspace:

Docker workspace
------------------

.. code-block:: bash
   :caption: How to setup a workspace inside a docker container.
   :name: setup docker workspace

   setup-ros-workspace-docker ROS_DISTRO WS_FOLDER WS_PREFIX WS_SUFFIX.

Like the script ``setup-ros-workspace`` the ``setup-ros-workspace-docker`` creates a new local ROS workspace. Then a Ubuntu docker container gets built and the created workspace is mounted inside the docker container under the home directory. The first time a docker container is build can take quite a while. When the first build is finished you are directly connected as user inside the container. You can verify this by checking if your hostname has changed. To exit a container simply type the ``exit`` command.

Reconnect to a container
""""""""""""""""""""""""""

If you exited a container an want to reconnect as a user, you have to run the ``rtw_switch_to_docker`` command. However before executing this you have to source your workspace with ``_<workspace_alias_command>``. If you want to connect as a root user you can execute ``rtw_switch_to_docker_root``.

Recreate a container
""""""""""""""""""""""

If you removed an image,  you can recreate it by switching into the ``.rtw_docker_defines`` folder inside your workspace and then executing the ``.\build_docker_image`` command. After the container is rebuilt, you can create it with ``.\create_docker_container`` command. Thereafter, you should be able to normally start, reconnect and stop the container.

.. note::
  For more general questions on the usage of docker and the limitations of RosTeamWS in interacting with docker have a look at our :ref:`docker related docs<docker-overview>`.

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
