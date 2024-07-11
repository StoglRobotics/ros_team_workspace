==============================
RosTeamWS CLI (Experimental)
==============================

RosTeamWS has a command line interface (CLI) that provides a set of commands to
manage ROS workspaces incl. Docker workspaces. The CLI is designed to have a
more user-friendly overview of the available RTW commands and to provide a more
intuitive way to interact with RTW.

.. warning::
   The CLI is still in the experimental stage and may have bugs or issues.
   Please report any bugs or issues to the RosTeamWS GitHub repository.


How to install the CLI
""""""""""""""""""""""""
.. _rtwcli-setup:

Follow the instructions in the ``README.md`` inside the ``rtwcli`` folder.


How to use the CLI
""""""""""""""""""""
.. _rtwcli-usage:

The CLI currently supports the following commands:

* ``rtw workspace``: Various workspace related sub-commands
   * ``create``: Create a new ROS workspace (local or dockerized)
   * ``use``: Select and source an existing ROS workspace
   * ``port``: Port existing RTW workspace(s)

* ``rtw docker``: Various Docker related sub-commands
   * ``enter``: Enter a Docker container for a dockerized workspace

* ``rtw ws``: Alias for ``rtw workspace use``

.. note::
   For more detailed usage of each command or sub-command, run
   ``rtw <command> -h`` or ``rtw <command> <sub-command> -h``.


Setting up a new workspace
"""""""""""""""""""""""""""""
.. _rtwcli-setup-workspace:

PR `#169 <https://github.com/StoglRobotics/ros_team_workspace/pull/169>`_ introduced a new feature to create a new local or dockerized workspace.
The workspace can additionally be created using ``.repos`` files in your repository, streamlining the setup process for complex projects with multiple repositories.

.. important::
   **From May 2024** If you want to setup a dockerized workspace with nvidia support based on Ubuntu 24.04 (for Jazzy and Rolling) - make sure to use the updated ``rocker`` from `PR #279 <https://github.com/osrf/rocker/pull/279>`_. Until this PR is merged you are encoruged to setup the rocker with:

   .. code-block:: bash

      pip3 uninstall rocker   # is you have installed it with `sudo` use it here too
      git clone https://github.com/StoglRobotics-forks/rocker.git --branch try_24
      cd rocker && pip3 install -e . && cd -

* Usage:
   * ``rtw workspace create``
     ``--ws-folder <path_to_workspace>``
     ``--ros-distro <distribution>``
     ``[options]``
   * Main ``[options]``:
      * ``--docker``: Create workspace(s) in a docker environment
      * ``--repos-containing-repository-url <url>``: URL of the repository
        containing the ``.repos`` files
        * Main ws repos format: ``{repo_name}.{ros_distro}.repos``
        * Upstream ws repos format: ``{repo_name}.{ros_distro}.upstream.repos``
      * ``--repos-branch <branch>``: Branch of the repository containing the
        ``.repos`` files

* Example:

.. code-block:: bash

   rtw workspace create \
      --ws-folder dummy_ws \
      --ros-distro humble \
      --docker \
      --repos-containing-repository-url \
         git@github.com:StoglRobotics/sr_dummy_packages.git \
      --repos-branch dummy_demo_pkg
..

   * This command will create a new dockerized workspace named ``dummy_ws``
     with ROS distribution ``humble`` using the ``.repos`` files from the
     repository ``sr_dummy_packages`` on branch ``dummy_demo_pkg``.

* Example of a ``standalone`` workspace and ``robot`` user:

.. code-block:: bash

   rtw workspace create \
      --ws-folder dummy_ws \
      --ros-distro humble \
      --docker \
      --repos-containing-repository-url \
         git@github.com:StoglRobotics/sr_dummy_packages.git \
      --repos-branch dummy_demo_pkg \
      --standalone \
      --user-override-name robot
..

   * This command will create a new dockerized standalone workspace named
     ``dummy_ws`` with ROS distribution ``humble`` using the
     ``.repos`` files from the repository ``sr_dummy_packages`` on branch
     ``dummy_demo_pkg``.

     However, for exporting the workspace docker image, the commit command must
     be executed first:

     .. code-block:: bash

         docker commit rtw_dummy_ws_final-instance rtw_dummy_ws_export

     When importing the workspace docker image, the following command must be
     executed:

     .. code-block:: bash

         rtw workspace import \
            --ws-name dummy_import_ws \
            --ros-distro humble \
            --standalone-docker-image rtw_dummy_ws_export \
            --user-override-name robot

     The ``--user-override-name`` flag is necessary to create the user with
     the same name as the one used in the exported workspace.

.. important::
   After PC restart, the ``.xauth`` cookie file will be removed. Therefore,
   before attaching VSCode, execute ``rtw ws <ws-name>`` and
   ``rtw docker enter`` to create the necessary ``.xauth`` cookie file.

.. note::
   After creating a new dockerized workspace, the rocker will start interactive
   bash session in the container.

   Only after exiting the container, the
   corresponding workspace config will be saved.

   This is done due to the fact that the setting up of the rocker container
   fails often.
