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

Follow the instructions in the ``README.md`` inside the ``rtwcli`` folder.


How to use the CLI
""""""""""""""""""""

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


Setting up a new workspace from ``.repos`` files
"""""""""""""""""""""""""""""""""""""""""""""""""

PR `#169 <https://github.com/RosTeamWS/RosTeamWS/pull/169>`_ introduced a new
feature to create a new workspace (local or dockerized) from ``.repos`` files,
streamlining the setup process for complex projects with multiple repositories.

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
      * ``--repos-branch``: Branch of the repository containing the ``.repos``
        files

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
