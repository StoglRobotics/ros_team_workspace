==============================
RosTeamWS CLI (Experimental)
==============================

RosTeamWS has a command line interface (CLI) that provides a set of commands to manage ROS workspaces incl. Docker workspaces.
The CLI is designed to have a more user-friendly overview of the available RTW commands and to provide a more intuitive way to interact with RTW.

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
   For more detailed usage of each command or sub-command, run ``rtw <command> -h`` or ``rtw <command> <sub-command> -h``.
