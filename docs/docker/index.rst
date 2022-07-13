===========
Docker
===========
.. _docker:

RosTeamWS supports docker container.

.. note::
  If you want to forward a xsession from docker (e.g. rviz2), you have to install xhost.

Use Cases with docker support
-------------------------------

.. list-table:: Overview of currently supported use cases
   :widths: auto
   :header-rows: 1
   :stub-columns: 1

   * - use case
     - docker command
     - description
   * - :ref:`Setup a new Workspace<uc-setup-workspace>` with ``setup-ros-workspace``
     - ``setup-ros-workspace-docker``
     - Creates a new workspace and maps the workspace inside a docker container. You can then switch to docker using ``rtw_switch_to_docker``.

General info on docker
-------------------------------
Generally you can always have look at the `docs of docker <https://docs.docker.com/>`_.

Installation
""""""""""""""""
You have to install docker which is dependent on the operating system you are using.

*   `Windows <https://docs.docker.com/desktop/windows/install/>`_
*   `Mac <https://docs.docker.com/desktop/mac/install/>`_
*   `Linux <https://docs.docker.com/engine/install/>`_: it depends.

.. note::
  However make sure your user is in the docker group. Check with: ``groups`` command. To add your user to the docker group run: ``sudo usermod -aG docker <username>``.

Usful commands
""""""""""""""""
For complete list of commands have a look at `official docker cli reference <https://docs.docker.com/engine/reference/commandline/cli/>`_.

* ``docker container <command>``:

    * ``ls`` lists all current active containers
    * ``ls -a`` lists all containers
    * ``rm <container>`` removes container

* ``docker image <command>``:

    * ``ls`` lists all images
    * ``rm <image>`` removes image
