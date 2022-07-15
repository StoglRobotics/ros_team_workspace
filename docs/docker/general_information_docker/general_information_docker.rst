==============================
General information on docker
==============================
.. _general-info-on-docker-index:

Generally you can always have a look at the `docs of docker <https://docs.docker.com/>`_.

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
