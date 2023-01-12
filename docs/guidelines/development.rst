=======================
Development Guidelines
=======================
.. _guidelines-development:

This documents proposes development guidelines to increase work efficiency and synergy within the rest of the team.

**NOTE**: All the proposal here are the results of authors' personal experiences. Saying that, if you have any idea to make them better you are very wellcome to create a PR.


1. Use *pre-commit* for formatting and linting
===============================================

``pre-commit`` is a program that adds hooks into ``git`` so when you commit something actions can be automatically executed.
There are many different possibility with ``pre-commit`` but it is mostly used for integrating code linters and formatters to always commit clean code.

Getting started with *pre-commit*
------------------------------------

First install it to you computer using:

   .. code-block:: bash

      pip install pre-commit


and than tell ``git`` that you are using it by executing the following in each repository:

   .. code-block:: bash

      pre-commit install

This will execute ``pre-commit`` on every commit you make and prevent it from doing it if some checks fail.
If you really need to ignore check in the commit, you can add ``-n`` flag to the ``git commit`` command and they will be skipped.


**NOTE**: If your repository does not uses ``pre-commit`` yet, it is very easy to add it by creating a configuration file ``.pre-commit-config.yaml`` in the top level of your repository (there where ``.git`` folder is).
If you don't know where to start with *pre-commit* configuration in your ROS project, simply copy our template from `templates/package/.pre-commit-config.yaml <https://github.com/StoglRobotics/ros_team_workspace/blob/master/templates/package/.pre-commit-config.yaml>`_.


Useful commands and options
----------------------------

* Update all the hooks in the configuration

  .. code-block:: bash

     pre-commit autoupdate


* Manually run and check status of all *pre-commit* hooks:

  .. code-block:: bash

     pre-commit run -a

* Remove *pre-commit*-hooks from automatic execution:

  .. code-block:: bash

     pre-commit uninstall
