=====================
Configure Repository
=====================
.. _uc-configure-repo:

Setup Repository CI configuration (GitHub)
===========================================

``setup-repository-ci`` script is used, accepting the github repository name and github user/organization namespace as parameters.

  .. warning:: The script **has to be executed** from the *main* folder of your package.

.. code-block:: bash
   :caption: Usage of the script for setting up new packages.
   :name: setup-package-ci

   setup-repository-ci "repo_name" "repo_namespace" ["first_package second_package ..."]

When executing the script, read all output carefully.
If you make any wrong decision or enter incorrect data, use <CTRL>+C keys to terminate the script.

The script can add setup for multiple ROS 2 versions.
Simply follow the output.

After a setup is created, you should go through the files and check if they are correct.
Please remember that each workflow you would like to run on schedule has to be in the default branch of your repository.
Then try to push new files to a GitHub-repository and open a PR.
You should then already see new workflows active.


Details about CI setup and created workflows
---------------------------------------------
Code formatters and linters
,,,,,,,,,,,,,,,,,,,,,,,,,,,,
For executing formatters, ``pre-commit`` program is used and automatically setup.
To use it in a cloned repository (if there is ``.pre-commit-conifg.yaml`` file), execute once ``pre-commit install``.
Clang-Format is used as the main code formatting program.


Building packages
,,,,,,,,,,,,,,,,,,
This script creates three stages of build configuration for each ROS 2 version.
Each of those stages test different compatibility levels where users can experience issues with actively developed repositories and ensures future compatibility of the repository.
In the following each stage is explained.

``binary``
  Building against released packages in a ROS distribution. The stage ensures that isolated builds on a local machine and build-farm are possible. Unreleased dependencies can be defined in ``.repos``-file with suffix ``-not-released.<ros-distro>.repos``. This functionality is useful for new projects where packages are located in multiple repositories and when packages are released in a new ROS distribution (and you are not releasing regularly to ``rolling``).
  The ``binary`` build has two workflows to build against ``main`` and ``testing`` ROS debian repositories. This is useful when for example API changes are done that involve multiple packages. Then it is expected that build against binary packages in the ``main`` repository fails, but if all the changes are released, building against ``testing`` repository has to work. Otherwise it means that the code has some regressions and cannot be synced to the ``main`` repositories (This is what is happening after maintainers from OpenRobotics announce a "Sync" or a ROS distribution).
  Scheduled ``binary`` build warns you that your package is incompatible with released dependencies (``main``) or that a package is breaking changes that will be released (``testing``). The first case should actually never happen if you are using all three building stages.

``semi-binary``
  Building against released core packages in a ROS distribution, but the main dependencies are build from source. The dependencies are defined in ``.repos``-file with suffix ``.<ros-distro>.repos``. This functionality is useful during development process when you have to sync changes in multiple repositories which are not released yet. This is the stage which should never fail if you are keeping your code up to date. Scheduled ``semi-binary`` builds warn you that some important dependency has changed something in their code so you can prepare yourself better for the next release.
  There are two workflows building against ``main`` and ``testing`` ROS repositories with released packages.

``source``
  Core ROS packages are build from source. Expects the ``.repos`` files for ROS core and the ``.repos``-file with suffix ``.<ros-distro>.repos`` for the package-specific dependencies. This stage makes mostly sense when working with ``rolling`` since other distribution of ROS should be API stable once released. This stage helps to anticipate breaking changes in the ROS API that will be relevant the mid-term future.


For the better overview and to avoid confusion for users, there are two different tables with the workflow overview.
One in the README file showing only the ``binary`` and ``semi-binary`` builds against ``main`` ROS repository, and another in the ``.github/workflows/README.md`` that shows all the workflows.
