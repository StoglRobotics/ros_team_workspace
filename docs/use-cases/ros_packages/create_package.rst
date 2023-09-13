=====================
Create New Package
=====================
.. _uc-new-package:

This use-case describes how to create a new package using scripts from the ROS Team Workspace (RosTeamWS) framework.


General Script for Creating Packages
=====================================

``create-new-package`` script is used, accepting the package name and description as parameters.

  .. warning:: The script **has to be executed** from the *source* folder of your workspace.

.. code-block:: bash
   :caption: Usage of script for setting up new packages.
   :name: create-package

   create-new-package NAME DESCRIPTION

When executing the script, read all output carefully.
If you make any wrong decision or enter incorrect data, use <CTRL>+C keys to terminate the script.

The script supports the following options and opportunities for data entry:

  #. Type of the package (standard, metapackage, or subpackage) - multiple choice

     - If subpackage, then you will need to enter the name of its metapackage

  #. License (string) - choose a license to add into package.xml - multiple choice
  #. Maintainer info: name and email (user input, local git, global git) - multiple choice
  #. Package build type (ament_cmake, ament_python, cmake) - multiple choice
  #. Create/Update repository with CI configuration - (y/n)


After a package is created, you can choose to configure or update the repository.
When a new package or metapackage is created, the ``setup-repository.bash`` script is called.
Check :ref:`here <uc-configure-repo>` for its documentation.
