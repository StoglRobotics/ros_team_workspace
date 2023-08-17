=====================
RTW Aliases
=====================
.. _uc-aliases:

Standard RTW Aliases
=====================

Standard RTW aliases are configured out-of-the-box and they enable you to use the same command independently of the workspace you are in and help you hide ``colcon`` complexities and minimize errors when interacting with it.
The aliases set for each workspace (see :ref:`ṁanaging multiple workspaces <tutorial-managing-multiple-workspaces>`) setup environment variables so that following aliases can interact with that specific workspace.


Explanation of Standard Aliases
--------------------------------
The standard aliases are callable from any folder as long as a workspace is sourced using ``_<workspace_name>`` alias.

.. note:: To make this reading short, Dr. Denis' favorite aliases are ``cb``, ``ca`` and ``crm`` - simply start with using those.


Entering Workspace Folders
^^^^^^^^^^^^^^^^^^^^^^^^^^^
rosd
  Enter the root directory of the ROS workspace.
  Alias for command: ``cd $ROS_WS``

rosds
  Enter ``src`` directory of the ROS workspace.
  Alias for command: ``cd $ROS_WS/src``

rosdb
  Enter ``build`` directory of the ROS workspace.
  Alias for command: ``cd $ROS_WS/build``

rosdi
  Enter ``install`` directory of the ROS workspace.
  Alias for command: ``cd $ROS_WS/install``


Building Packages in Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Every alias has optional package names. If none is provided the whole workspace is built.

cb [package1_name, package2_name]
  Building named packages or the whole workspace.
  Alias for command: ``colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo``.

cbup [package1_name, package2_name]
  Building all packages up to the named packages (build all dependencies).
  Alias for command: ``colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-up-to``.

cbd [package1_name, package2_name]
  Building named packages or the whole workspace as *Debug* build type.
  Alias for command: ``colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug``.

cbr [package1_name, package2_name]
  Building named packages or the whole workspace as *Release* build type.
  Alias for command: ``colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release``.


Executing Package Tests
^^^^^^^^^^^^^^^^^^^^^^^^
Every alias has optional package names. If none is provided the whole workspace is built.

ct [package1_name, package2_name]
  Testing named packages or the whole workspace.
  Alias for command: ``colcon test``.

ctup [package_name]
  Testing named packages up to the named package (build all dependencies).
  Alias for command: ``colcon test --packages-up-to``.

ctres [package_name]
  Get test results for the whole workspace or a package. If using package, the output will be ``grep``-processed to filter-out the package you are looking for.
  Alias for command: ``colcon test-result --all``


Executing Multiple Commands
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Every alias has optional package names. If none is provided the whole workspace is built.

ca [package1_name, package2_name]
  Build, test and show test results for the named packages or the whole workspace.

caup [package_name]
  Build, test and show test results for the package and all its dependencies.


Cleaning Workspace
^^^^^^^^^^^^^^^^^^^

crm [package1_name, package2_name]
  Remove ``build``, ``log`` and ``install`` folders for the workspace or corresponding sub-folders for specific packages.


Defining Your Own Aliases
===========================

**TBD**

Idea: sharing aliases with your team that are very specific for your use-cases.
