=====================
Configure Repository
=====================
.. _uc-configure-repo:

Setup Repository CI configuration (GitHub)
===========================================

``setup-repository-ci`` script is used, accepting the package name and description as parameters.
The script **has to be executed** from the *main* folder of your package.

.. code-block:: bash
   :caption: Usage of script for setting up new packages.
   :name: setup-package-ci

   setup-repository-ci "repo_name" "repo_namespace" ["first_package" "second_package" ...]

When executing the script, read all output carefully.
If you make any wrong decision or enter incorrect data, use <CTRL>+C keys to terminate the script.

The script can add setup for multiple ROS2 versions.
Simply follow the output.

After a setup is created, you should go through the files and check if they are correct.
Then try to push this to a GitHub-repository and open a PR.
You should then already see new workflows active.
