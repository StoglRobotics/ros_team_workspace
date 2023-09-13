=======================
Development Guidelines
=======================
.. _guidelines-development:

This documents proposes development guidelines to increase work efficiency and synergy within the rest of the team.

**NOTE**: All the proposal here are the results of authors' personal experiences. Saying that, if you have any idea to make them better you are very welcome to create a PR.

Overview
=========

1. `Making changes to the code-base by submitting a Pull Request <#code-changes-and-pull-request-submissions>`_

2. `Using *pre-commit* <#use-pre-commit-for-formatting-and-linting>`_


Code Changes and Pull Request Submissions
==========================================

To make a change of the existing code base, Pull Request (GitHub) or Merge Request (GitLab) are used.
This section describes in short the process with valuable tips to make your and reviewers' life easier.

#. Check with the team if it is usual to submit a PR/MR from a fork or directly into the target repository.
   This usually depends on the team size and organization.
   When working with public repositories you always need to create a fork.

   .. note::

      In *Stogl Robotics* we have the organization called `StoglRobotics-forks <https://github.com/StoglRobotics-forks>`_ where all forks of public repositories live and are accessible for writing by all team members.
      This simplifies collaboration inside the team - there is no need for individual access grants when using forks under your user.
      **Always** check if there is already a fork in *StoglRobotics-forks* organization and if not create it.

#. Start development always from the up-to date state of the repositories default branch (usually called *master* or *main* - for simplicity we call it *master* here).
   Take into account that the master branch of your fork is usually not up to date with the upstream repository.
   Therefore be careful about that and use the opportunity to sync the *master* branch of the fork to the state of the *master* branch of the upstream repository.

#. **Always** create a new branch for each feature or bug fix.
   **Never** make multiple changes on a same feature branch. One feature / change == one branch. Don't submit PRs from *master* branch.

#. **Before** starting development check how the branch will be merged, using *merge commit* or *squash* method.
   If *merge commit* is used make sure that each of your commits is clean and named properly since they will become part of the repository's history.

   .. note::

      In *Stogl Robotics* we are always *squashing* commits, i.e., one future or one bug fix is one commit in the default branch. The commit message is edited before merging. Make sure the commit message retains relevant information of the commits to be squashed (e.g. non-trivial reason why a change was made)

#. Explain in the PR/MR description what your code is doing and why.

#. When you are finished with development and want to submit code for the review - consider the following tips:

   - **Always** run *pre-commit* formatters;
   - Review your code **first by yourself** before asking someone else;
   - Make sure there **are no** commented code blocks or if they have to be there, add explanations why;
   - Resolve all TODOs or add concrete questions about them either in the code or in review comments so that other people know this is open for discussion;
   - By iterating on the review adjust **all parts** of the code with the same or similar patterns even if you get a comment about those only in one place - reviewers usually don't like to repeat themselves on each iteration of the same issue - if you are not sure about something, ask;
   - Ask yourself: *Would I like to review this code ?*.

   .. important::

      The reviews are done by other people and show respect toward their time. Reviewer's task is **not to clean** the code behind you.
      Many people get very angry if you provide messy code, usually so much that you have to wait for weeks to get your code reviewed again.


Use *pre-commit* for formatting and linting
============================================

``pre-commit`` is a program that adds hooks into ``git`` so when you commit something, actions can be automatically executed.
There are many different possibility with ``pre-commit`` but it is mostly used for integrating code linters and formatters to always commit clean code.

Getting started with *pre-commit*
----------------------------------

First install it to you computer using:

   .. code-block:: bash

      pip install pre-commit


and than tell ``git`` that you are using it by executing the following in each repository:

   .. code-block:: bash

      pre-commit install

This will execute ``pre-commit`` on every commit you make and prevent it from doing it if some checks fail.
If you really need to ignore check in the commit, you can add ``-n`` flag to the ``git commit`` command and they will be skipped.


.. note:: If your repository does not use ``pre-commit`` yet, it is very easy to add it by creating a configuration file ``.pre-commit-config.yaml`` in the top level of your repository (there where ``.git`` folder is).
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
