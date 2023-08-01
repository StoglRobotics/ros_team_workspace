============================================================
Welcome the documentation of *ROS Team Workspace*-Framework
============================================================

ROS Team Workspace (RosTeamWS) is a framework for boosting collaboration in teams when developing software for robots using `Robot Operating System (ROS) <https://www.ros.org/>`_.
It supports both **ROS** and **ROS 2**.
Its main goal is to optimize the workflow of development teams and focus more on programming robots.

The framework was initiated by Dr. Denis in 2016 to increase collaboration at the Institute for Anthropomatics and Robotics (IAR) - Intelligent Process Control and Robotics (IPR), Karlsruhe Institute of Technology (KIT).

From 2021, the framework is maintained by Stogl Robotics Consulting.

.. .. contents:: Table of Contents
..    :depth: 2


Purpose of the *RosTeamWS*
==========================

This package targets the following stakeholders:

* **Robotic Companies** to have a unified, overview repository of their use-cases, enable across-teams re-use of ROS packages, and to define "standardized" development structure.
* **Robotic Consultants** to simplify ROS packages' management and provide open and documented package-structure for their customers.
* **R&D Departments** to propagate the packages across their development stages by keeping simple access and sharing policy.
* **Research Labs** to organize their work across research groups and reduce onboarding time for new students.


The framework is the main entry-point for teams to:

#. organize their public and private ROS packages;
#. describes scenarios;
#. enable continuous integration across the use-cases;
#. provide scripts for easy use of ROS.


To achieve this, RosTeamWS defines:

#. an architecture of overlaid workspaces for sharing standard ROS packages;
#. standardized package structure for straightforward collaboration;
#. scripts for workspace and package management to keep their internal structure familiar to everyone in the team;
#. often-used scripts for tests of small development-chunks.


DISCLAIMER
==========
The work in the RosTeamWS-framework tries to follow, if applicable, `ROS Enhancement Proposals (REPs) <https://www.ros.org/reps/rep-0000.html>`_. Still, collisions in the best-practices proposals may occur.
The opinions and proposals stated here are merely related to the authors' experiences.


.. toctree::
   :hidden:

   tutorials/index.rst
   guidelines/index.rst
   use-cases/index.rst
   docker/index.rst
   faq/index.rst
