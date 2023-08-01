# ros_team_workspace

Ros Team Workspace (RosTeamWS) is a framework for boosting collaboration in teams when developing software for robots using [Robot Operating System (ROS)](https://www.ros.org/).
It supports both **ROS** and **ROS 2**.
Its main goal is to optimize the workflow of development teams and focus more on programming robots.

[![Licence](https://img.shields.io/badge/License-Apache%20License%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

[Documentation](https://stoglrobotics.github.io/ros_team_workspace)

### Build status

|         | Foxy    | Rolling
|:-------:|:-------:|:-------:|
| Branch  | [`foxy`](https://github.com/StoglRobotics/ros_team_workspace/tree/rolling) | [`foxy`](https://github.com/StoglRobotics/ros_team_workspace/tree/rolling)
| Build  | Linux: [![Build Status](https://github.com/StoglRobotics/ros_team_workspace/workflows/Build&20Foxy%20ros_team_workspace/badge.svg)](https://github.com/StoglRobotics/ros_team_workspace/actions?query=workflow%3A"Build%20Foxy") | Linux: [![Build Status](https://github.com/StoglRobotics/ros_team_workspace/workflows/Build%20ros_team_workspace/badge.svg)](https://github.com/StoglRobotics/ros_team_workspace/actions?query=workflow%3A"Build") |
| Lint  | [![Linters Status](https://github.com/StoglRobotics/ros_team_workspace/workflows/Lint%20ros_team_workspace/badge.svg)](https://github.com/StoglRobotics/ros_team_workspace/actions?query=workflow%3A"Lint")| [![Linters Status](https://github.com/StoglRobotics/ros_team_workspace/workflows/Lint%20ros_team_workspace/badge.svg)](https://github.com/StoglRobotics/ros_team_workspace/actions?query=workflow%3A"Lint") |


## Purpose

This package targets the following stakeholders:

* **Robotic Companies** to have a unified, overview repository of their use-cases, enable across-teams re-use of ROS packages, and to define "standardized" development structure.
* **Robotic Consultants** to simplify ROS packages' management and provide open and documented package-structure for their customers.
* **R&D Departments** to propagate the packages across their development stages by keeping simple access and sharing policy.
* **Research Labs** to organize their work across research groups and reduce onboarding time for new students.


**The framework is the main entry-point for teams to**:

1. organize their public and private ROS packages;
2. describes scenarios;
3. enable continuous integration across the use-cases;
4. and provide scripts for easy use of ROS.


**To achieve this, RosTeamWS defines**:

1. an architecture of overlaid workspaces for sharing standard ROS packages;
2. standardized package structure for straightforward collaboration;
3. scripts for workspace and package management to keep their internal structure familiar to everyone in the team;
4. often-used scripts for tests of small development-chunks.


The framework was initiated by Dr. Denis (2017-2020) for increasing collaboration at the Institute for Anthropomatics and Robotics (IAR) - Intelligent Process Control and Robotics (IPR) of Karlsruhe Institute of Technology (KIT).

From 2021, the framework is maintained by Stogl Robotics Consulting.

DISCLAIMER
-------------
The work in the RosTeamWS-framework tries to follow, if applicable, [ROS Enhancement Proposals (REPs)](https://github.com/ros-infrastructure/rep). Still, collisions in the best-practices proposals may occur.
The opinions and proposals stated here are merely related to the authors' experiences.
