================================
FAQ
================================

On Docker
----------------

How to forward an xsession
"""""""""""""""""""""""""""""
If you want to forward an xsession from docker (e.g. rviz2), you have to install xhost.
The forwarding is done automatically by adding docker user to the X Server access list when docker is created.


How to use nvidia driver in docker
""""""""""""""""""""""""""""""""""""
We explain :ref:`here <docker-nvidia-support-how-to>` how to expose your nvidia drivers into the docker container.


sudo: unable to resolve host <hostname>: Name or service not known
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
Problem using sudo: If you encounter following message trying to use sudo: ``sudo: unable to resolve host <hostname>: Name or service not known``. You have to add ``127.0.0.1 <hostname>`` to the ``/etc/hosts`` file inside the container. However, using sudo should work fine without adding it.

Docker and ROS (ROS1): Can not start my roscore.
"""""""""""""""""""""""""""""""""""""""""""""""""
If you try to start your roscore inside the docker container with ``roscore``-command and get the following error message: ``RLException: Unable to contact my own server at [http://<hostname>:<port>]``. You have to add ``127.0.0.1 <hostname>`` to the ``/etc/hosts`` file inside the container.
