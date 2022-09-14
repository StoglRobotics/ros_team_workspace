================================
FAQ
================================

On Docker
----------------

How to forward a xsession
"""""""""""""""""""""""""""""
If you want to forward a xsession from docker (e.g. rviz2), you have to install xhost. When executing the commands ``start_container`` the docker user is added to the X Server access list.


How to use nvidia driver in docker
""""""""""""""""""""""""""""""""""""
We explain :ref:`here <docker-nvidia-support-how-to>` how to expose your nvidia drivers into the docker container.


sudo: unable to resolve host <hostname>: Name or service not known
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
Problem using sudo: If you encounter following message trying to use sudo: ``sudo: unable to resolve host <hostname>: Name or service not known``. You have to add ``127.0.0.1 <hostname>`` to the ``/etc/hosts`` file inside the container. However, using sudo should work fine without adding it.
