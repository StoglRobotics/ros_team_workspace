================================
FAQ
================================

Docker
----------------
sudo: unable to resolve host <hostname>: Name or service not known
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
Problem using sudo: If you encounter following message trying to use sudo: ``sudo: unable to resolve host <hostname>: Name or service not known``. You have to add ``127.0.0.1 <hostname>`` to the ``/etc/hosts`` file inside the container. However, using sudo should work fine without adding it.
