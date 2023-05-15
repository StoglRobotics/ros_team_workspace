======================================
Nvidia support for docker
======================================
.. _docker-nvidia-support-how-to:

If you want to expose your nvidia driver into the docker container this is possible by using the `NVIDIA Container Toolkit <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/overview.html>`_.

Prerequisites
""""""""""""""
Before we can install the NVIDIA Container Toolkit we have to install the nvidia drivers for our platform and install docker.

**NOTE**: Check the end of the file for some tips about throubleshoting.

Nvidia drivers
---------------
Make sure you have the NVIDIA drivers for your Linux distribution installed. This can either be done by using your package manager or you can download the ``.run`` installers from `NVIDIA Driver Downloads <https://www.nvidia.com/Download/index.aspx?lang=en-us>`_.
This depends on the operating system and package manager you are using so you have to look it up.
You can then check your drivers by typing

  .. code-block:: bash

     nvidia-smi # NVIDIA System Management Interface program


Which should print something like this:

.. code-block:: text

  +-----------------------------------------------------------------------------+
  | NVIDIA-SMI 515.48.07    Driver Version: 515.48.07    CUDA Version: 11.7     |
  |-------------------------------+----------------------+----------------------+
  | GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
  | Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
  |                               |                      |               MIG M. |
  |===============================+======================+======================|
  |   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0  On |                  N/A |
  | N/A   37C    P8    18W /  N/A |    550MiB /  8192MiB |      0%      Default |
  |                               |                      |                  N/A |
  +-------------------------------+----------------------+----------------------+

  +-----------------------------------------------------------------------------+
  | Processes:                                                                  |
  |  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
  |        ID   ID                                                   Usage      |
  |=============================================================================|
  |    0   N/A  N/A      1435      G   ...xorg-server-1.20.14/bin/X      288MiB |
  |    0   N/A  N/A      2516      G   ....0.2/bin/.firefox-wrapped      144MiB |
  +-----------------------------------------------------------------------------+

Docker installation
--------------------
.. _docker-nvidia-support-prerequisites_docker:

Make sure docker is installed and it's working correctly. For instructions on how to install docker have a look :ref:`in general info on docker installation <general-info-on-docker-installation>`.

How to setup a container with nvidia support
""""""""""""""""""""""""""""""""""""""""""""""
Install nvidia-docker
----------------------
**NOTE**: If you already have installed nvidia-docker, you can skip this and go right to the next section.

1. You then have to install the NVIDIA Container Toolkit for docker as described `in official documentation <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker>`_.
   For Ubuntu this can be done as follows:

   .. code-block:: bash

      distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
         && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
         && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

   You then have to update your package list to include the new added nvidia-docker2.

   .. code-block:: bash

      sudo apt-get update

   And install the the NVIDIA Container Toolkit:

   .. code-block:: bash

      sudo apt-get install -y nvidia-docker2

   After the installation finished, you need to restart the Docker daemon:

   .. code-block:: bash

      sudo service docker restart

   At this point you can verify that everything works as intended by running:

   .. code-block:: bash

      docker run --rm --gpus all nvidia/cuda:11.7.1-base-ubuntu22.04 nvidia-smi

  **NOTE**: if you get an error executing above docker command make sure that you have ``Nvidia Driver version 515`` or above installed!

  Which should print something like:

  .. code-block:: text

    +-----------------------------------------------------------------------------+
    | NVIDIA-SMI 515.48.07    Driver Version: 515.48.07    CUDA Version: 11.7     |
    |-------------------------------+----------------------+----------------------+
    | GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
    | Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
    |                               |                      |               MIG M. |
    |===============================+======================+======================|
    |   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0  On |                  N/A |
    | N/A   37C    P8    18W /  N/A |    478MiB /  8192MiB |      3%      Default |
    |                               |                      |                  N/A |
    +-------------------------------+----------------------+----------------------+

    +-----------------------------------------------------------------------------+
    | Processes:                                                                  |
    |  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
    |        ID   ID                                                   Usage      |
    |=============================================================================|
    +-----------------------------------------------------------------------------+

Change the Dockerfile
----------------------------------
(1.) If you haven't done so already:
   Create a new docker workspace with the :ref:`setup-ros-workspace-docker<uc-setup-docker-workspace>` command.

   **NOTE**: If you set up a nvidia-docker container, you are finished at this point.

2. Replace the the ``FROM ubuntu:<version>`` directive in your Dockerfile with the nvidia container of your needs. The following table gives you a quick overview:

   .. list-table:: Examples for nvidia containers
      :widths: auto
      :header-rows: 1
      :stub-columns: 1

      * - Ubuntu version directive
        - Nvidia docker replacement
      * - Ubuntu:20.04
        - nvidia/cuda:11.7.1-base-ubuntu20.04
      * - Ubuntu:22.04
        - nvidia/cuda:11.7.1-base-ubuntu22.04

   A list of all available containers can be found `in the official documentation <https://hub.docker.com/r/nvidia/cuda>`_.

3. Remove the existing docker container and image.

   .. code-block:: bash

      docker container rm <container-name>

   .. code-block:: bash

      docker image rm <image-name>

4. Recreate your container.
   Go inside the ``.rtw_docker_defines`` directory in your workspace folder and the execute:

   .. code-block:: bash

      ./recreate_docker.sh

You now should have a docker container which exposes your nvidia drivers and can switch to your workspace with ``rtw_switch_to_docker``.

Troubleshoting
""""""""""""""

Be sure that graphics card driver is installed properly
--------------------------------------------------------
Check if drivers and libraries are installed properly by executing ``nvidia-smi`` after a fresh system start (Yes, you have to restart your computer!).
There you should see expected version of Nvidia driver.
If you get any error follow the next steps to be sure that the expected version of driver is installed properly.

1. Install required version of the driver using GUI for additional drivers in system's settings or ``ubuntu-drivers`` command.

   .. note::
      Be careful when executing command if multiple devices are using custom drivers you can unintenntionaly install wrong driver for another device (you will know if this is relevant for you - but it is imprtant to note it.)

2. Delete all other nvidia driver's version and corresponding libraries - use ``purge`` command for it.
3. Restart your computer.
4. If you have issues with the graphics after restart do the following:

   1. Close your eyes and breathe slowely in and out at least once :)
   2. You probably didn't install everything properly so the open-source ``noveau`` driver is used which is not adequate for this scenario.
   3. Use ``<CTRL> + <ALT> + <F2-3-4...>`` keys to switch to a linux terminal.
   4. Login there and execute ``ubuntu-drivers`` command to install missing drivers.
   5. Now restart again and everything should work properly.

5. Now check again output of the ``ṅvidia-smi`` command


Graphics card should be used in the ``Performance`` mode
---------------------------------------------------------
If you have only ``X`` in the output from ``nvidia-smi`` than make sure that the graphics card uses ``Performance Mode`` in ``PRIME profile`` section of "NVIDIA X Server Settings" application.


``nvidia-smi`` command inside docker container
-----------------------------------------------
Take a note that ``nvidia-smi`` command in the docker container is necessary test to see if docker has access to the graphic card, but it doesn't shows any applications that are using it.
You can see on your host if a docker application is using graphic card and how much.


References
"""""""""""

- [Docker configuration for rviz using nvidia graphics card (by @ruffsl)](https://gist.github.com/ruffsl/e7ca631a618ece3eb0be8e4bf168accb)
