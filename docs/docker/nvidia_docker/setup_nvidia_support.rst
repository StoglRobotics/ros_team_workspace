======================================
Nvidia support for docker
======================================
.. _docker-nvidia-support-how-to:

If you want to expose your nvidia driver into the docker container this is possible by using the `NVIDIA Container Toolkit <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/overview.html>`_. 

Prerequisites
""""""""""""""
Before we can install the NVIDIA Container Toolkit we have to install the nvidia drivers for our platform and install docker.

Nvidia drivers
----------------
Make sure you have the NVIDIA drivers for your Linux distribution installed. This can either be done by using your package manager or you can download the ``.run`` installers from NVIDIA Driver Downloads `here <https://www.nvidia.com/Download/index.aspx?lang=en-us>`_. This depends on the operating system and package manager you are using so you have to look it up.
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

Docker
----------------
.. _docker-nvidia-support-prerequisites_docker:

Make sure docker is installed and it's working correctly. For instructions on how to install docker have a look :ref:`here<general-info-on-docker-installation>`.

Installation
""""""""""""""""
1. You then have to install the NVIDIA Container Toolkit for docker as described `here <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker>`_. 
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

   sudo systemctl restart docker

  At this point you can verify that everything works as intended by running:
  
  .. code-block:: bash

   sudo docker run --rm --gpus all nvidia/cuda:11.0.3-base-ubuntu20.04 nvidia-smi

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

2. Replace the