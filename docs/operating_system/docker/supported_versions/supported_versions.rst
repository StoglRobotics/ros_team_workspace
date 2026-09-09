=============================================
Supported ros versions and operating systems
=============================================
.. _supported-os-ros-docker-versions-index:

For the supported Ubuntu and ros version combinations have a look at the table below.

.. list-table:: Supported Ubuntu and ros versions combinations are marked with an X.
   :widths: auto
   :header-rows: 1
   :stub-columns: 1

   * - ros version
     - ubuntu 20.04
     - ubuntu 22.04
     - ubuntu 24.04
   * - foxy
     - X
     -
     -
   * - galactic
     - X
     -
     -
   * - humble
     -
     - X
     -
   * - iron
     -
     - X
     -
   * - rolling
     - X*
     - X*
     - X (rtwcli only)
   * - jazzy
     -
     -
     - X (rtwcli only)


\* - last release for that Ubuntu distro - no ``rosdep`` support anymore.

Debian
""""""

Debian has no native ROS binary distribution — the Ubuntu/ROS combinations in the table above don't apply to it directly.
It is used as a **real-time docker host**: a Debian PC provides the PREEMPT_RT kernel and Docker runtime, while ROS itself still runs inside one of the Ubuntu containers from the table above.
See :ref:`Setup Real-Time Kernel (Debian, prebuilt package) <uc-setup-rt-kernel-debian>` for the full setup.
