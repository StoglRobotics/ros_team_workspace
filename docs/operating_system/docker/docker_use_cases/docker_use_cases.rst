==============================
Use Cases with docker support
==============================
.. _uc-with-docker-support-index:

All use cases with docker support are listed below. For a detailed description of the use case as well as the respective commands, you can click on the link in the use case column.

.. list-table:: Overview of currently supported use cases
   :widths: auto
   :header-rows: 1
   :stub-columns: 1

   * - use case
     - docker command
     - description
   * - :ref:`Setup a new Workspace<rtwcli-setup-workspace>` with the ``rtw`` CLI
     - ``rtw workspace create --docker``
     - Creates a new workspace and maps it inside a Docker container. Use ``rtw docker enter`` to connect to it afterwards.
