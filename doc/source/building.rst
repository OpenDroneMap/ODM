.. Notes and doc on building ODM

Building
========


Hardware Recommendations
------------------------

OpenDroneMap is built on Ubuntu 16.04 but can be run on other major platforms using Docker.

Minimum 4GB of RAM, recommended 16GB or more. Many parts of the ODM toolchain are parallelized, and memory requirements will increase as the size of the input data increases.

.. _docker-installation:

Docker Installation (cross-platform)
------------------------------------

First you need to `download and install Docker <https://www.docker.com/>`_. Note for Windows users that Docker CE only works for Windows 10 Professional and Enterprise. Otherwise you should use `Docker Toolbox <https://www.docker.com/products/docker-toolbox>`_

You can easily pull and run a pre-built image. Start here: :ref:`docker-usage`. If you want to build your own docker image, follow the instructions below.

Before running ODM it's advised to check that Docker is allocating sufficient resources to containers. In Windows this can be done in the 'Docker for Windows' program under the 'Advanced' tab.

Build the image
```````````````

Download and extract the latest version of ODM: :ref:`download`

In Docker toolbox or Docker CE, navigate to your extracted ODM directory. Then build the Docker image.::

    cd Documents/OpenDroneMap_v0_3_1/
    docker build -t my_odm_image .

When building your own Docker image, if image size is of importance to you, you should use the ``--squash`` flag, like so:::

    docker build --squash -t my_odm_image .

This will clean up intermediate steps in the Docker build process, resulting in a significantly smaller image (about half the size).

Experimental flags need to be enabled in Docker to use the ``--squash`` flag. To enable this, insert the following into the file /etc/docker/daemon.json:::

    {
        "experimental": true
    }

After this, you must restart docker by typing ``sudo service docker restart`` into your Linux terminal.



Once this is done, go to :ref:`docker-usage`


.. _native-installation:

Native Installation (Ubuntu 16.04)
----------------------------------

Download and extract the latest version of ODM: :ref:`download`

The installation is simple::

    bash configure.sh install


configure.sh can take up to 2 arguments

configure.sh command [n]
    command: can be one of (install, reinstall, uninstall, usage)

    [n] is an optional argument that will set the number of processes for the compiler


Setting environment variables
`````````````````````````````

Using your favorite editor, open `~/.bashrc` and append the following to the bottom of the file (replace /your/path/OpenDroneMap with your installation path, e.g. /home/user/OpenDroneMap)::

    export PYTHONPATH=$PYTHONPATH:/your/path/OpenDroneMap/SuperBuild/install/lib/python2.7/dist-packages
    export PYTHONPATH=$PYTHONPATH:/your/path/OpenDroneMap/SuperBuild/src/opensfm
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/your/path/OpenDroneMap/SuperBuild/install/lib

You will need to log out and back in again for the variables to set.

