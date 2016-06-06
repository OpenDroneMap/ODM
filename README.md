Instructions:
==================================
Assumes clean installation of Ubuntu 14.04

    sudo add-apt-repository ppa:xorg-edgers/ppa
    sudo apt-get update
    sudo apt-get install nvidia-352
    sudo apt-get install build-essential git cmake cmake-gui
    git clone https://github.com/OpenDroneMap/OpenDroneMap.git
    sudo apt-get install ppa-purge #fix from http://askubuntu.com/questions/575548/system-settings-stopped-showing-up
    sudo add-apt-repository ppa:xorg-edgers/ppa && sudo apt-get update
    sudo ppa-purge  ppa:xorg-edgers/ppa && sudo apt-get update
    cd OpenDroneMap
    chmod 777 configure.sh
    ./configure.sh

    #copypaste the following lines to the bottom of your .bashrc file:
    export PYTHONPATH=$PYTHONPATH:`pwd`/SuperBuild/install/lib/python2.7/dist-packages
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/SuperBuild/install/lib

    #ROS pre-installation steps to get access to ROS Indigo repositories
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116

    #Install Ecto library
    sudo apt-get install ros-indigo-ecto-*

    #Final steps:
    python run.py python run.py --project-path ~/OpenDroneMap/

    #It will complain about libraries you don't have. Install them one by one using aptitude, not apt-get. Pay very close attention to what aptitude says it's about to do so that you don't uninstall somethign important.


When the process finishes, the results will be organized as follows

    |-- images/
        |-- img-1234.jpg
        |-- ...
    |-- images_resize/
        |-- img-1234.jpg
        |-- ...
    |-- opensfm/
        |-- not much useful in here
    |-- pmvs/
        |-- recon0/
            |-- models/
                |-- option-0000.ply         # Dense point cloud
    |-- odm_meshing/
        |-- odm_mesh.ply                    # A 3D mesh
        |-- odm_meshing_log.txt             # Output of the meshing task. May point out errors.
    |-- odm_texturing/
        |-- odm_textured_model.obj          # Textured mesh
        |-- odm_textured_model_geo.obj      # Georeferenced textured mesh
        |-- texture_N.jpg                   # Associated textured images used by the model
    |-- odm_georeferencing/
        |-- odm_georeferenced_model.ply     # A georeferenced dense point cloud
        |-- odm_georeferenced_model.ply.laz # LAZ format point cloud
        |-- odm_georeferenced_model.csv     # XYZ format point cloud
        |-- odm_georeferencing_log.txt      # Georeferencing log
        |-- odm_georeferencing_utm_log.txt  # Log for the extract_utm portion
    |-- odm_georeferencing/
        |-- odm_orthophoto.png              # Orthophoto image (no coordinates)
        |-- odm_orthophoto.tif              # Orthophoto GeoTiff
        |-- odm_orthophoto_log.txt          # Log file
        |-- gdal_translate_log.txt          # Log for georeferencing the png file

    You can access the full textured meshes using MeshLab. Open MeshLab, choose `File:Import Mesh` and choose your textured mesh from a location similar to the following: `reconstruction-with-image-size-1200-results\odm_texturing\odm_textured_model.obj`

Extra Documentation:
==============

For documentation, please take a look at the [wiki](https://github.com/OpenDroneMap/OpenDroneMap/wiki).

