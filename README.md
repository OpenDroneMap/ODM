![](https://opendronemap.github.io/OpenDroneMap/img/odm_image.png)

What is it?
===========

OpenDroneMap is an open source toolkit for processing aerial drone imagery. Typical drones use simple point-and-shoot cameras, so the images from drones, while from a different perspective, are similar to any pictures taken from point-and-shoot cameras, i.e. non-metric imagery. OpenDroneMap turns those simple images into three dimensional geographic data that can be used in combination with other geographic datasets.

In a word, OpenDroneMap is a toolchain for processing raw civilian UAS imagery to other useful products. What kind of products?

1. Point Clouds
2. Digital Surface Models
3. Textured Digital Surface Models
4. Orthorectified Imagery
5. Classified Point Clouds
6. Digital Elevation Models
7. etc.

So far, it does Point Clouds, Digital Surface Models, Textured Digital Surface Models, and Orthorectified Imagery.

Users' mailing list: http://lists.osgeo.org/cgi-bin/mailman/listinfo/opendronemap-users

Developer's mailing list: http://lists.osgeo.org/cgi-bin/mailman/listinfo/opendronemap-dev

Overview video: https://www.youtube.com/watch?v=0UctfoeNB_Y

Developers
=================

Help improve our software!

[![Join the chat at https://gitter.im/OpenDroneMap/OpenDroneMap](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/OpenDroneMap/OpenDroneMap?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

1. Try to keep commits clean and simple
2. Submit a pull request with detailed changes and test results

Steps to get OpenDroneMap running:
==================================

(Requires Ubuntu 14.04 or later, see https://github.com/OpenDroneMap/odm_vagrant for running on Windows in a VM)

Support for Ubuntu 12.04 is currently BROKEN with the addition of OpenSfM and Ceres-Solver. We are working hard to get it working again in the future. 

#### Building OpenDroneMap using git

    cd path/to/odm/dir
    git clone https://github.com/OpenDroneMap/OpenDroneMap.git .
    export PYTHONPATH=$PYTHONPATH:`pwd`/SuperBuild/install/lib/python2.7/dist-packages:`pwd`/SuperBuild/src/opensfm
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/SuperBuild/install/lib
    bash configure.sh
    mkdir build && cd build && cmake .. && make && cd ..
    
#### Running OpenDroneMap

First you need a set of images, which may or may not be georeferenced. There are two ways OpenDroneMap can understand geographic coordinates. First, the images can be geotagged in their EXIF data. This is the default. Alternatively, you can create a GCP file, [a process detailed here](https://github.com/OpenDroneMap/OpenDroneMap/wiki/2.-Running-OpenDroneMap#running-odm-with-ground-control)

Create a project folder and places your images in an "images" directory:


    |-- /path/to/project/
        |-- images/
            |-- img-1234.jpg
            |-- ...


Then run:

    python run.py --project-path /path/to/project
    
There are many options for tuning your project. See the [wiki](https://github.com/OpenDroneMap/OpenDroneMap/wiki/3.-Run-Time-Parameters) or run `python run.py -h`

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

Here are some other videos, which may be outdated:

- https://www.youtube.com/watch?v=7ZTufQkODLs (2015-01-30)
- https://www.youtube.com/watch?v=m0i4GQdfl8A (2015-03-15)

Now that texturing is in the code base, you can access the full textured meshes using MeshLab. Open MeshLab, choose `File:Import Mesh` and choose your textured mesh from a location similar to the following: `reconstruction-with-image-size-1200-results\odm_texturing\odm_textured_model.obj`

__For Ubuntu 15.10 users, this will help you get running:__

    sudo apt-get install python-xmltodict
    sudo ln -s /usr/lib/x86_64-linux-gnu/libproj.so.9 /usr/lib/libproj.so

---

Alternatively, you can also run OpenDroneMap in a Docker container:

    export IMAGES=/absolute/path/to/your/images
    docker build -t opendronemap:latest .
    docker run -v $IMAGES:/images opendronemap:latest

To pass in custom parameters to the `run.py` script, simply pass it as arguments to the `docker run` command.

---

Example data can be found at https://github.com/OpenDroneMap/odm_data

---

Long term, the aim is for the toolchain to also be able to optionally push to a variety of online data repositories, pushing hi-resolution aerials to [OpenAerialMap](http://opentopography.org/), point clouds to [OpenTopography](http://opentopography.org/), and pushing digital elevation models to an emerging global repository (yet to be named...). That leaves only digital surface model meshes and UV textured meshes with no global repository home.

---


Documentation:
==============

For documentation, please take a look at our [wiki](https://github.com/OpenDroneMap/OpenDroneMap/wiki).

