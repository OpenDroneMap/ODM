# OpenDroneMap

![](https://raw.githubusercontent.com/OpenDroneMap/OpenDroneMap/master/img/odm_image.png)

## What is it?

OpenDroneMap is an open source toolkit for processing aerial drone imagery. Typical drones use simple point-and-shoot cameras, so the images from drones, while from a different perspective, are similar to any pictures taken from point-and-shoot cameras, i.e. non-metric imagery. OpenDroneMap turns those simple images into three dimensional geographic data that can be used in combination with other geographic datasets.

![](https://raw.githubusercontent.com/OpenDroneMap/OpenDroneMap/master/img/tol_ptcloud.png)

In a word, OpenDroneMap is a toolchain for processing raw civilian UAS imagery to other useful products. What kind of products?

1. Point Clouds
2. Digital Surface Models
3. Textured Digital Surface Models
4. Orthorectified Imagery
5. Classified Point Clouds
6. Digital Elevation Models
7. etc.

So far, it does Point Clouds, Digital Surface Models, Textured Digital Surface Models, and Orthorectified Imagery. Open Drone Map now includes state-of-the-art 3D reconstruction work by Michael Waechter, Nils Moehrle, and Michael Goesele. See their publication at http://www.gcc.tu-darmstadt.de/media/gcc/papers/Waechter-2014-LTB.pdf.


## QUICKSTART

OpenDroneMap can run natively on Ubuntu 14.04 or later, see [Build and Run Using Docker](#build-and-run-using-docker) for running on Windows / MacOS. A Vagrant VM is also available: https://github.com/OpenDroneMap/odm_vagrant.

*Support for Ubuntu 12.04 is currently BROKEN with the addition of OpenSfM and Ceres-Solver. It is likely to remain broken unless a champion is found to fix it.*

**[Download the latest release here](https://github.com/OpenDroneMap/OpenDroneMap/releases)**

Current version: 0.2 (this software is in beta)

1. Extract and enter the OpenDroneMap directory
2. Run `bash configure.sh`
3. Download and extract a sample dataset [here](https://github.com/OpenDroneMap/odm_data_aukerman/archive/master.zip)
4. Run `./run.sh --project-path <PATH>`, replacing `<PATH>` with the dataset path. 
5. Enter dataset directory to view results: 
  - orthophoto: odm_orthophoto/odm_orthophoto.tif
  - textured mesh model: odm_texturing/odm_textured_model_geo.obj
  - point cloud (georeferenced): odm_georeferencing/odm_georeferenced_model.ply
  
See [here](https://github.com/OpenDroneMap/OpenDroneMap/tree/ebaaf802a1fb50e335b3807a35d00cba1e106d11#installation) for more detailed installation instructions. 

### Installation

Extract and enter the downloaded OpenDroneMap directory and compile all of the code by executing a single configuration script:
  
    bash configure.sh

For Ubuntu 15.10 users, this will help you get running:

    sudo apt-get install python-xmltodict
    sudo ln -s /usr/lib/x86_64-linux-gnu/libproj.so.9 /usr/lib/libproj.so
    
### Environment Variables

There are some environmental variables that need to be set. Open the ~/.bashrc file on your machine and add the following 3 lines at the end. The file can be opened with ```gedit ~/.bashrc``` if you are using an Ubuntu desktop environment. Be sure to replace the "/your/path/" with the correct path to the location where you extracted OpenDroneMap:

    export PYTHONPATH=$PYTHONPATH:/your/path/OpenDroneMap/SuperBuild/install/lib/python2.7/dist-packages
    export PYTHONPATH=$PYTHONPATH:/your/path/OpenDroneMap/SuperBuild/src/opensfm
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/your/path/OpenDroneMap/SuperBuild/install/lib
    
Note that using `run.sh` sets these temporarily in the shell. 
    
### Run OpenDroneMap

First you need a set of images, taken from a drone or otherwise. Example data can be cloned from https://github.com/OpenDroneMap/odm_data

Then run:

    python run.py --project-path /path/to/project -i /path/to/images

The images will be copied over to the project path so you only need to specify the `-i /path/` once. There are many options for tuning your project. See the [wiki](https://github.com/OpenDroneMap/OpenDroneMap/wiki/Run-Time-Parameters) or run `python run.py -h`

### View Results

When the process finishes, the results will be organized as follows:

    |-- images/
        |-- img-1234.jpg
        |-- ...
    |-- images_resize/
        |-- img-1234.jpg
        |-- ...
    |-- opensfm/
        |-- see mapillary/opensfm repository for more info
    |-- pmvs/
        |-- recon0/
            |-- models/
                |-- option-0000.ply         # Dense point cloud (not georeferenced)
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

Any file ending in .obj or .ply can be opened and viewed in [MeshLab](http://meshlab.sourceforge.net/) or similar software. That includes `pmvs/recon0/models/option-000.ply`, `odm_meshing/odm_mesh.ply`, `odm_texturing/odm_textured_model[_geo].obj`, or `odm_georeferencing/odm_georeferenced_model.ply`. Below is an example textured mesh:

![](https://raw.githubusercontent.com/alexhagiopol/OpenDroneMap/feature-better-docker/toledo_dataset_example_mesh.jpg)

You can also view the orthophoto GeoTIFF in [QGIS](http://www.qgis.org/) or other mapping software:

![](https://raw.githubusercontent.com/OpenDroneMap/OpenDroneMap/master/img/bellus_map.png)

## Build and Run Using Docker

(Instructions below apply to Ubuntu 14.04, but the Docker image workflow 
has equivalent procedures for Mac OS X and Windows. See [docs.docker.com](docs.docker.com))

OpenDroneMap is Dockerized, meaning you can use containerization to build and run it without tampering with the configuration of libraries and packages already
installed on your machine. Docker software is free to install and use in this context. If you don't have it installed,
see the [Docker Ubuntu installation tutorial](https://docs.docker.com/engine/installation/linux/ubuntulinux/) and follow the
instructions through "Create a Docker group". Once Docker is installed, the fastest way to use OpenDroneMap is to run a pre-built image by typing:

    docker run -it --rm -v $(pwd)/images:/code/images -v $(pwd)/odm_orthophoto:/code/odm_orthophoto -v $(pwd)/odm_texturing:/code/odm_texturing opendronemap/opendronemap

If you want to build your own Docker image from sources, type:

    docker build -t packages -f packages.Dockerfile .

    docker build -t my_odm_image .
    docker run -it --rm -v $(pwd)/images:/code/images -v $(pwd)/odm_orthophoto:/code/odm_orthophoto -v $(pwd)/odm_texturing:/code/odm_texturing my_odm_image

Using this method, the containerized ODM will process the images in the OpenDroneMap/images directory and output results
to the OpenDroneMap/odm_orthophoto and OpenDroneMap/odm_texturing directories as described in the **Viewing Results** section. 
If you want to view other results outside the Docker image simply add which directories you're interested in to the run command in the same pattern
established above. For example, if you're interested in the dense cloud results generated by PMVS and in the orthophoto,
simply use the following `docker run` command after building the image:

    docker run -it --rm -v $(pwd)/images:/code/images -v $(pwd)/pmvs:/code/pmvs -v $(pwd)/odm_orthophoto:/code/odm_orthophoto my_odm_image

If you want to get all intermediate outputs, run the following command:

    docker run -it --rm -v $(pwd)/images:/code/images -v $(pwd)/odm_georeferencing:/code/odm_georeferencing -v $(pwd)/odm_meshing:/code/odm_meshing -v $(pwd)/odm_orthophoto:/code/odm_orthophoto -v $(pwd)/odm_texturing:/code/odm_texturing -v $(pwd)/opensfm:/code/opensfm -v $(pwd)/pmvs:/code/pmvs opendronemap/opendronemap

To pass in custom parameters to the run.py script, simply pass it as arguments to the `docker run` command. For example:

    docker run -it --rm -v $(pwd)/images:/code/images v $(pwd)/odm_orthophoto:/code/odm_orthophoto -v $(pwd)/odm_texturing:/code/odm_texturing opendronemap/opendronemap --resize-to 1800 --force-ccd 6.16

## User Interface

A web interface and API to OpenDroneMap is currently under active development in the [WebODM](https://github.com/OpenDroneMap/WebODM) repository.

## Video Support

Currently we have an experimental feature that uses ORB_SLAM to render a textured mesh from video. It is only supported on Ubuntu 14.04 on machines with X11 support. See the [wiki](https://github.com/OpenDroneMap/OpenDroneMap/wiki/Reconstruction-from-Video)for details on installation and use.

## Examples

Coming soon...

## Documentation:

For documentation, please take a look at our [wiki](https://github.com/OpenDroneMap/OpenDroneMap/wiki).Check here first if you are having problems. If you still need help, look through the issue queue or create one. There's also a general help chat [here](https://gitter.im/OpenDroneMap/generalhelp). 

## Developers

Help improve our software!

[![Join the chat at https://gitter.im/OpenDroneMap/OpenDroneMap](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/OpenDroneMap/OpenDroneMap?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

1. Try to keep commits clean and simple
2. Submit a pull request with detailed changes and test results


