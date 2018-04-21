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
5. Classified Point Clouds (coming soon)
6. Digital Elevation Models
7. etc.

Open Drone Map now includes state-of-the-art 3D reconstruction work by Michael Waechter, Nils Moehrle, and Michael Goesele. See their publication at [http://www.gcc.tu-darmstadt.de/media/gcc/papers/Waechter-2014-LTB.pdf](http://www.gcc.tu-darmstadt.de/media/gcc/papers/Waechter-2014-LTB.pdf).

## QUICKSTART

### Docker (All platforms)

The easiest way to run ODM is through Docker. If you don't have it installed,
see the [Docker Ubuntu installation tutorial](https://docs.docker.com/engine/installation/linux/ubuntulinux/) and follow the
instructions through "Create a Docker group". The Docker image workflow
has equivalent procedures for Mac OS X and Windows found at [docs.docker.com](docs.docker.com). Then run the following command which will build a pre-built image and run on images found in `$(pwd)/images` (you can change this if you need to, see the [wiki](https://github.com/OpenDroneMap/OpenDroneMap/wiki/Docker) for more detailed instructions.

```
docker run -it --rm -v $(pwd)/images:/code/images -v $(pwd)/odm_orthophoto:/code/odm_orthophoto -v $(pwd)/odm_texturing:/code/odm_texturing opendronemap/opendronemap
```

### Native Install (Ubuntu 16.04)

** Please note that we need help getting ODM updated to work for 16.10+. Look at [#659](https://github.com/OpenDroneMap/OpenDroneMap/issues/659) or drop into the [gitter](https://gitter.im/OpenDroneMap/OpenDroneMap) for more info.


**[Download the latest release here](https://github.com/OpenDroneMap/OpenDroneMap/releases)**  
Current version: 0.3.1 (this software is in beta)

1. Extract and enter the OpenDroneMap directory
2. Run `bash configure.sh install`
4. Edit the `settings.yaml` file in your favorite text editor. Set the `project-path` value to an empty directory (you will place sub-directories containing individual projects inside). You can add many options to this file, [see here](https://github.com/OpenDroneMap/OpenDroneMap/wiki/Run-Time-Parameters)
3. Download a sample dataset from [here](https://github.com/OpenDroneMap/odm_data_aukerman/archive/master.zip) (about 550MB) and extract it as a subdirectory in your project directory.
4. Run `./run.sh odm_data_aukerman`
5. Enter dataset directory to view results:
    - orthophoto: odm_orthophoto/odm_orthophoto.tif
    - textured mesh model: odm_texturing/odm_textured_model_geo.obj
    - point cloud (georeferenced): odm_georeferencing/odm_georeferenced_model.ply

See below for more detailed installation instructions.

## Diving Deeper

### Installation

Extract and enter the downloaded OpenDroneMap directory and compile all of the code by executing a single configuration script (requires root privileges):

    bash configure.sh install

When updating to a newer version of ODM, it is recommended that you run

    bash configure.sh reinstall

to ensure all the dependent packages and modules get updated.  

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

First you need a set of images, taken from a drone or otherwise. Example data can be obtained from https://github.com/OpenDroneMap/odm_data

Next, you need to edit the `settings.yaml` file. The only setting you must edit is the `project-path` key. Set this to an empty directory within projects will be saved. There are many options for tuning your project. See the [wiki](https://github.com/OpenDroneMap/OpenDroneMap/wiki/Run-Time-Parameters) or run `python run.py -h`


Then run:

    python run.py -i /path/to/images project-name

The images will be copied over to the project path so you only need to specify the `-i /path/` once. You can also override any variable from settings.yaml here using the command line arguments. If you want to rerun the whole thing, run

    python run.py --rerun-all project-name

or

    python run.py --rerun-from odm_meshing project-name

The options for rerunning are: 'resize', 'opensfm', 'slam', 'cmvs', 'pmvs', 'odm_meshing', 'mvs_texturing', 'odm_georeferencing', 'odm_orthophoto'

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
        |-- depthmaps/
            |-- merged.ply                  # Dense Point cloud (not georeferenced)
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
        |-- odm_georeferencing_transform.txt# Transform used for georeferencing
        |-- odm_georeferencing_utm_log.txt  # Log for the extract_utm portion
    |-- odm_orthophoto/
        |-- odm_orthophoto.png              # Orthophoto image (no coordinates)
        |-- odm_orthophoto.tif              # Orthophoto GeoTiff
        |-- odm_orthophoto_log.txt          # Log file
        |-- gdal_translate_log.txt          # Log for georeferencing the png file

Any file ending in .obj or .ply can be opened and viewed in [MeshLab](http://meshlab.sourceforge.net/) or similar software. That includes `opensfm/depthmaps/merged.ply`, `odm_meshing/odm_mesh.ply`, `odm_texturing/odm_textured_model[_geo].obj`, or `odm_georeferencing/odm_georeferenced_model.ply`. Below is an example textured mesh:

![](https://raw.githubusercontent.com/alexhagiopol/OpenDroneMap/feature-better-docker/toledo_dataset_example_mesh.jpg)

You can also view the orthophoto GeoTIFF in [QGIS](http://www.qgis.org/) or other mapping software:

![](https://raw.githubusercontent.com/OpenDroneMap/OpenDroneMap/master/img/bellus_map.png)

## Build and Run Using Docker

(Instructions below apply to Ubuntu 14.04, but the Docker image workflow 
has equivalent procedures for Mac OS X and Windows. See [docs.docker.com](https://docs.docker.com/))

OpenDroneMap is Dockerized, meaning you can use containerization to build and run it without tampering with the configuration of libraries and packages already
installed on your machine. Docker software is free to install and use in this context. If you don't have it installed,
see the [Docker Ubuntu installation tutorial](https://docs.docker.com/engine/installation/linux/ubuntulinux/) and follow the
instructions through "Create a Docker group". Once Docker is installed, the fastest way to use OpenDroneMap is to run a pre-built image by typing:

    docker run -it --rm -v $(pwd)/images:/code/images -v $(pwd)/odm_orthophoto:/code/odm_orthophoto -v $(pwd)/odm_texturing:/code/odm_texturing opendronemap/opendronemap

If you want to build your own Docker image from sources, type:

    docker build -t my_odm_image .
    docker run -it --rm -v $(pwd)/images:/code/images -v $(pwd)/odm_orthophoto:/code/odm_orthophoto -v $(pwd)/odm_texturing:/code/odm_texturing my_odm_image

Using this method, the containerized ODM will process the images in the OpenDroneMap/images directory and output results
to the OpenDroneMap/odm_orthophoto and OpenDroneMap/odm_texturing directories as described in the [Viewing Results](https://github.com/OpenDroneMap/OpenDroneMap/wiki/Output-and-Results) section.
If you want to view other results outside the Docker image simply add which directories you're interested in to the run command in the same pattern
established above. For example, if you're interested in the dense cloud results generated by PMVS and in the orthophoto,
simply use the following `docker run` command after building the image:

    docker run -it --rm -v $(pwd)/images:/code/images -v $(pwd)/odm_georeferencing:/code/odm_georeferencing -v $(pwd)/odm_orthophoto:/code/odm_orthophoto my_odm_image

If you want to get all intermediate outputs, run the following command:

    docker run -it --rm -v $(pwd)/images:/code/images -v $(pwd)/odm_georeferencing:/code/odm_georeferencing -v $(pwd)/odm_meshing:/code/odm_meshing -v $(pwd)/odm_orthophoto:/code/odm_orthophoto -v $(pwd)/odm_texturing:/code/odm_texturing -v $(pwd)/opensfm:/code/opensfm -v $(pwd)/pmvs:/code/pmvs opendronemap/opendronemap

To pass in custom parameters to the run.py script, simply pass it as arguments to the `docker run` command. For example:

    docker run -it --rm -v $(pwd)/images:/code/images v $(pwd)/odm_orthophoto:/code/odm_orthophoto -v $(pwd)/odm_texturing:/code/odm_texturing opendronemap/opendronemap --resize-to 1800 --force-ccd 6.16

If you want to pass in custom parameters using the settings.yaml file, you can pass it as a -v volume binding:

    docker run -it --rm -v $(pwd)/images:/code/images v $(pwd)/odm_orthophoto:/code/odm_orthophoto -v $(pwd)/odm_texturing:/code/odm_texturing -v $(pwd)/settings.yaml:/code/settings.yaml opendronemap/opendronemap

When building your own Docker image, if image size is of importance to you, you should use the ```--squash``` flag, like so:

    docker build --squash -t my_odm_image .

This will clean up intermediate steps in the Docker build process, resulting in a significantly smaller image (about half the size).

Experimental flags need to be enabled in Docker to use the ```--squash``` flag. To enable this, insert the following into the file ```/etc/docker/daemon.json```:
    
    {
        "experimental": true
    }

After this, you must restart docker by typing ```sudo service docker restart``` into your Linux terminal.


## User Interface

A web interface and API to OpenDroneMap is currently under active development in the [WebODM](https://github.com/OpenDroneMap/WebODM) repository.

## Video Support

Currently we have an experimental feature that uses ORB_SLAM to render a textured mesh from video. It is only supported on Ubuntu 14.04 on machines with X11 support. See the [wiki](https://github.com/OpenDroneMap/OpenDroneMap/wiki/Reconstruction-from-Video)for details on installation and use.

## Examples

Coming soon...

## Documentation:

For documentation, everything is being moved to [http://docs.opendronemap.org/](http://docs.opendronemap.org/) but you can also take a look at our [wiki](https://github.com/OpenDroneMap/OpenDroneMap/wiki). Check those places first if you are having problems. There's also help at [community forum](http://community.opendronemap.org/), and if you still need help and think you've found a bug or need an enhancement, look through the issue queue or create one. 

## Developers

Help improve our software!

[![Join the chat at https://gitter.im/OpenDroneMap/OpenDroneMap](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/OpenDroneMap/OpenDroneMap?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

1. Try to keep commits clean and simple
2. Submit a pull request with detailed changes and test results
