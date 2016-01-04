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

Run install.sh to build.

    ./install.sh

From a directory full of your images, run

    ./run.py

An overview of installing and running OpenDroneMap on Ubuntu can be found here: https://www.youtube.com/watch?v=e2qp3o8caPs

Here are some other videos:

- https://www.youtube.com/watch?v=7ZTufQkODLs (2015-01-30)
- https://www.youtube.com/watch?v=m0i4GQdfl8A (2015-03-15)

Now that texturing is in the code base, you can access the full textured meshes using MeshLab. Open MeshLab, choose `File:Import Mesh` and choose your textured mesh from a location similar to the following: `reconstruction-with-image-size-1200-results\odm_texturing\odm_textured_model.obj`

For Ubuntu 15.10 users, this will help you get running: 
```
sudo apt-get install python-xmltodict
sudo ln -s /usr/lib/x86_64-linux-gnu/libproj.so.9 /usr/lib/libproj.so
```

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


Troubleshooting:
================

Make sure you have enough RAM and CPU. Only lowercase file extension supported now.

If you run ODM with your own camera, it is possible you will see something like this:

```
  - configuration:
    --cmvs-maxImages: 500
    --end-with: pmvs
    --match-size: 200
    --matcher-ratio: 0.6
    --matcher-threshold: 2
    --pmvs-csize: 2
    --pmvs-level: 1
    --pmvs-minImageNum: 3
    --pmvs-threshold: 0.7
    --pmvs-wsize: 7
    --resize-to: 1200
    --start-with: resize


  - source files - Fri Sep 19 13:47:42 UTC 2014


    no CCD width or focal length found for DSC05391.JPG - camera: "SONY DSC-HX5V"
    no CCD width or focal length found for DSC05392.JPG - camera: "SONY DSC-HX5V"
    no CCD width or focal length found for DSC05393.JPG - camera: "SONY DSC-HX5V"
    no CCD width or focal length found for DSC05394.JPG - camera: "SONY DSC-HX5V"
    no CCD width or focal length found for DSC05395.JPG - camera: "SONY DSC-HX5V"
    no CCD width or focal length found for DSC05396.JPG - camera: "SONY DSC-HX5V"
    no CCD width or focal length found for DSC05397.JPG - camera: "SONY DSC-HX5V"
    no CCD width or focal length found for DSC05398.JPG - camera: "SONY DSC-HX5V"
    no CCD width or focal length found for DSC05399.JPG - camera: "SONY DSC-HX5V"

    found no usable images - quitting
Died at ../../OpenDroneMap/./run.pl line 364.


```

This means that your camera is not in the database, https://github.com/OpenDroneMap/OpenDroneMap/blob/gh-pages/ccd_defs.json

This problem is easily remedied. We need to know CCD size in the camera. We'll get these for our Sony Cyber-shot DSC-HX5 from dpreview: http://www.dpreview.com/products/sony/compacts/sony_dschx5/specifications

So, we'll add the following line to our ccd_defs.json:

     "SONY DSC-HX5V": 6.104,

To check that ccd_defs.json compiles, run `ccd_defs_check.py`
If it prints the message 'CCD_DEFS compiles OK', then you can commit your changes.

And so others can use it, we'll do a pull request to add it to our array for everyone else.

---

Maintainers can run the ccd_defs.json compilation test automatically by creating a
symbolic link in .git/hooks to hooks/pre-commit

     cd .git/hooks
     ln -s ../../hooks/pre-commit

If ccd_defs.json does not compile, then the pre-commit hook will abort the commit.
