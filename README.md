![](https://opendronemap.github.io/OpenDroneMap/img/odm_image.png)

What is it?
===========

OpenDroneMap is a toolchain for processing raw civilian UAS imagery to other useful products. What kind of products?

1. Point Clouds
2. Digital Surface Models
3. Textured Digital Surface Models
4. Orthorectified Imagery
5. Classified Point Clouds
6. Digital Elevation Models
7. etc.

So far, it does Point Clouds, Digital Surface Models, Textured Digital Surface Models, and Orthorectified Imagery.

Steps to get OpenDroneMap running:
==================================

<del>(Requires Ubuntu 12.04 32-bit at this time. May also work with Ubuntu 12.04 64-bit.</del>

(Requires Ubuntu 12.04 or later, see https://github.com/OpenDroneMap/odm_vagrant for running on Windows in a VM)

Run install.sh to build.

``` ./install.sh ```

From a directory full of your images, run

``` ./run.pl ```

An overview of installing and running OpenDroneMap on Ubuntu can be found here: https://www.youtube.com/watch?v=e2qp3o8caPs

and here:

https://www.youtube.com/watch?v=7ZTufQkODLs

Now that texturing is in the code base, you can access the full textured meshes using MeshLab. Open MeshLab, and choose File:Import Mesh and choose your textured mesh from a location similar to the following: reconstruction-with-image-size-1200-results\odm_texturing\odm_textured_model.obj

---

Example data can be found at https://github.com/OpenDroneMap/odm_data

---

Long term, the aim is for the toolchain to also be able to optionally push to a variety of online data repositories, pushing hi-resolution aerials to [OpenAerialMap](http://opentopography.org/), point clouds to [OpenTopography](http://opentopography.org/), and pushing digital elevation models to an emerging global repository (yet to be named...). That leaves only digital surface model meshes and UV textured meshes with no global repository home.

---

Troubleshooting:
================

If you run ODM with your own camera, it is possible you will see something like this:

```
  - configuration:
    --cmvs-maxImages: 100
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
     
And so others can use it, we'll do a pull request to add it to our array for everyone else.
