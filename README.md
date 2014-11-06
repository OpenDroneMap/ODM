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

Sadly, it does not do all of this yet. So far, it does step 1: Point Clouds. But outputs 2-4 are on their way shortly, so stay tuned.

Steps to get OpenDroneMap running:
==================================

(Requires Ubuntu 12.04 32-bit at this time. May also work with Ubuntu 12.04 64-bit.  See https://github.com/OpenDroneMap/odm_vagrant for running on Windows in a VM)

Run install.sh to build.

``` ./install.sh ```

From a directory full of your images, run

``` ./run.pl ```

---


From Meshlab 1.3.3:

    * Open Project file, navigate to:
		* <project_location>/reconstruction-with-image-size-1200/bundle/bundle.out
	* It will prompt for the image list file
		* <project_location>/reconstruction-with-image-size-1200/list.txt
	* Control-L and delete "0 model"
	* Import dense point cloud:
		* e.g. <project_location>/reconstruction-with-image-size-1200-results/option-0000.ply
		* (there may be multiple ply files)
	* Make a mesh:
		* Filters:Remeshing, Simplification and Reconstruction:Surface Reconstruction Poisson
	* Texture the mesh
		* Parameterization + texturing from registered rasters

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

This means that your camera is not in the database, https://github.com/OpenDroneMap/OpenDroneMap/blob/gh-pages/ccd_defs.pl

This problem is easily remedied. We need to know CCD size in the camera. We'll get these for our Sony Cyber-shot DSC-HX5 from dpreview: http://www.dpreview.com/products/sony/compacts/sony_dschx5/specifications

So, we'll add the following line to our ccd_defs.pl:

     "SONY DSC-HX5V"                            => 6.104,  # 1/2.4"
     
And so others can use it, we'll do a pull request to add it to our array for everyone else.
