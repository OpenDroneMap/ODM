![](https://opendronemap.github.io/OpenDroneMap/img/odm_image.png)

Steps to get OpenDroneMap running:
==================================

(Requires Ubuntu 12.04 32-bit at this time. May also work with Ubuntu 12.04 64-bit)

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
