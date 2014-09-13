Steps to get OpenDroneMap running:
==================================

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
