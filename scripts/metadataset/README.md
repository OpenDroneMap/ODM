
# Split and merge pipeline for large-scale reconstructions

Large datasets can be slow to process.  An option to speed up the reconstruction process is to split them into smaller datasets.  We will call each of the small datasets a *submodel*.  Smaller datasets run faster because they involve fewer images on each bundle adjustment iteration.  Additionally, the reconstruction of the different submodels can be done in parallel.

Since the reconstructions of the submodels are done independently, they will not be necessarily aligned with each other.  Only the GPS positions of the images and the ground control points will determine the alignment.  When the neighboring reconstructions share cameras or points, it is possible to enforce the alignment of common cameras and points between the different reconstructions.

Here, we describe the OpenDroneMap pipeline for splitting a large dataset and aligning the resulting submodels.  The pipeline uses the OpenSfM commands documented [here](http://opensfm.readthedocs.io/en/latest/large.html) and combines them with the rest of the ODM pipeline.

The main workflow is as follows
  - Initial setup
  - Run feature detection and matching
  - Splitting the dataset
  - Running SfM reconstruction on each submodel
  - Aligning the reconstructions
  - Run dense matching and the rest of the ODM pipeline for each of the aligned reconstructions

The script `run_all.sh` runs all the steps but it is also possible to run one by one.  It the following we describe what each command does.

## Initial setup
The `setup.py` command initializes the dataset and writes the config file for OpenSfM.  The command accepts command line parameters to configure the process.

A first group of parameters are equivalent to the standard ODM parameters and configure the feature extraction and matching: `--resize-to`, `--min-num-features`, `--num-cores`, `--matcher-neighbors`.

A second group of parameters controls the size and overlap of the submodels.  They are equivalent to the [OpenSfM parameters](http://opensfm.readthedocs.io/en/latest/large.html#config-parameters) with the same name.

  - `submodel_size`: Average number of images per submodel.  When splitting a large dataset into smaller submodels, images are grouped into clusters.  This value regulates the number of images that each cluster should have on average.  The splitting is done via K-means clustering with `k` set to the number of images divided by `submodel_size`.

  - `submodel_overlap`: Radius of the overlapping region between submodels in meters.  To be able to align the different submodels, there needs to be some common images between the neighboring submodels.  Any image that is closer to a cluster than `submodel_overlap` it is added to that cluster.

Finally, if you already know how you want to split the dataset, you can provide that information and it will be used instead of the clustering algorithm.

The grouping can be provided by adding a file named `image_groups.txt` in the main dataset folder.  The file should have one line per image.  Each line should have two words: first the name of the image and second the name of the group it belongs to.  For example:

    01.jpg A
    02.jpg A
    03.jpg B
    04.jpg B
    05.jpg C

will create 3 submodels.

## Run feature detection and matching
The `run_matching.py` command runs feature extraction and matching for all images in the dataset.  These are reused for each submodel.

## Splitting the dataset
The `split.py` command, runs OpenSfM's `create_submodels` command to split the dataset into submodels.  It uses the parameters in `dataset/OpenSfM/config.yaml`, which are set by the `setup.py` described above.

The submodels are created with the following directory structure

    dataset/
    |-- image_groups.txt
    |-- images/
    |-- opensfm/
    |   |-- camera_models.json
    |   |-- config.yaml
    |   |-- image_list.txt
    |   |-- exif/
    |   |-- features/
    |   |-- matches/
    |   |-- image_groups.txt -> ../image_groups.txt
    |   |-- profile.log
    |   `-- reference_lla.json
    `-- submodels/
        |-- opensfm/
        |   |-- clusters.npz
        |   |-- clusters_with_neighbors.geojson
        |   |-- clusters_with_neighbors.npz
        |   `-- image_list_with_gps.tsv
        |-- submodel_0000/
        |   |-- images/
        |   |-- opensfm/
        |   |   |-- config.yaml
        |   |   |-- image_list.txt
        |   |   |-- camera_models.json -> ../../../opensfm/camera_models.json
        |   |   |-- exif -> ../../../opensfm/exif
        |   |   |-- features -> ../../../opensfm/features
        |   |   |-- matches -> ../../../opensfm/matches
        |   |   `-- reference_lla.json -> ../../../opensfm/reference_lla.json
        |   |-- odm_meshing/
        |   |-- odm_texturing/
        |   |-- odm_georeferencing/
        |   `-- odm_orthophoto/
        |-- submodel_0001/
        |   `-- ...
        `-- ...


## Running SfM reconstruction on each submodel
The command `run_reconstructions.py` will run create SfM reconstruction for each submodel.  This will only create the sparse reconstructions, which are stored in the file `opensfm/reconstruction.json` on each submodel folder.

It will run multiple reconstructions in parallel with the number of processes specified by the `--num-cores` options in the setup.

## Aligning the reconstructions
Once all submodels have been reconstructed, the `align.py` command will improve the alignment between each other.  The result is a sparse reconstruction stored in `opensfm/reconstruction.aligned.json` on each submodel folder.

## Run dense matching, meshing and texturing
Now that each submodel has a sparse reconstruction and that they are all aligned, the rest of the ODM pipeline can be run normally. The command `run_dense.py` will run dense matching, meshing and texturing for each submodel independently.
