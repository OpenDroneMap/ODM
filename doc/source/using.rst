.. Usage

Usage
=====


.. _docker-usage:

Docker
------

There are two methods for running with docker. One pulls a pre-built image from the docker hub. This is the most reliable. You can also :ref:`build your own image <docker-installation>`. In either case, the run command is the same, what you will change is the name of the image. For the docker hub image, use ``opendronemap/opendronemap``. For an image you built yourself, use that image name (in our case, ``my_odm_image``).::

    docker run -it --rm \
        -v $(pwd)/images:/code/images \
        -v $(pwd)/odm_texturing:/code/odm_texturing \
        -v $(pwd)/odm_orthophoto:/code/odm_orthophoto \
        <docker-image>

``-v`` is used to connect folders in the docker container to local folders. See :doc:`dataset` for reference on the project layout.

If you want to get all intermediate outputs, run the following command:::

    docker run -it --rm \
        -v $(pwd)/images:/code/images \
        -v $(pwd)/odm_meshing:/code/odm_meshing \
        -v $(pwd)/odm_orthophoto:/code/odm_orthophoto \
        -v $(pwd)/odm_georeferencing:/code/odm_georeferencing \
        -v $(pwd)/odm_texturing:/code/odm_texturing \
        -v $(pwd)/opensfm:/code/opensfm \
        -v $(pwd)/pmvs:/code/pmvs \
        opendronemap/opendronemap

To pass in custom parameters to the run.py script, simply pass it as arguments to the docker run command. For example:::

    docker run -it --rm \
        -v $(pwd)/images:/code/images \
        -v $(pwd)/odm_orthophoto:/code/odm_orthophoto \
        -v $(pwd)/odm_texturing:/code/odm_texturing \
        opendronemap/opendronemap --resize-to 1800 --force-ccd 6.16

If you want to pass in custom parameters using the settings.yaml file, you can pass it as a -v volume binding:::

    docker run -it --rm \
        -v $(pwd)/images:/code/images \
        -v $(pwd)/odm_orthophoto:/code/odm_orthophoto \
        -v $(pwd)/odm_texturing:/code/odm_texturing \
        -v $(pwd)/settings.yaml:/code/settings.yaml \
        opendronemap/opendronemap

For more information about Docker, check out their `docs <https://docs.docker.com/>`_.

.. _native-usage:

Native
------


First thing you need to do is set the project path. Edit the ``settings.yaml`` file to add your projects folder::

    # This line is really important to set up properly
    project_path: '' # Example: '/home/user/ODMProjects'

    # The rest of the settings will default to the values set unless you uncomment and change them
    #resize_to: 2400

You must change ``project_path: ''`` to add an absolute path to somewhere on your machine. Whenever you run a new project, it will be saved here.

To use OpenDroneMap run the following command::

    python run.py --images </path/to/images> [arguments] <project-name>

Then sit back, grab a coffee and wait. You only have to specify ``--images </path/to/images>`` on the first run.

.. _arguments:

Additional Arguments
````````````````````

Args::

  -h, --help            show this help message and exit
  --images <path>, -i <path>
                        Path to input images
  --project-path <path>
                        Path to the project folder
  --resize-to <integer>
                        resizes images by the largest side for opensfm. Set to
                        -1 to disable. Default: 2048
  --start-with <string>, -s <string>
                        Can be one of: dataset | opensfm | slam | cmvs | pmvs
                        | odm_meshing | odm_25dmeshing | mvs_texturing |
                        odm_georeferencing | odm_dem | odm_orthophoto
  --end-with <string>, -e <string>
                        Can be one of:dataset | opensfm | slam | cmvs | pmvs |
                        odm_meshing | odm_25dmeshing | mvs_texturing |
                        odm_georeferencing | odm_dem | odm_orthophoto
  --rerun <string>, -r <string>
                        Can be one of:dataset | opensfm | slam | cmvs | pmvs |
                        odm_meshing | odm_25dmeshing | mvs_texturing |
                        odm_georeferencing | odm_dem | odm_orthophoto
  --rerun-all           force rerun of all tasks
  --rerun-from <string>
                        Can be one of:dataset | opensfm | slam | cmvs | pmvs |
                        odm_meshing | odm_25dmeshing | mvs_texturing |
                        odm_georeferencing | odm_dem | odm_orthophoto
  --video <string>      Path to the video file to process
  --slam-config <string>
                        Path to config file for orb-slam
  --force-focal <positive float>
                        Override the focal length information for the images
  --proj <PROJ4 string>
                        Projection used to transform the model into geographic
                        coordinates
  --force-ccd <positive float>
                        Override the ccd width information for the images
  --min-num-features <integer>
                        Minimum number of features to extract per image. More
                        features leads to better results but slower execution.
                        Default: 8000
  --matcher-neighbors <integer>
                        Number of nearest images to pre-match based on GPS
                        exif data. Set to 0 to skip pre-matching. Neighbors
                        works together with Distance parameter, set both to 0
                        to not use pre-matching. OpenSFM uses both parameters
                        at the same time, Bundler uses only one which has
                        value, prefering the Neighbors parameter. Default: 8
  --matcher-distance <integer>
                        Distance threshold in meters to find pre-matching
                        images based on GPS exif data. Set both matcher-
                        neighbors and this to 0 to skip pre-matching. Default:
                        0
  --use-fixed-camera-params
                        Turn off camera parameter optimization during bundler
  --opensfm-processes <positive integer>
                        The maximum number of processes to use in dense
                        reconstruction. Default: 16
  --use-hybrid-bundle-adjustment
                        Run local bundle adjustment for every image added to
                        the reconstruction and a global adjustment every 100
                        images. Speeds up reconstruction for very large
                        datasets.
  --use-25dmesh         Use a 2.5D mesh to compute the orthophoto. This option
                        tends to provide better results for planar surfaces.
                        Experimental.
  --use-pmvs            Use pmvs to compute point cloud alternatively
  --cmvs-maxImages <integer>
                        The maximum number of images per cluster. Default: 500
  --pmvs-level <positive integer>
                        The level in the image pyramid that is used for the
                        computation. see
                        http://www.di.ens.fr/pmvs/documentation.html for more
                        pmvs documentation. Default: 1
  --pmvs-csize <positive integer>
                        Cell size controls the density of
                        reconstructionsDefault: 2
  --pmvs-threshold <float: -1.0 <= x <= 1.0>
                        A patch reconstruction is accepted as a success and
                        kept if its associated photometric consistency measure
                        is above this threshold. Default: 0.7
  --pmvs-wsize <positive integer>
                        pmvs samples wsize x wsize pixel colors from each
                        image to compute photometric consistency score. For
                        example, when wsize=7, 7x7=49 pixel colors are sampled
                        in each image. Increasing the value leads to more
                        stable reconstructions, but the program becomes
                        slower. Default: 7
  --pmvs-min-images <positive integer>
                        Each 3D point must be visible in at least minImageNum
                        images for being reconstructed. 3 is suggested in
                        general. Default: 3
  --pmvs-num-cores <positive integer>
                        The maximum number of cores to use in dense
                        reconstruction. Default: 16
  --mesh-size <positive integer>
                        The maximum vertex count of the output mesh Default:
                        100000
  --mesh-octree-depth <positive integer>
                        Oct-tree depth used in the mesh reconstruction,
                        increase to get more vertices, recommended values are
                        8-12. Default: 9
  --mesh-samples <float >= 1.0>
                        Number of points per octree node, recommended and
                        default value: 1.0
  --mesh-solver-divide <positive integer>
                        Oct-tree depth at which the Laplacian equation is
                        solved in the surface reconstruction step. Increasing
                        this value increases computation times slightly but
                        helps reduce memory usage. Default: 9
  --mesh-neighbors <positive integer>
                        Number of neighbors to select when estimating the
                        surface model used to compute the mesh and for
                        statistical outlier removal. Higher values lead to
                        smoother meshes but take longer to process. Applies to
                        2.5D mesh only. Default: 24
  --mesh-resolution <positive float>
                        Size of the interpolated surface model used for
                        deriving the 2.5D mesh, expressed in pixels per meter.
                        Higher values work better for complex or urban
                        terrains. Lower values work better on flat areas.
                        Resolution has no effect on the number of vertices,
                        but high values can severely impact runtime speed and
                        memory usage. When set to zero, the program
                        automatically attempts to find a good value based on
                        the point cloud extent and target vertex count.
                        Applies to 2.5D mesh only. Default: 0
  --fast-orthophoto     Skips dense reconstruction and 3D model generation. It
                        generates an orthophoto directly from the sparse
                        reconstruction. If you just need an orthophoto and do
                        not need a full 3D model, turn on this option.
                        Experimental.
  --crop <positive float>
                        Automatically crop image outputs by creating a smooth
                        buffer around the dataset boundaries, shrinked by N
                        meters. Use 0 to disable cropping. Default: 3
  --pc-classify <string>
                        Classify the .LAS point cloud output using either a
                        Simple Morphological Filter or a Progressive
                        Morphological Filter. If --dtm is set this parameter
                        defaults to smrf. You can control the behavior of both
                        smrf and pmf by tweaking the --dem-* parameters.
                        Default: none
  --texturing-data-term <string>
                        Data term: [area, gmi]. Default: gmi
  --texturing-outlier-removal-type <string>
                        Type of photometric outlier removal method: [none,
                        gauss_damping, gauss_clamping]. Default:
                        gauss_clamping
  --texturing-skip-visibility-test
                        Skip geometric visibility test. Default: False
  --texturing-skip-global-seam-leveling
                        Skip global seam leveling. Useful for IR data.Default:
                        False
  --texturing-skip-local-seam-leveling
                        Skip local seam blending. Default: False
  --texturing-skip-hole-filling
                        Skip filling of holes in the mesh. Default: False
  --texturing-keep-unseen-faces
                        Keep faces in the mesh that are not seen in any
                        camera. Default: False
  --texturing-tone-mapping <string>
                        Turn on gamma tone mapping or none for no tone
                        mapping. Choices are 'gamma' or 'none'. Default: none
  --gcp <path string>   path to the file containing the ground control points
                        used for georeferencing. Default: None. The file needs
                        to be on the following line format: easting northing
                        height pixelrow pixelcol imagename
  --use-exif            Use this tag if you have a gcp_list.txt but want to
                        use the exif geotags instead
  --dtm                 Use this tag to build a DTM (Digital Terrain Model,
                        ground only) using a progressive morphological filter.
                        Check the --dem* parameters for fine tuning.
  --dsm                 Use this tag to build a DSM (Digital Surface Model,
                        ground + objects) using a progressive morphological
                        filter. Check the --dem* parameters for fine tuning.
  --dem-gapfill-steps <positive integer>
                        Number of steps used to fill areas with gaps. Set to 0
                        to disable gap filling. Starting with a radius equal
                        to the output resolution, N different DEMs are
                        generated with progressively bigger radius using the
                        inverse distance weighted (IDW) algorithm and merged
                        together. Remaining gaps are then merged using nearest
                        neighbor interpolation. Default=4
  --dem-resolution <float>
                        Length of raster cell edges in meters. Default: 0.1
  --dem-maxangle <positive float>
                        Points that are more than maxangle degrees off-nadir
                        are discarded. Default: 20
  --dem-maxsd <positive float>
                        Points that deviate more than maxsd standard
                        deviations from the local mean are discarded. Default:
                        2.5
  --dem-initial-distance <positive float>
                        Used to classify ground vs non-ground points. Set this
                        value to account for Z noise in meters. If you have an
                        uncertainty of around 15 cm, set this value large
                        enough to not exclude these points. Too small of a
                        value will exclude valid ground points, while too
                        large of a value will misclassify non-ground points
                        for ground ones. Default: 0.15
  --dem-approximate     Use this tag use the approximate progressive
                        morphological filter, which computes DEMs faster but
                        is not as accurate.
  --dem-decimation <positive integer>
                        Decimate the points before generating the DEM. 1 is no
                        decimation (full quality). 100 decimates ~99% of the
                        points. Useful for speeding up generation. Default=1
  --dem-terrain-type <string>
                        One of: FlatNonForest, FlatForest, ComplexNonForest,
                        ComplexForest. Specifies the type of terrain. This
                        mainly helps reduce processing time. FlatNonForest:
                        Relatively flat region with little to no vegetation
                        FlatForest: Relatively flat region that is forested
                        ComplexNonForest: Varied terrain with little to no
                        vegetation ComplexForest: Varied terrain that is
                        forested Default=ComplexForest
  --orthophoto-resolution <float > 0.0>
                        Orthophoto ground resolution in pixels/meterDefault:
                        20.0
  --orthophoto-target-srs <EPSG:XXXX>
                        Target spatial reference for orthophoto creation. Not
                        implemented yet. Default: None
  --orthophoto-no-tiled
                        Set this parameter if you want a stripped geoTIFF.
                        Default: False
  --orthophoto-compression <string>
                        Set the compression to use. Note that this could break
                        gdal_translate if you don't know what you are doing.
                        Options: JPEG, LZW, PACKBITS, DEFLATE, LZMA, NONE.
                        Default: DEFLATE
  --orthophoto-bigtiff {YES,NO,IF_NEEDED,IF_SAFER}
                        Control whether the created orthophoto is a BigTIFF or
                        classic TIFF. BigTIFF is a variant for files larger
                        than 4GiB of data. Options are YES, NO, IF_NEEDED,
                        IF_SAFER. See GDAL specs:
                        https://www.gdal.org/frmt_gtiff.html for more info.
                        Default: IF_SAFER
  --build-overviews     Build orthophoto overviews using gdaladdo.
  --zip-results         compress the results using gunzip
  --verbose, -v         Print additional messages to the console Default:
                        False
  --time                Generates a benchmark file with runtime info Default:
                        False
  --version             Displays version number and exits.

.. _ground-control:

Ground Control Points
`````````````````````

If you supply a GCP file called gcp_list.txt then ODM will automatically detect it. If it has another name you can specify using ``--gcp <path>``. If you have a gcp file and want to do georeferencing with exif instead, then you can specify ``--use-exif``.

`This post has some information about placing Ground Control Targets before a flight <http://diydrones.com/profiles/blogs/ground-control-points-gcps-for-aerial-photography>`_, but if you already have images, you can find your own points in the images post facto. It's important that you find high-contrast objects that are found in **at least** 3 photos, and that you find a minimum of 5 objects.

For example, in this image, I would use the sharp corners of the diamond-shaped bioswales in the parking lot:

.. image:: _static/tol_sm.jpg

You should also place/find the GCPs evenly around your survey area.

The ``gcp_list.txt`` file must then be created in the base of your project folder:

The format of the GCP file is simple. The header line is a description of the coordinate system, which must be written as a http://spatialreference.org/ is a good resource for finding that information. proj4 string. Please note that currently angular coordinates (like lat/lon) do not work. Subsequent lines are the X, Y & Z coordinate in your coordinate system, your associated pixel and line number in the image, and the image name itself::

    coordinate system description
    x1 y1 z1 pixelx1 pixely1 imagename1
    x2 y2 z2 pixelx2 pixely2 imagename2
    x3 y3 z3 pixelx3 pixely3 imagename3

e.g. for the Langley dataset::

    WGS84 UTM 10N
    544256.7 5320919.9 5 3044 2622 IMG_0525.jpg
    544157.7 5320899.2 5 4193 1552 IMG_0585.jpg
    544033.4 5320876.0 5 1606 2763 IMG_0690.jpg


Given the recommendations above, your file should have a minimum of 15 lines after the header (5 points with 3 images to each point).

Video Reconstruction (Experimental)
```````````````````````````````````

**Note: This is an experimental feature**

It is possible to build a reconstruction using a video file instead of still images.  The technique for reconstructing the camera trajectory from a video is called Simultaneous Localization And Mapping (SLAM).  OpenDroneMap uses the opensource `ORB_SLAM2 <https://github.com/raulmur/ORB_SLAM2>`_ library for this task.

We will explain here how to use it.  We will need to build the SLAM module, calibrate the camera and finally run the reconstruction from a video.


Building with SLAM support
^^^^^^^^^^^^^^^^^^^^^^^^^^

By default, OpenDroneMap does not build the SLAM module.  To build it we need to do the following two steps

**Build SLAM dependencies**::

    sudo apt-get install libglew-dev
    cd SuperBuild/build
    cmake -DODM_BUILD_SLAM=ON .
    make
    cd ../..

**Build the SLAM module**::

    cd build
    cmake -DODM_BULID_SLAM=ON .
    make
    cd ..


.. _calibration:

Calibrating the camera
^^^^^^^^^^^^^^^^^^^^^^

The SLAM algorithm requires the camera to be calibrated.  It is difficult to extract calibration parameters from the video's metadata as we do when using still images.  Thus, it is required to run a calibration procedure that will compute the calibration from a video of a checkerboard.

We will start by **recording the calibration video**.  Display this `chessboard pattern <https://dl.dropboxusercontent.com/u/2801164/odm/chessboard.pdf>`_ on a large screen, or `print it on a large paper and stick it on a flat surface <http://www.instructables.com/id/How-to-make-a-camera-calibration-pattern/>`_.  Now record a video pointing the camera to the chessboard.

While recording move the camera to both sides and up and down always maintaining the entire pattern framed.  The goal is to capture the pattern from different points of views.


Now you can **run the calibration script** as follows::

    python modules/odm_slam/src/calibrate_video.py --visual PATH_TO_CHESSBOARD_VIDEO.mp4

You will see a window displaying the video and the detected corners.  When it finish, it will print the computed calibration parameters. They should look like this (with different values)::

    # Camera calibration and distortion parameters (OpenCV)
    Camera.fx: 1512.91332401
    Camera.fy: 1512.04223185
    Camera.cx: 956.585155225
    Camera.cy: 527.321715394

    Camera.k1: 0.140581949184
    Camera.k2: -0.292250537695
    Camera.p1: 0.000188785464717
    Camera.p2: 0.000611510377372
    Camera.k3: 0.181424769625

Keep this text.  We will use it on the next section.


Running OpenDroneMap from a video
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We are now ready to run the OpenDroneMap pipeline from a video.  For this we need the video and a config file for ORB_SLAM2.  Here's an `example config.yaml <https://dl.dropboxusercontent.com/u/2801164/odm/config.yaml>`_.  Before using it, copy-paste the calibration parameters for your camera that you just computed on the previous section.

Put the video and the `config.yaml` file on an empty folder.  Then run OpenDroneMap using the following command::

    python run.py --project-path PROJECT_PATH --video VIDEO.mp4 --slam-config config.yaml --resize-to VIDEO_WIDTH

where ``PROJECT_PATH`` is the path to the folder containing the video and config file, ``VIDEO.mp4`` is the name of your video, and ``VIDEO_WIDTH`` is the width of the video (for example, 1920 for an HD video).

That command will run the pipeline starting with SLAM and continuing with stereo matching and mesh reconstruction and texturing.

When done, the textured model will be in ``PROJECT_PATH/odm_texturing/odm_textured_model.obj``.  The point cloud created by the stereo matching algorithm will be in ``PROJECT_PATH/pmvs/recon0/models/option-0000.ply``


.. _camera-calibration:

Camera Calibration
------------------

It is highly recommended that you calibrate your images to reduce lens distortion. Doing so will increase the likelihood of finding quality matches between photos and reduce processing time. You can do this in Photoshop or `ImageMagick <http://www.imagemagick.org/Usage/lens/>`_. We also have some simple scripts to perform this task: https://github.com/OpenDroneMap/CameraCalibration . This suite of scripts will find camera matrix and distortion parameters with a set of checkerboard images, then use those parameters to remove distortion from photos.

Installation
````````````

You need to install numpy and opencv:::

    pip install numpy
    sudo apt-get install python-opencv exiftool

Usage: Calibrate chessboard
```````````````````````````

First you will need to take some photos of a black and white chessboard with a white border, `like this one <https://raw.githubusercontent.com/LongerVision/OpenCV_Examples/master/markers/pattern_chessboard.png>`_.

Then you will run the opencv_calibrate.py script to generate the matrix and distortion files.::

    python opencv_calibrate.py ./sample/chessboard/ 10 7

The first argument is the path to the chessboard. You will also have to input the chessboard dimensions (the number of squares in x and y) Optional arguments:::

    --out           path      if you want to output the parameters and the image outputs to a specific path. otherwise it gets writting to ./out
    --square_size   float     if your chessboard squares are not square, you can change this. default is 1.0

Usage: undistort photos
```````````````````````

With the photos and the produced matrix.txt and distortion.txt, run the following:::

    python undistort.py --matrix matrix.txt --distortion distortion.txt "/path/to/images/"

Note: Do not forget the quotes in "/path/to/images"

Docker Usage for undistorting images
````````````````````````````````````

The ``undistort.py`` script depends on exiftool to copy exif metadata to the new images, so on Windows you may have to use Docker for the undistort step. Put the matrix.txt and distortion.txt in their own directory (eg. sample/config) and do the following:::

    docker build -t cc_undistort .
    docker run -v ~/CameraCalibration/sample/images:/app/images \
               -v ~/CameraCalibration/sample/config:/app/config \
               cc_undistort

