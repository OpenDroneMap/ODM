.. large

Splitting Large Datasets
========================

A recent ODM update (coined split-merge) introduces a new workflow for splitting up very large datasets into manageable chunks (called submodels), running the pipeline on each chunk, and then producing some merged products.

Why might you use the split-merge pipeline? If you have a very large number of images in your dataset, split-merge will help make the processing more manageable on a large machine. It will also alleviate the need for Ground Control. GPS information gathered from the UAV is still a long way from being accurate, and those problems only get more noticeable as datasets scale. Obviously the best results will come from survey-grade GCPs, but this is not always possible.

What split-merge doesn’t solve is the ability to run large datasets on small machines. We have made strides towards reducing processing costs in general, but the goal of split-merge was specifically to solve a scaling problem, not necessarily an efficiency one.


Calibrate images
----------------

Image calibration is essential for large datasets because error propagation due to image distortion will cause a bowl effect on the models. Calibration instructions can be found at :ref:`calibration`.

Overview
--------

The scripts lay inside the ``scripts/metadataset`` folder. They are:::

    run_all.sh
    setup.py
    run_matching.py
    split.py
    run_reconstructions.py
    align.py
    run_dense.py
    merge.py

If you look into ``run_all.sh`` you will see that you run each of these scripts in the order above. It's really just a placeholder file we used for testing, so I will largely ignore it. Instead I will go through each step in order to explain what it does and how to run it.
Before you run the following scripts, you should set up the environment variables:::

    RUNPATH=<Set this to your ODM base directory, eg. /home/dmb2/opendronemap>
    export PYTHONPATH=$RUNPATH:$RUNPATH/SuperBuild/install/lib/python2.7/dist-packages:$RUNPATH/SuperBuild/src/opensfm:$PYTHONPATH
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$RUNPATH/SuperBuild/install/lib

And make sure you have all the parameters you want to pass in the settings.yaml file in the software root directory. You won't be able to put most of those settings into the command line.

setup.py
````````

This script sets up the metadataset/submodel structure. It takes some arguments:::

    <path-to-project>
    --resize-to n
    --min-num-features n
    --num-cores n
    --matcher-neighbors n
    --submodel-size n
    --submodel-overlap float

``<path-to-project>`` is where the data lies, set up like an ODM project (images should be in an "images" subdirectory), `n` in each of the other parameters is an integer. See https://github.com/OpenDroneMap/OpenDroneMap/wiki/Run-Time-Parameters for more info on the first four. The submodel-size parameter sets how many images are put in each submodel cluster on average. The default is 80 images. It is good to keep the size small because it decreases the bowling effect overall. There also needs to be sufficient overlap between clusters for alignment and merging the models. The submodel-overlap parameter determines how large a radius (in meters) around the cluster to include neighboring images. ::

    python setup.py ~/ODMProjects/split-merge-test/

run_matching.py
```````````````

Before we split the dataset up, we have to match the images so that the software knows where to make the splits. This is done on the whole dataset, so it can take a while if there are a lot of photos.
This one takes nothing except the project path:::

    python run_matching.py ~/ODMProjects/split-merge-test/

split.py
````````

Now we split the model. This will create a directory called “submodels” within which is a set of directories for each submodel. Each of these is set up like a typical ODM project folder, except the images are symlinked to the root images folder. This is an important concept to know because moving the project folder will break the symlinks if you are not careful.::

    python split.py ~/ODMProjects/split-merge-test/

run_reconstructions.py
``````````````````````

Now that we have the submodels, we can run the sparse reconstruction on each. There is an optional argument in this script to run matching on each submodel. You already should have run matching above so we don't need to do it again.::

    --run-matching

Here's what I ran:::

    python run_reconstructions.py ~/ODMProjects/split-merge-test/

align.py
````````

Each submodel is self-referenced, so it’s important to realign the point clouds before getting to the next steps of the pipeline:::

    python align.py ~/ODMProjects/split-merge-test/

run_dense.py
````````````

This is the one that will take a while. It basically runs the rest of the toolchain for each aligned submodel.::

    python run_dense.py ~/ODMProjects/split-merge-test/

And then we wait....

merge.py
````````

The previous script generated an orthophoto for each submodel, and now we have to merge them into a single file. By default it will not overwrite the resulting TIFF so if you need to rerun, make sure you append ``--overwrite``.::

    python merge.py ~/ODMProjects/split-merge-test/ --overwrite


Next steps
----------
This process is a pretty great start to scaling our image processing capabilities, although there is always work to be done. Overall, I would like to see the process streamlined into the standard OpenDroneMap flow. I would also like to see more merged outputs than only the GeoTIFF: the point cloud, textured mesh, and DSM/DTM for starters. Huge props to Pau and the folks at Mapillary for their amazing contributions to OpenDroneMap through their OpenSfM code. I look forward to further pushing the limits of OpenDroneMap and seeing how big a dataset we can process.