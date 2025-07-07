# Plugin Time-SIFT

This script does Time-SIFT processing with ODM. Time-SIFT is a method for multi-temporal analysis without the need to co-registrate the data.

>  D. Feurer, F. Vinatier, Joining multi-epoch archival aerial images in a single SfM block allows 3-D change detection with almost exclusively image information, ISPRS Journal of Photogrammetry and Remote Sensing, Volume 146, 2018, Pages 495-506, ISSN 0924-2716, doi: 10.1016/j.isprsjprs.2018.10.016
(https://doi.org/10.1016/j.isprsjprs.2018.10.016)

## Requirements
* ODM ! :-) 
* subprocess
* json
* os
* shutil
* pathlib
* sys
* argparse
* textwrap

## Usage

### Provided example
Download or clone [this repo](https://forge.inrae.fr/Denis.Feurer/timesift-odm-data-example.git) to get example data.

Then execute
```
python Timesift_odm.py datasets --end-with odm_filterpoints
```
It should make the Time-SIFT processing on the downloaded example data, stopping after the filtered dense clouds step.

In the destination dir, you should obtain new directories, ```0_before``` and ```1_after``` at the same level as the ```time-sift-block``` directory. These new directories contain all the results natively co-registered.

You can then use [CloudCompare](https://cloudcompare.org/) to compute distance between the ```datasets/0_before/odm_filterpoints/point_cloud.ply``` and the ```datasets/1_after/odm_filterpoints/point_cloud.ply``` and obtain this image showing the difference between the two 3D surfaces. Here, two soil samples were excavated as can be seen on the image below.
![](https://forge.inrae.fr/Denis.Feurer/timesift-odm-data-example/-/raw/main/Example.png?ref_type=heads)

### Your own data
In your dataset directory (usually ```datasets```, but you can have chosen another name) you have to prepare a Time-SIFT project directory (default name : ```time-sift-block```, *can be tuned via a parameter*) that contains :
   * ```images/``` : a subdirectory with all images of all epochs. This directory name is fixed as it is the one expected by ODM
   * ```images_epochs.txt``` : a file that has the same format as the file used for the split and merge ODM function. This file name *can be tuned via a parameter*.

The ```images_epochs.txt``` file has two columns, the first column contains image names and the second contains the epoch name as follows
```
DSC_0368.JPG 0_before
DSC_0369.JPG 0_before
DSC_0370.JPG 0_before
DSC_0389.JPG 1_after
DSC_0390.JPG 1_after
DSC_0391.JPG 1_after
```

Your directory, before running the script, should look like this :
```
$PWD/datasets/
└── time-sift-block/
    ├── images/
    └── images_epochs.txt
```

At the end of the script you obtain a directory by epoch (at the same level as the Time-SIFT project directory). Each directory is processed with images of each epoch and all results are natively co-registered due to the initial sfm step done with all images.
```
$PWD/datasets/
├── 0_before/
├── 1_after/
└── time-sift-block/
```