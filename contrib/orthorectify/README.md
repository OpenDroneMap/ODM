# Orthorectification Tool

![image](https://user-images.githubusercontent.com/1951843/111536715-fc91c380-8740-11eb-844c-5b7960186391.png)

This tool is capable of orthorectifying individual images (or all images) from an existing ODM reconstruction.

![image](https://user-images.githubusercontent.com/1951843/111529183-3ad6b500-8738-11eb-9960-b1aa676f863b.png)

## Usage

After running a reconstruction using ODM:

```
docker run -ti --rm -v /home/youruser/datasets:/datasets opendronemap/odm --project-path /datasets project
```

You can run the orthorectification module by running:

```
docker run -ti --rm -v /home/youruser/datasets:/datasets --entrypoint /code/contrib/orthorectify/run.sh opendronemap/odm /datasets/project
```

This will start the orthorectification process for all images in the dataset. See additional flags you can pass at the end of the command above:

```
usage: orthorectify.py [-h] [--dem DEM] [--no-alpha NO_ALPHA]
                       [--interpolation {nearest,bilinear}]
                       [--outdir OUTDIR] [--image-list IMAGE_LIST]
                       [--images IMAGES] [--threads THREADS]
                       [--skip-visibility-test SKIP_VISIBILITY_TEST]
                       dataset

Orthorectification Tool

positional arguments:
  dataset               Path to ODM dataset

optional arguments:
  -h, --help            show this help message and exit
  --dem DEM             Absolute path to DEM to use to
                        orthorectify images. Default:
                        odm_dem/dsm.tif
  --no-alpha NO_ALPHA   Don't output an alpha channel
  --interpolation {nearest,bilinear}
                        Type of interpolation to use to sample
                        pixel values.Default: bilinear
  --outdir OUTDIR       Output directory where to store results.
                        Default: orthorectified
  --image-list IMAGE_LIST
                        Path to file that contains the list of
                        image filenames to orthorectify. By
                        default all images in a dataset are
                        processed. Default: img_list.txt
  --images IMAGES       Comma-separated list of filenames to
                        rectify. Use as an alternative to --image-
                        list. Default: process all images.
  --skip-visibility-test SKIP_VISIBILITY_TEST
                        Skip visibility testing (faster but leaves
                        artifacts due to relief displacement)
```

## Roadmap

Help us improve this module! We could add:

 - [ ] GPU support for faster processing
 - [ ] Merging of multiple orthorectified images (blending, filtering, seam leveling)
 - [ ] Faster visibility test
 - [ ] Different methods for orthorectification (direct)