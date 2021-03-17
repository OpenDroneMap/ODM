# Orthorectification Tool

This tool is capable of orthorectifing individual images (or all images) from an ODM reconstruction. It does not account for visibility occlusion, so you will get artifacts near buildings (help us improve this?).

## Usage

After running a reconstruction using ODM:

```
docker run -ti --rm -v /home/youruser/datasets:/datasets opendronemap/odm --project-path /datasets project
```

You can run the orthorectification module by running:

```
docker run -ti --rm -v /home/youruser/datasets:/datasets --entrypoint /code/contrib/orthorectify/orthorectify.py opendronemap/odm /datasets/project
```

This will start the orthorectification process for all images in the dataset. See additional flags you can pass at the end of the command above:

```
usage: orthorectify.py [-h] [--dem DEM] [--no-alpha NO_ALPHA]
                       [--interpolation {nearest,bilinear}]
                       [--outdir OUTDIR] [--image-list IMAGE_LIST]
                       [--images IMAGES]
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
  --images IMAGES       Comma-separeted list of filenames to
                        rectify. Use as an alternative to --image-
                        list. Default: process all images.
```

## Roadmap

Help us improve this module! We could add:

 - [ ] GPU support for faster processing
 - [ ] Visibility checks
 - [ ] Merging of multiple orthorectified images (blending, filtering, seam leveling)