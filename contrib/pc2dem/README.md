# Point Cloud To DEM

Convert point clouds (LAS, LAZ, PLY, and any other format compatible with [PDAL](https://pdal.io/stages/readers.html)) to GeoTIFF elevation models.

![image](https://user-images.githubusercontent.com/1951843/112354653-492a5100-8ca3-11eb-9f21-4dda4cae976f.png)

This tool includes methods to perform efficient and scalable gapfill interpolation and is the same method used by ODM's processing pipeline. It is offered here as a standalone module for processing individual point clouds.

## Usage

```
docker run -ti --rm -v /home/youruser/folder_with_point_cloud:/input --entrypoint /code/contrib/pc2dem/pc2dem.py opendronemap/odm /input/point_cloud.las [flags]
```

The result (`dsm.tif` or `dtm.tif`) will be stored in the same folder as the input point cloud. See additional `flags` you can pass at the end of the command above:

```
usage: pc2dem.py [-h] [--type {dsm,dtm}] [--resolution RESOLUTION]
                 [--gapfill-steps GAPFILL_STEPS]
                 point_cloud

Generate DEMs from point clouds using ODM's algorithm.

positional arguments:
  point_cloud           Path to point cloud file (.las, .laz,
                        .ply)

optional arguments:
  -h, --help            show this help message and exit
  --type {dsm,dtm}      Type of DEM. Default: dsm
  --resolution RESOLUTION
                        Resolution in m/px. Default: 0.05
  --gapfill-steps GAPFILL_STEPS
                        Number of steps used to fill areas with
                        gaps. Set to 0 to disable gap filling.
                        Starting with a radius equal to the output
                        resolution, N different DEMs are generated
                        with progressively bigger radius using the
                        inverse distance weighted (IDW) algorithm
                        and merged together. Remaining gaps are
                        then merged using nearest neighbor
                        interpolation. Default: 3

```
