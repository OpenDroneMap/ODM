# GRASS scripts
# odm_grass
Generates DEM, contour and textured relief maps.

## Requirements
* GRASS
* Environment variables:
  * PYTHONHOME set to the location of Python
  * PYTHONPATH set to the location of GRASS Python libs
  * PATH includes GRASS bin and lib directories
  * GISBASE set to the location of GRASS

## Usage
    python odm_grass.py <project-path>

Output is `<project-path>/odm_georeferencing/odm_dem.tif`, `<project-path>/odm_georeferencing/odm_contour.shp` and `<project-path>/odm_orthophoto/odm_relief.tif`.