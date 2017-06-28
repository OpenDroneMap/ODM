# Visible Vegetation Indexes

This script produces a Vegetation Index raster from a RGB orthophoto (odm_orthophoto.tif in your project)

## Requirements
* rasterio (pip install rasterio)
* numpy python package (included in ODM build)

## Usage
```
vegind.py <orthophoto.tif> index

positional arguments:
  <orthophoto.tif>  The RGB orthophoto. Must be a GeoTiff.
  index     Index identifier. Allowed values: ngrdi, tgi, vari
```
Output will be generated with index suffix in the same directory as input.

## Examples

`python vegind.py /path/to/odm_orthophoto.tif tgi`

Orthophoto photo of Koniaków grass field and forest in QGIS: ![](http://imgur.com/K6x3nB2.jpg)
The Triangular Greenness Index output in QGIS (with a spectral pseudocolor): ![](http://i.imgur.com/f9TzISU.jpg)
Visible Atmospheric Resistant Index: ![](http://imgur.com/Y7BHzLs.jpg)
Normalized green-red difference index: ![](http://imgur.com/v8cmaPS.jpg)

## Bibliography

1. Hunt, E. Raymond, et al. "A Visible Band Index for Remote Sensing Leaf Chlorophyll Content At the Canopy Scale." ITC journal 21(2013): 103-112. doi: 10.1016/j.jag.2012.07.020
(https://doi.org/10.1016/j.jag.2012.07.020)
