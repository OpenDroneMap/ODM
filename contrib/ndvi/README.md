# NDVI 

This script produces a NDVI raster from a CIR orthophoto (odm_orthophoto.tif in your project)

## Requirements
* python_gdal package from apt
* numpy python package (included in ODM build)

## Usage
```
ndvi.py [-h] [--overwrite] <orthophoto.tif> N N <outfile.tif>

positional arguments:
  <orthophoto.tif>  The CIR orthophoto. Must be a GeoTiff.
  N                 NIR band number
  N                 Vis band number
  <outfile.tif>     The output file. Also must be in GeoTiff format

optional arguments:
  -h, --help        show this help message and exit
  --overwrite, -o   Will overwrite output file if it exists.
```

**Argument order matters! NIR first, then VIS**

## Examples:
Use the [Seneca](https://github.com/OpenDroneMap/odm_data_seneca) dataset for a good working CIR. The band order for that set is NIR-G-B, so you will want to use bands 1 and 2 for this script. After running ODM, the command goes as follows:

`python ndvi.py /path/to/odm_orthophoto.tif 1 2 /path/to/ndvi.tif`

The output in QGIS (with a spectral pseudocolor): ![](http://i.imgur.com/TdLECII.png)