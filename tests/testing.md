
## opendm/
### config.py
* test each parameter (min, max, out of bounds, etc.)

### context.py
* Each path must be a valid, existing path

### io.py
* check each function with a known string

### system.py
* (Get_ccd_widths is depreciated)
* parse_coordinate_system:
  * This is also old code - new one is in types

### types.py
* ODM_Photos
  * updated focal equal to known focal length
  * updated ccd equal to known ccd
* ODM_Georef
  * calculate_epsg needs to be updated, see master
  * convert_to_las: assert existence of las file output
  * utm_to_latlon:
    * 
    * 

## scripts/
### dataset.py
* test that supported_extensions works with a variety of file names, even bogus ones
* check outputs are all the photos

### resize.py
* resulting images are the right size 
* metadata has been properly updated

### OpenSfM.py
* config file is contains info same as params
* when matcher_distance > 0 it is written to the config
* at least one reconstruction file is generated
* check that bundler file is exported (possible to check if valid?)

### CMVS.py
* validate params
* system.run() command is equal to some known string

### PMVS.py
* validate params
* system.run() command is equal to some known string

### odm_meshing.py
* validate params
* system.run() command is equal to some known string

### odm_texturing.py
* validate params
* system.run() command is equal to some known string

### odm_georeferencing.py
* validate params
* system.run() command is equal to some known string when using EXIF coords
* system.run() when using GCP

### odm_orthophoto.py
* validate params
* system.run() command is equal to some known string
