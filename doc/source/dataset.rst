.. Explaining the dataset structure

Dataset Structure
=================
::

    project/
    ├── images/
    │   ├── img-1234.jpg
    │   └── ...
    ├── opensfm/                            # Tie Points and camera positions here in JSON format
    │   ├── config.yaml
    │   ├── images/
    │   ├── masks/
    │   ├── gcp_list.txt
    │   ├── metadata/
    │   ├── features/
    │   ├── matches/
    │   ├── tracks.tsv
    │   ├── reconstruction.json
    │   ├── reconstruction.meshed.json
    │   ├── undistorted/
    │   ├── undistorted_tracks.json
    │   ├── undistorted_reconstruction.json
    │   └── depthmaps/
    │      └── merged.ply                   # Dense Point Cloud
    ├── odm_meshing/
    │   ├── odm_mesh.ply                    # A 3D mesh
    │   └── odm_meshing_log.txt             # Output of the meshing task. May point out errors.
    ├── odm_texturing/
    │   ├── odm_textured_model.obj          # Textured mesh
    │   ├── odm_textured_model_geo.obj      # Georeferenced textured mesh
    │   └── texture_N.jpg                   # Associated textured images used by the model
    ├── odm_georeferencing/
    │   ├── odm_georeferenced_model.ply     # A georeferenced dense point cloud
    │   ├── odm_georeferenced_model.ply.laz # LAZ format point cloud
    │   ├── odm_georeferenced_model.csv     # XYZ format point cloud
    │   ├── odm_georeferencing_log.txt      # Georeferencing log
    │   └── odm_georeferencing_utm_log.txt  # Log for the extract_utm portion
    ├── odm_orthophoto/
    │   ├── odm_orthophoto.png              # Orthophoto image (no coordinates)
    │   ├── odm_orthophoto.tif              # Orthophoto GeoTiff
    │   ├── odm_orthophoto_log.txt          # Log file
    │   └── gdal_translate_log.txt          # Log for georeferencing the png file
    └── odm_dem/
        ├── odm_dsm.tif                     # Digital Surface Model Geotiff - the tops of everything
        └── odm_dtm.tif                     # Digital Terrain Model Geotoff - the ground.

Outputs
```````
Listed below are some of the useful outputs ODM produces

3D Models (unreferenced)
^^^^^^^^^^^^^^^^^^^^^^^^

``pmvs/recon0/models/option-0000.ply`` -- The point cloud file

``odm_meshing/odm_mesh.ply`` -- The meshed surface

Textured Models
^^^^^^^^^^^^^^^

``odm_texturing/odm_textured_model.obj`` -- The textured surface mesh

You can access the point cloud and textured meshes using MeshLab. Open MeshLab, and choose File:Import Mesh and choose your textured mesh from a location similar to the following: ``odm_texturing\odm_textured_model.obj``

Georeferencing
^^^^^^^^^^^^^^

``odm_texturing/odm_textured_model_geo.obj`` -- The georeferenced and textured surface mesh

``odm_georeferenced_model.ply/laz/csv`` -- The georeferenced point cloud in different file formats

Orthophoto
^^^^^^^^^^

``odm_orthphoto.png`` -- The orthophoto, but this is a simple png, which doesn't have any georeferencing information

``odm_orthphoto.tif`` -- GeoTIFF Orthophoto. You can use it in QGIS as a raster layer.


DEM/DSM
^^^^^^^

DEM/DSM will only be created if the ``--dem`` or ``--dsm`` options are used.

``odm_dem.tif``

``odm_dsm.tif``

2.5D Meshing Reconstruction
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``--use-25dmesh`` option will generate two extra directories, ``odm_texturing_25d/`` and ``odm_25dgeoreferencing``. The items inside correspond to those inside the 3D meshed output folders, only they were run using the 2.5D meshing algorithm.