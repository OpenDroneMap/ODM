# Fix Ply

Use to translate a modified ply into a compatible format for subsequent steps in ODM. Via Jaime Chacoff, https://community.opendronemap.org/t/edited-point-cloud-with-cloudcompare-wont-rerun-from-odm-meshing/21449/6

The basic idea is to process through ODM until the point cloud is created, use a 3rd party tool, like CloudCompare to edit the point cloud, and then continue processing in OpenDroneMap.

This useful bit of python will convert the PLY exported from CloudCompare back into a compatible format for continued processing in OpenDroneMap.

1. Run project in WebODM and add this to your settings: `end-with: odm-filterpoints`
1. Once complete, go to your NodeODM container and copy `/var/www/data/[Task ID]/odm-filterpoints` directory
1. Open CloudCompare and from `odm-filterpoints` directory you've copied, open `point_cloud.ply`
1. In the box that pops up, add a scalar field `vertex - views`
1. To see the actual colours again - select the point cloud, then in properties change colours from "Scalar field" to "RGB"
1. Make your changes to the point cloud
1. Compute normals (Edit > Normals > Compute)
1. Save PLY file as ASCII
1. Run Python file above to fix PLY file and convert to binary
1. Copy `odm_filterpoints` directory (or just `point_cloud.ply`) back into NodeODM container
1. Restart project in WebODM "From Meshing" (don't forget to edit settings to remove `end-with: odm-filterpoints` or it's not going to do anything).
