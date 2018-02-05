from opendm import io
from opendm import log
from opendm import system
import argparse
from osgeo import ogr
import os
from opensfm.large import metadataset


def create_bounds_file(clusters_geojson_path):
    # Create a convex hull around the boundary
    # as to encompass the entire area (no holes)
    driver = ogr.GetDriverByName('GeoJSON')
    ds = driver.Open(clusters_geojson_path, 0) # read-only
    in_layer = ds.GetLayer()


    # Save to a new file
    out_path = io.extract_path_from_file(clusters_geojson_path)
    bounds_geojson_path = os.path.join(out_path, 'bounds.geojson')
    if os.path.exists(bounds_geojson_path):
        driver.DeleteDataSource(bounds_geojson_path)

    out_ds = driver.CreateDataSource(bounds_geojson_path)
    out_layer = out_ds.CreateLayer("bounds.geojson", geom_type=ogr.wkbPolygon)
    out_layer.CreateField(ogr.FieldDefn('ID', ogr.OFTInteger))

    layer_def = in_layer.GetLayerDefn()

    feature_def = in_layer.GetLayerDefn()

    # For each submodel, create a convex hull
    num_clusters = 0
    # get number of submodels
    for in_feat in in_layer:
        x = in_feat.GetFieldAsInteger('submodel')
        if x > num_clusters:
            num_clusters = x
    num_clusters += 1
    log.ODM_DEBUG("Number of clusters: {}".format(num_clusters))

    in_layer.ResetReading()

    hull_collection = ogr.Geometry(ogr.wkbGeometryCollection)
    for i in range(num_clusters):

        # Collect all Geometry
        geomcol = ogr.Geometry(ogr.wkbGeometryCollection)
        for in_feat in in_layer:
            if in_feat.GetFieldAsInteger('submodel') == i:
                # add point to geometry feature
                geomcol.AddGeometry(in_feat.GetGeometryRef())
        in_layer.ResetReading()

        # Calculate convex hull for each feature
        convexhull = geomcol.ConvexHull()
        hull_collection.AddGeometry(convexhull)

        ## geomcol.Destroy()

    feat_iter = 0

    for feat in hull_collection:

        out_feat = ogr.Feature(feature_def)
        out_feat.SetGeometry(feat)
        # add ID
        out_feat.SetField(0, feat_iter)
        feat_iter += 1
        out_layer.CreateFeature(out_feat)
        out_feat = None

    # Save and close data sources
    out_ds = ds = None

    return bounds_geojson_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Align metadaset submodels')
    parser.add_argument('dataset',
                        help='path to the dataset to be processed')
    parser.add_argument('--overwrite', '-o',
                        action='store_true',
                        default=False,
                        help='Force overwrite of generated files')
    args = parser.parse_args()

    submodels_path = io.join_paths(args.dataset, 'submodels')

    path = os.path.join(args.dataset, 'opensfm')
    meta_data = metadataset.MetaDataSet(path)
    data = metadataset.DataSet(path)
    bounds_file = None

    clusters_file = os.path.join(args.dataset, "submodels/opensfm/clusters_with_neighbors.geojson")
    if io.file_exists(clusters_file):
        log.ODM_DEBUG("Creating cluster bounds")
        bounds_file = create_bounds_file(clusters_file)
    else:
        log.ODM_ERROR("Clusters file not found")
        exit()

    if not io.file_exists(bounds_file):
        log.ODM_ERROR("Bounds file not created. Exiting...")
    else:
        # List of tifs paths to merge
        ortho_tifs = {}
        for folder in os.listdir(io.join_paths(args.dataset, 'submodels')):
            if 'submodel' in folder:
                folder_number = folder.split('_')[1]  # string extract number
                tif_file = io.join_paths(submodels_path, folder + "/odm_orthophoto/odm_orthophoto.tif")
                if io.file_exists(tif_file):
                    ortho_tifs[folder_number] = tif_file

        kwargs = {
            'f_out': io.join_paths(submodels_path, 'big-ole-tiff.tif'),
            'files': ' '.join(ortho_tifs.values()),
            'clusters': bounds_file
        }

        if io.file_exists(kwargs['f_out']) and not args.overwrite:
            log.ODM_ERROR("File {f_out} exists, use --overwrite to force overwrite of file.".format(**kwargs))
        else:
            # use bounds as cutlines (blending)
            system.run('gdal_merge.py -o {f_out} '
                       '-createonly '
                       '-co "BIGTIFF=YES" '
                       '-co "BLOCKXSIZE=512" '
                       '-co "BLOCKYSIZE=512" {files}'.format(**kwargs)
                       )

            for tif in ortho_tifs:
                kwargs['name'] = '0' if tif == '0000' else tif.lstrip('0')  # is tif a tuple?
                kwargs['file'] = ortho_tifs[tif]
                system.run('gdalwarp -cutline {clusters} '
                           '-cwhere "NAME = \'{name}\'" '
                           '-r lanczos -multi -wo NUM_THREADS=ALL_CPUS '
                           '{file} {f_out}'.format(**kwargs)
                )
