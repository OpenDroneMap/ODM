from opendm import io
from opendm import log
from opendm import system
from opendm import context
import argparse
from functools import partial
import os
import ecto
from opensfm.large import metadataset
from scipy.spatial import Voronoi
from shapely.geometry import shape, LineString, Point
import shapely.ops
import numpy as np
import json
import pyproj


class SMMergeCell(ecto.Cell):

    def declare_params(self, params):
        params.declare("merge_overwrite", 'force overwrite of merged tiff', False)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("sm_meta", "SplitMerge metadata", [])
        outputs.declare("sm_meta", "SplitMerge metadata", [])

    def process(self, inputs, outputs):
        args = self.inputs.args
        tree = self.inputs.tree

        command = os.path.join(context.opensfm_path, 'bin', 'opensfm')
        path = tree.opensfm

        submodels_path = tree.submodels_path
        sfm_path = tree.opensfm
        meta_data = metadataset.MetaDataSet(sfm_path)
        data = metadataset.DataSet(sfm_path)
        voronoi_file = io.join_paths(meta_data.data_path, 'voronoi.geojson')
        proj_path = tree.odm_georeferencing_proj
        out_tif = tree.out_tif
        addo_log = tree.addo_log

        bounds_files = {}
        for folder in os.listdir(submodels_path):
            if 'submodel' in folder:
                folder_number = '0' if folder.split('_')[1] == '0000' else folder.split('_')[1].lstrip('0')
                bounds_file = io.join_paths(submodels_path, folder +
                                            "/odm_georeferencing/odm_georeferenced_model.bounds.geojson")
                if io.file_exists(bounds_file):
                    bounds_files[folder_number] = bounds_file

        ## Run points merge
        points_merge()

        ## Run the simple merge
        old_ortho_merge()

        # example: get_submodel_files("odm_georeferencing", "odm_georeferenced_model.laz")
        def get_submodel_files(directory, filename):
            files = []
            for folder in os.listdir(submodels_path):
                if 'submodel' in folder:
                    folder_number = '0' if folder.split('_')[1] == '0000' else folder.split('_')[1].lstrip('0')
                    file = io.join_paths(submodels_path, folder + directory, filename)
                    if io.file_exists(file):
                        files += file
            return files

        def points_merge():
            fin = get_submodel_files("odm_georeferencing", "odm_georeferenced_model.laz")
            cmd = [
                'pdal',
                'merge',
                '-i %s' % ' '.join(fin),
                '-o %s' % fout
            ]
            if maxWindowSize is not None:
                cmd.append('--filters.smrf.window=%s' % maxWindowSize)

            if verbose:
                print ' '.join(cmd)

            out = system.run(' '.join(cmd))
            if verbose:
                print out


        def old_ortho_merge():
            # Do voronoi calcs
            # # load clusters
            images, positions, labels, centers = meta_data.load_clusters()
            cluster_proj = pyproj.Proj(init='epsg:4326')
            with open(proj_path, 'r') as fr:
                transform_proj = pyproj.Proj(fr.read())

            # projection transformation
            project = partial(
                pyproj.transform,
                cluster_proj,
                transform_proj)

            # turn this into a list of points
            pos_transformed = [shapely.ops.transform(project, Point(x[1], x[0])) for x in positions]

            #back to ndarray
            positions = np.array([pos_transformed[x].coords[0] for x in range(len(pos_transformed))])

            clust = np.concatenate((images, labels, positions), 1)
            # Run voronoi on the whole cluster
            vor = Voronoi(clust[:, [2, 3]].astype(float))
            lines = [
                LineString(vor.vertices[line])
                for line in vor.ridge_vertices
                if -1 not in line
            ]
            # # For each part, build a boundary polygon
            v_poly_dis_intersected = {}
            for subnum in np.unique(clust[:, 1]):
                submodel = clust[clust[:, 1] == subnum]
                polygons = []
                for poly in shapely.ops.polygonize(lines):
                    for point in submodel:
                        if poly.contains(Point(point[[2, 3]])):
                            polygons.append(poly)           # Todo: this is expensive
                            break
                    # Dissolve list of polyogns
                    voronoi_polygons_dissolved = shapely.ops.unary_union(polygons)
                # intersect with bounds
                with open(bounds_files[subnum]) as f:
                    # There should only be one polygon here
                    bounds = shape(json.loads(f.read())['features'][0]['geometry'])

                v_poly_dis_intersected[subnum] = voronoi_polygons_dissolved.intersection(bounds)

            features = []
            for submodel in v_poly_dis_intersected:
                features.append({
                    "type": "Feature",
                    "geometry": shapely.geometry.mapping(v_poly_dis_intersected[submodel]),
                    "properties": {
                        "submodel": int(submodel)
                    }
                })

            polygons_layer = {
                "type": "FeatureCollection",
                "features": features,
                "crs": {"type": "name", "properties": {"name": transform_proj.srs, "type": "proj4"}}
            }

            with open(voronoi_file, "w") as f:
                json.dump(polygons_layer, f)

            ortho_tifs = {}
            for folder in os.listdir(submodels_path):
                if 'submodel' in folder:
                    folder_number = folder.split('_')[1]  # string extract number
                    tif_file = io.join_paths(submodels_path, folder + "/odm_orthophoto/odm_orthophoto.tif")
                    if io.file_exists(tif_file):
                        ortho_tifs[folder_number] = tif_file

            kwargs = {
                    'f_out': out_tif,
                    'files': ' '.join(ortho_tifs.values()),
                    'clusters': voronoi_file
                }

            if io.file_exists(kwargs['f_out']) and not args.merge_overwrite:
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
                               '-cwhere "submodel = \'{name}\'" '
                               '-r lanczos -multi -wo NUM_THREADS=ALL_CPUS '
                               ' {file} {f_out}'.format(**kwargs)
                    )

                log.ODM_INFO("Building Overviews")
                kwargs = {
                    'orthophoto': out_tif,
                    'log': addo_log
                }
                # Run gdaladdo
                system.run('gdaladdo -ro -r average '
                           '--config BIGTIFF_OVERVIEW IF_SAFER '
                           '--config COMPRESS_OVERVIEW JPEG '
                           '{orthophoto} 2 4 8 16 > {log}'.format(**kwargs))

