import os, sys, shutil, tempfile, math, json
from opendm import system
from opendm import log
from opendm import context
from opendm.system import run
from opendm import entwine
from opendm import io
from opendm.concurrency import parallel_map
from opendm.utils import double_quote
from opendm.boundary import as_polygon, as_geojson
from opendm.dem.pdal import run_pipeline
from opendm.opc import classify
from opendm.dem import commands

def ply_info(input_ply):
    if not os.path.exists(input_ply):
        raise IOError("%s does not exist" % input_ply)

    # Read PLY header, check if point cloud has normals
    has_normals = False
    has_views = False
    vertex_count = 0

    with open(input_ply, 'r', errors='ignore') as f:
        line = f.readline().strip().lower()
        i = 0
        while line != "end_header":
            line = f.readline().strip().lower()
            props = line.split(" ")
            if len(props) == 3:
                if props[0] == "property" and props[2] in ["nx", "normalx", "normal_x"]:
                    has_normals = True
                if props[0] == "property" and props[2] in ["views"]:
                    has_views = True
                elif props[0] == "element" and props[1] == "vertex":
                    vertex_count = int(props[2])
            i += 1
            if i > 100:
                raise IOError("Cannot find end_header field. Invalid PLY?")
            

    return {
        'has_normals': has_normals,
        'vertex_count': vertex_count,
        'has_views': has_views,
        'header_lines': i + 1
    }


def split(input_point_cloud, outdir, filename_template, capacity, dims=None):
    log.ODM_INFO("Splitting point cloud filtering in chunks of {} vertices".format(capacity))

    if not os.path.exists(input_point_cloud):
        log.ODM_ERROR("{} does not exist, cannot split point cloud. The program will now exit.".format(input_point_cloud))
        sys.exit(1)

    if not os.path.exists(outdir):
        system.mkdir_p(outdir)

    if len(os.listdir(outdir)) != 0:
        log.ODM_ERROR("%s already contains some files. The program will now exit.".format(outdir))
        sys.exit(1)

    cmd = 'pdal split -i "%s" -o "%s" --capacity %s ' % (input_point_cloud, os.path.join(outdir, filename_template), capacity)
    
    if filename_template.endswith(".ply"):
        cmd += ("--writers.ply.sized_types=false "
                "--writers.ply.storage_mode=\"little endian\" ")
    if dims is not None:
        cmd += '--writers.ply.dims="%s"' % dims
    system.run(cmd)

    return [os.path.join(outdir, f) for f in os.listdir(outdir)]


def filter(input_point_cloud, output_point_cloud, output_stats, standard_deviation=2.5, sample_radius=0, boundary=None, max_concurrency=1):
    """
    Filters a point cloud
    """
    if not os.path.exists(input_point_cloud):
        log.ODM_ERROR("{} does not exist. The program will now exit.".format(input_point_cloud))
        sys.exit(1)

    args = [
        '--input "%s"' % input_point_cloud,
        '--output "%s"' % output_point_cloud,
        '--concurrency %s' % max_concurrency
    ]

    if sample_radius > 0:
        log.ODM_INFO("Sampling points around a %sm radius" % sample_radius)
        args.append('--radius %s' % sample_radius)

    meank = 16
    log.ODM_INFO("Filtering {} (statistical, meanK {}, standard deviation {})".format(input_point_cloud, meank, standard_deviation))
    args.append('--meank %s' % meank)
    args.append('--std %s' % standard_deviation)
    args.append('--stats "%s"' % output_stats)
    
    if boundary is not None:
        log.ODM_INFO("Boundary {}".format(boundary))
        fd, boundary_json_file = tempfile.mkstemp(suffix='.boundary.json')
        os.close(fd)
        with open(boundary_json_file, 'w') as f:
            f.write(as_geojson(boundary))
        args.append('--boundary "%s"' % boundary_json_file)

    system.run('"%s" %s' % (context.fpcfilter_path, " ".join(args)))

    if not os.path.exists(output_point_cloud):
        log.ODM_WARNING("{} not found, filtering has failed.".format(output_point_cloud))


def get_spacing(stats_file, resolution_fallback=5.0):
    def fallback():
        log.ODM_WARNING("Cannot read %s, falling back to resolution estimate" % stats_file)
        return (resolution_fallback / 100.0) / 2.0

    if not os.path.isfile(stats_file):
        return fallback()
    
    with open(stats_file, 'r') as f:
        j = json.loads(f.read())
        if "spacing" in j:
            d = j["spacing"]
            if d > 0:
                return round(d, 3)
            else:
                return fallback()
        else:
            return fallback()

def export_info_json(pointcloud_path, info_file_path):
    system.run('pdal info --dimensions "X,Y,Z" "{0}" > "{1}"'.format(pointcloud_path, info_file_path))


def export_summary_json(pointcloud_path, summary_file_path):
    system.run('pdal info --summary "{0}" > "{1}"'.format(pointcloud_path, summary_file_path))

def get_extent(input_point_cloud):
    fd, json_file = tempfile.mkstemp(suffix='.json')
    os.close(fd)
    
    # Get point cloud extent
    fallback = False

    # We know PLY files do not have --summary support
    if input_point_cloud.lower().endswith(".ply"):
        fallback = True
        run('pdal info "{0}" > "{1}"'.format(input_point_cloud, json_file))

    try:
        if not fallback:
            run('pdal info --summary "{0}" > "{1}"'.format(input_point_cloud, json_file))
    except:
        fallback = True
        run('pdal info "{0}" > "{1}"'.format(input_point_cloud, json_file))

    bounds = {}
    with open(json_file, 'r') as f:
        result = json.loads(f.read())
        
        if not fallback:
            summary = result.get('summary')
            if summary is None: raise Exception("Cannot compute summary for %s (summary key missing)" % input_point_cloud)
            bounds = summary.get('bounds')
        else:
            stats = result.get('stats')
            if stats is None: raise Exception("Cannot compute bounds for %s (stats key missing)" % input_point_cloud)
            bbox = stats.get('bbox')
            if bbox is None: raise Exception("Cannot compute bounds for %s (bbox key missing)" % input_point_cloud)
            native = bbox.get('native')
            if native is None: raise Exception("Cannot compute bounds for %s (native key missing)" % input_point_cloud)
            bounds = native.get('bbox')

        if bounds is None: raise Exception("Cannot compute bounds for %s (bounds key missing)" % input_point_cloud)
        
        if bounds.get('maxx', None) is None or \
            bounds.get('minx', None) is None or \
            bounds.get('maxy', None) is None or \
            bounds.get('miny', None) is None or \
            bounds.get('maxz', None) is None or \
            bounds.get('minz', None) is None:
            raise Exception("Cannot compute bounds for %s (invalid keys) %s" % (input_point_cloud, str(bounds)))
            
    os.remove(json_file)
    return bounds


def merge(input_point_cloud_files, output_file, rerun=False):
    num_files = len(input_point_cloud_files)
    if num_files == 0:
        log.ODM_WARNING("No input point cloud files to process")
        return

    if io.file_exists(output_file):
        log.ODM_WARNING("Removing previous point cloud: %s" % output_file)
        os.remove(output_file)

    kwargs = {
        'all_inputs': " ".join(map(double_quote, input_point_cloud_files)),
        'output': output_file
    }

    system.run('lasmerge -i {all_inputs} -o "{output}"'.format(**kwargs))
   

def fast_merge_ply(input_point_cloud_files, output_file):
    # Assumes that all input files share the same header/content format
    # As the merge is a naive byte stream copy

    num_files = len(input_point_cloud_files)
    if num_files == 0:
        log.ODM_WARNING("No input point cloud files to process")
        return
    
    if io.file_exists(output_file):
        log.ODM_WARNING("Removing previous point cloud: %s" % output_file)
        os.remove(output_file)
    
    vertex_count = sum([ply_info(pcf)['vertex_count'] for pcf in input_point_cloud_files])
    master_file = input_point_cloud_files[0]
    with open(output_file, "wb") as out:
        with open(master_file, "r", errors="ignore") as fhead:
            # Copy header
            line = fhead.readline()
            out.write(line.encode('utf8'))

            i = 0
            while line.strip().lower() != "end_header":
                line = fhead.readline()
                
                # Intercept element vertex field
                if line.lower().startswith("element vertex "):
                    out.write(("element vertex %s\n" % vertex_count).encode('utf8'))
                else:
                    out.write(line.encode('utf8'))

                i += 1
                if i > 100:
                    raise IOError("Cannot find end_header field. Invalid PLY?")
            
        for ipc in input_point_cloud_files:
            i = 0
            with open(ipc, "rb") as fin:
                # Skip header
                line = fin.readline()
                while line.strip().lower() != b"end_header":
                    line = fin.readline()

                    i += 1
                    if i > 100:
                        raise IOError("Cannot find end_header field. Invalid PLY?")
                
                # Write fields
                out.write(fin.read())
    
    return output_file


def merge_ply(input_point_cloud_files, output_file, dims=None):
    num_files = len(input_point_cloud_files)
    if num_files == 0:
        log.ODM_WARNING("No input point cloud files to process")
        return

    cmd = [
        'pdal',
        'merge',
        '--writers.ply.sized_types=false',
        '--writers.ply.storage_mode="little endian"',
        ('--writers.ply.dims="%s"' % dims) if dims is not None else '',
        ' '.join(map(double_quote, input_point_cloud_files + [output_file])),
    ]

    system.run(' '.join(cmd))

def post_point_cloud_steps(args, tree, rerun=False):
    # Classify and rectify before generating derivate files
    if args.pc_classify:
        pc_classify_marker = os.path.join(tree.odm_georeferencing, 'pc_classify_done.txt')

        if not io.file_exists(pc_classify_marker) or rerun:
            log.ODM_INFO("Classifying {} using Simple Morphological Filter (1/2)".format(tree.odm_georeferencing_model_laz))
            commands.classify(tree.odm_georeferencing_model_laz,
                                args.smrf_scalar, 
                                args.smrf_slope, 
                                args.smrf_threshold, 
                                args.smrf_window
                            )

            log.ODM_INFO("Classifying {} using OpenPointClass (2/2)".format(tree.odm_georeferencing_model_laz))
            classify(tree.odm_georeferencing_model_laz, args.max_concurrency)

            with open(pc_classify_marker, 'w') as f:
                f.write('Classify: smrf\n')
                f.write('Scalar: {}\n'.format(args.smrf_scalar))
                f.write('Slope: {}\n'.format(args.smrf_slope))
                f.write('Threshold: {}\n'.format(args.smrf_threshold))
                f.write('Window: {}\n'.format(args.smrf_window))
    
    if args.pc_rectify:
        commands.rectify(tree.odm_georeferencing_model_laz)

    # XYZ point cloud output
    if args.pc_csv:
        log.ODM_INFO("Creating CSV file (XYZ format)")
        
        if not io.file_exists(tree.odm_georeferencing_xyz_file) or rerun:
            system.run("pdal translate -i \"{}\" "
                "-o \"{}\" "
                "--writers.text.format=csv "
                "--writers.text.order=\"X,Y,Z\" "
                "--writers.text.keep_unspecified=false ".format(
                    tree.odm_georeferencing_model_laz,
                    tree.odm_georeferencing_xyz_file))
        else:
            log.ODM_WARNING("Found existing CSV file %s" % tree.odm_georeferencing_xyz_file)

    # LAS point cloud output
    if args.pc_las:
        log.ODM_INFO("Creating LAS file")
        
        if not io.file_exists(tree.odm_georeferencing_model_las) or rerun:
            system.run("pdal translate -i \"{}\" "
                "-o \"{}\" ".format(
                    tree.odm_georeferencing_model_laz,
                    tree.odm_georeferencing_model_las))
        else:
            log.ODM_WARNING("Found existing LAS file %s" % tree.odm_georeferencing_xyz_file)

    # EPT point cloud output
    if args.pc_ept:
        log.ODM_INFO("Creating Entwine Point Tile output")
        entwine.build([tree.odm_georeferencing_model_laz], tree.entwine_pointcloud, max_concurrency=args.max_concurrency, rerun=rerun)

    # COPC point clouds
    if args.pc_copc:
        log.ODM_INFO("Creating Cloud Optimized Point Cloud (COPC)")

        copc_output = io.related_file_path(tree.odm_georeferencing_model_laz, postfix=".copc")
        entwine.build_copc([tree.odm_georeferencing_model_laz], copc_output, convert_rgb_8_to_16=True)