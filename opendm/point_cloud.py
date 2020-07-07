import os, sys, shutil, tempfile, json
from opendm import system
from opendm import log
from opendm import context
from opendm.system import run
from opendm import entwine
from opendm import io
from pipes import quote

def ply_has_normals(input_ply):
    if not os.path.exists(input_ply):
        return False

    # Read PLY header, check if point cloud has normals
    has_normals = False
    with open(input_ply, 'r') as f:
        line = f.readline().strip().lower()
        i = 0
        while line != "end_header" and i < 100:
            line = f.readline().strip().lower()
            props = line.split(" ")
            if len(props) == 3 and props[0] == "property" and props[2] in ["nx", "normalx", "normal_x"]:
                has_normals = True
                break
            i += 1

    return has_normals

def filter(input_point_cloud, output_point_cloud, standard_deviation=2.5, meank=16, sample_radius=0, verbose=False):
    """
    Filters a point cloud
    """
    if not os.path.exists(input_point_cloud):
        log.ODM_ERROR("{} does not exist. The program will now exit.".format(input_point_cloud))
        sys.exit(1)

    if (standard_deviation <= 0 or meank <= 0) and sample_radius <= 0:
        log.ODM_INFO("Skipping point cloud filtering")
        # if using the option `--pc-filter 0`, we need copy input_point_cloud
        shutil.copy(input_point_cloud, output_point_cloud)
        return

    filters = []

    if sample_radius > 0:
        log.ODM_INFO("Sampling points around a %sm radius" % sample_radius)
        filters.append('sample')

    if standard_deviation > 0 and meank > 0:
        log.ODM_INFO("Filtering point cloud (statistical, meanK {}, standard deviation {})".format(meank, standard_deviation))
        filters.append('outlier')

    if len(filters) > 0:
        filters.append('range')

    dims = "x=float,y=float,z=float,"
    if ply_has_normals(input_point_cloud):
        dims += "nx=float,ny=float,nz=float,"
    dims += "red=uchar,blue=uchar,green=uchar"

    filterArgs = {
      'inputFile': input_point_cloud,
      'outputFile': output_point_cloud,
      'stages': " ".join(filters),
      'dims': dims
    }

    cmd = ("pdal translate -i \"{inputFile}\" "
            "-o \"{outputFile}\" "
            "{stages} "
            "--writers.ply.sized_types=false "
            "--writers.ply.storage_mode='little endian' "
            "--writers.ply.dims=\"{dims}\" "
            "").format(**filterArgs)

    if 'sample' in filters:
        cmd += "--filters.sample.radius={} ".format(sample_radius)
    
    if 'outlier' in filters:
        cmd += ("--filters.outlier.method='statistical' "
               "--filters.outlier.mean_k={} "
               "--filters.outlier.multiplier={} ").format(meank, standard_deviation)  
    
    if 'range' in filters:
        # Remove outliers
        cmd += "--filters.range.limits='Classification![7:7]' "

    system.run(cmd)

    if not os.path.exists(output_point_cloud):
        log.ODM_WARNING("{} not found, filtering has failed.".format(output_point_cloud))

def get_extent(input_point_cloud):
    fd, json_file = tempfile.mkstemp(suffix='.json')
    os.close(fd)
    
    # Get point cloud extent
    fallback = False

    # We know PLY files do not have --summary support
    if input_point_cloud.lower().endswith(".ply"):
        fallback = True
        run('pdal info {0} > {1}'.format(input_point_cloud, json_file))

    try:
        if not fallback:
            run('pdal info --summary {0} > {1}'.format(input_point_cloud, json_file))
    except:
        fallback = True
        run('pdal info {0} > {1}'.format(input_point_cloud, json_file))

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

    if rerun and io.file_exists(output_file):
        log.ODM_WARNING("Removing previous point cloud: %s" % output_file)
        os.remove(output_file)

    kwargs = {
        'all_inputs': " ".join(map(quote, input_point_cloud_files)),
        'output': output_file
    }

    system.run('lasmerge -i {all_inputs} -o "{output}"'.format(**kwargs))
   

def post_point_cloud_steps(args, tree):
    # XYZ point cloud output
    if args.pc_csv:
        log.ODM_INFO("Creating geo-referenced CSV file (XYZ format)")
        
        system.run("pdal translate -i \"{}\" "
            "-o \"{}\" "
            "--writers.text.format=csv "
            "--writers.text.order=\"X,Y,Z\" "
            "--writers.text.keep_unspecified=false ".format(
                tree.odm_georeferencing_model_laz,
                tree.odm_georeferencing_xyz_file))

    # LAS point cloud output
    if args.pc_las:
        log.ODM_INFO("Creating geo-referenced LAS file")
        
        system.run("pdal translate -i \"{}\" "
            "-o \"{}\" ".format(
                tree.odm_georeferencing_model_laz,
                tree.odm_georeferencing_model_las))

    # EPT point cloud output
    if args.pc_ept:
        log.ODM_INFO("Creating geo-referenced Entwine Point Tile output")
        entwine.build([tree.odm_georeferencing_model_laz], tree.entwine_pointcloud, max_concurrency=args.max_concurrency, rerun=False)
