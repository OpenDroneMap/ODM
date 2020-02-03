import os, sys, shutil, tempfile, json
from opendm import system
from opendm import log
from opendm import context
from opendm.system import run
from opendm import entwine
from opendm import io
from pipes import quote

def filter(input_point_cloud, output_point_cloud, standard_deviation=2.5, meank=16, confidence=None, sample_radius=0, verbose=False):
    """
    Filters a point cloud
    """
    if (standard_deviation <= 0 or meank <= 0) and sample_radius <= 0:
        log.ODM_INFO("Skipping point cloud filtering")
        return

    if standard_deviation > 0 and meank > 0:
        log.ODM_INFO("Filtering point cloud (statistical, meanK {}, standard deviation {})".format(meank, standard_deviation))
    
    if confidence:
        log.ODM_INFO("Keeping only points with > %s confidence" % confidence)

    if sample_radius > 0:
        log.ODM_INFO("Sampling points around a %sm radius" % sample_radius)

    if not os.path.exists(input_point_cloud):
        log.ODM_ERROR("{} does not exist, cannot filter point cloud. The program will now exit.".format(input_point_cloud))
        sys.exit(1)

    filter_program = os.path.join(context.odm_modules_path, 'odm_filterpoints')
    if not os.path.exists(filter_program):
        log.ODM_WARNING("{} program not found. Will skip filtering, but this installation should be fixed.")
        shutil.copy(input_point_cloud, output_point_cloud)
        return

    filterArgs = {
      'bin': filter_program,
      'inputFile': input_point_cloud,
      'outputFile': output_point_cloud,
      'sd': standard_deviation,
      'meank': meank,
      'verbose': '-verbose' if verbose else '',
      'confidence': '-confidence %s' % confidence if confidence else '',
      'sample': max(0, sample_radius)
    }

    system.run('{bin} -inputFile {inputFile} '
         '-outputFile {outputFile} '
         '-sd {sd} '
         '-meank {meank} '
         '-sample {sample} '
         '{confidence} {verbose} '.format(**filterArgs))

    # Remove input file, swap temp file
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
