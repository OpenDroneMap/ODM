import os, sys, shutil, tempfile, json, math
from opendm import system
from opendm import log
from opendm import context
from opendm.system import run
from opendm import entwine
from opendm import io
from opendm.concurrency import parallel_map
from pipes import quote

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
        'has_views': has_views
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
                "--writers.ply.storage_mode='little endian' ")
    if dims is not None:
        cmd += '--writers.ply.dims="%s"' % dims
    system.run(cmd)

    return [os.path.join(outdir, f) for f in os.listdir(outdir)]


def filter(input_point_cloud, output_point_cloud, standard_deviation=2.5, meank=16, sample_radius=0, verbose=False, max_concurrency=1):
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
        log.ODM_INFO("Filtering {} (statistical, meanK {}, standard deviation {})".format(input_point_cloud, meank, standard_deviation))
        filters.append('outlier')

    if len(filters) > 0:
        filters.append('range')

    info = ply_info(input_point_cloud)
    dims = "x=float,y=float,z=float,"
    if info['has_normals']:
        dims += "nx=float,ny=float,nz=float,"
    dims += "red=uchar,blue=uchar,green=uchar"
    if info['has_views']:
        dims += ",views=uchar"

    if info['vertex_count'] == 0:
        log.ODM_ERROR("Cannot read vertex count for {}".format(input_point_cloud))
        sys.exit(1)

    # Do we need to split this?
    VERTEX_THRESHOLD = 250000
    should_split = max_concurrency > 1 and info['vertex_count'] > VERTEX_THRESHOLD*2

    if should_split:
        partsdir = os.path.join(os.path.dirname(output_point_cloud), "parts")
        if os.path.exists(partsdir):
            log.ODM_WARNING("Removing existing directory %s" % partsdir)
            shutil.rmtree(partsdir)

        point_cloud_submodels = split(input_point_cloud, partsdir, "part.ply", capacity=VERTEX_THRESHOLD, dims=dims)

        def run_filter(pcs):
            # Recurse
            filter(pcs['path'], io.related_file_path(pcs['path'], postfix="_filtered"), 
                        standard_deviation=standard_deviation, 
                        meank=meank, 
                        sample_radius=sample_radius, 
                        verbose=verbose,
                        max_concurrency=1)
        # Filter
        parallel_map(run_filter, [{'path': p} for p in point_cloud_submodels], max_concurrency)

        # Merge
        log.ODM_INFO("Merging %s point cloud chunks to %s" % (len(point_cloud_submodels), output_point_cloud))
        filtered_pcs = [io.related_file_path(pcs, postfix="_filtered") for pcs in point_cloud_submodels]
        #merge_ply(filtered_pcs, output_point_cloud, dims)
        fast_merge_ply(filtered_pcs, output_point_cloud)

        if os.path.exists(partsdir):
            shutil.rmtree(partsdir)
    else:
        # Process point cloud (or a point cloud submodel) in a single step
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

    if io.file_exists(output_file):
        log.ODM_WARNING("Removing previous point cloud: %s" % output_file)
        os.remove(output_file)

    kwargs = {
        'all_inputs': " ".join(map(quote, input_point_cloud_files)),
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
        ' '.join(map(quote, input_point_cloud_files + [output_file])),
    ]

    system.run(' '.join(cmd))

def post_point_cloud_steps(args, tree, rerun=False):
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
