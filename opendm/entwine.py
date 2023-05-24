import os
import sys
import shutil
from opendm.utils import double_quote
from opendm import io
from opendm import log
from opendm import system
from opendm import concurrency


def build(input_point_cloud_files, output_path, max_concurrency=8, rerun=False):
    num_files = len(input_point_cloud_files)
    if num_files == 0:
        log.ODM_WARNING("No input point cloud files to process")
        return

    tmpdir = io.related_file_path(output_path, postfix="-tmp")

    def dir_cleanup():
        if io.dir_exists(output_path):
            log.ODM_WARNING("Removing previous EPT directory: %s" % output_path)
            shutil.rmtree(output_path)
    
        if io.dir_exists(tmpdir):
            log.ODM_WARNING("Removing previous EPT temp directory: %s" % tmpdir)
            shutil.rmtree(tmpdir)
    
    if rerun:
        dir_cleanup()

    # Attempt with entwine (faster, more memory hungry)
    try:
        build_entwine(input_point_cloud_files, tmpdir, output_path, max_concurrency=max_concurrency)
    except Exception as e:
        log.ODM_WARNING("Cannot build EPT using entwine (%s), attempting with untwine..." % str(e))
        dir_cleanup()
        build_untwine(input_point_cloud_files, tmpdir, output_path, max_concurrency=max_concurrency)

    if os.path.exists(tmpdir):
        shutil.rmtree(tmpdir)


def build_entwine(input_point_cloud_files, tmpdir, output_path, max_concurrency=8, reproject=None):
    kwargs = {
        'threads': max_concurrency,
        'tmpdir': tmpdir,
        'all_inputs': "-i " + " ".join(map(double_quote, input_point_cloud_files)),
        'outputdir': output_path,
        'reproject': (" -r %s " % reproject) if reproject is not None else "" 
    }

    system.run('entwine build --threads {threads} --tmp "{tmpdir}" {all_inputs} -o "{outputdir}" {reproject}'.format(**kwargs))

def build_untwine(input_point_cloud_files, tmpdir, output_path, max_concurrency=8, rerun=False):
    kwargs = {
        # 'threads': max_concurrency,
        'tmpdir': tmpdir,
        'files': "--files " + " ".join(map(double_quote, input_point_cloud_files)),
        'outputdir': output_path
    }

    # Run untwine
    system.run('untwine --temp_dir "{tmpdir}" {files} --output_dir "{outputdir}"'.format(**kwargs))

def build_copc(input_point_cloud_files, output_file, convert_rgb_8_to_16=False, embed_gcp=None):
    if len(input_point_cloud_files) == 0:
        logger.ODM_WARNING("Cannot build COPC, no input files")
        return

    base_path, ext = os.path.splitext(output_file)
    
    tmpdir = io.related_file_path(base_path, postfix="-tmp")
    if os.path.exists(tmpdir):
        log.ODM_WARNING("Removing previous directory %s" % tmpdir)
        shutil.rmtree(tmpdir)
    cleanup = [tmpdir]

    do_embed_gcp = embed_gcp is not None and os.path.isfile(embed_gcp)

    if convert_rgb_8_to_16 or do_embed_gcp:
        tmpdir_2 = io.related_file_path(base_path, postfix="-tmp2")
        if os.path.exists(tmpdir_2):
            log.ODM_WARNING("Removing previous directory %s" % tmpdir_2)
            shutil.rmtree(tmpdir_2)
        os.makedirs(tmpdir_2, exist_ok=True)
        cleanup.append(tmpdir_2)

        converted = []
        ok = True
        for f in input_point_cloud_files:
            # Convert 8bit RGB to 16bit RGB (per COPC spec)
            base = os.path.basename(f)
            filename, ext = os.path.splitext(base)
            out_intermediate = os.path.join(tmpdir_2, "%s_inter%s" % (filename, ext))
            try:
                stages = []
                params = []

                if convert_rgb_8_to_16:
                    stages.append("assign")
                    params.append('--filters.assign.value="Red = Red / 255 * 65535"')
                    params.append('--filters.assign.value="Green = Green / 255 * 65535"')
                    params.append('--filters.assign.value="Blue = Blue / 255 * 65535"')
                
                # if do_embed_gcp:
                #     params.append('--writers.las.vlrs="{\\\"filename\\\": \\\"%s\\\", \\\"user_id\\\": \\\"ODM\\\", \\\"record_id\\\": \\\"1\\\", \\\"description\\\": \\\"Ground Control Points (GML)\\\"}"' % gcp_geojson_export_file.replace(os.sep, "/")
                #     ]")

                system.run('pdal translate -i "{input}" -o "{output}" {stages} {params}'.format(input=f, output=out_intermediate, stages=" ".join(stages), params=" ".join(params)))
                
                converted.append(out_intermediate)
            except Exception as e:
                log.ODM_WARNING("Cannot execute pdal translate: %s" % str(e))
                ok = False
                break
        if ok:
            input_point_cloud_files = converted
        
    kwargs = {
        'tmpdir': tmpdir,
        'files': "--files " + " ".join(map(double_quote, input_point_cloud_files)),
        'output': output_file
    }

    # Run untwine
    system.run('untwine --temp_dir "{tmpdir}" {files} -o "{output}" --single_file'.format(**kwargs))

    for d in cleanup:
        if os.path.exists(d):
            shutil.rmtree(d)