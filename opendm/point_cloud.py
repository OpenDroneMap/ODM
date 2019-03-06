import os, sys
from opendm import system
from opendm import log
from opendm import context

def filter(pointCloudPath, standard_deviation=2.5, meank=16, verbose=False):
    """
    Filters a point cloud in place (it will replace the input file with the filtered result).
    """
    if standard_deviation <= 0 or meank <= 0:
        log.ODM_INFO("Skipping point cloud filtering")
        return

    log.ODM_INFO("Filtering point cloud (statistical, meanK {}, standard deviation {})".format(meank, standard_deviation))

    if not os.path.exists(pointCloudPath):
        log.ODM_ERROR("{} does not exist, cannot filter point cloud. The program will now exit.".format(pointCloudPath))
        sys.exit(1)

    filter_program = os.path.join(context.odm_modules_path, 'odm_filterpoints')
    if not os.path.exists(filter_program):
        log.ODM_WARNING("{} program not found. Will skip filtering, but this installation should be fixed.")
        return

    pc_path, pc_filename = os.path.split(pointCloudPath)
    # pc_path = path/to
    # pc_filename = pointcloud.ply

    basename, ext = os.path.splitext(pc_filename)
    # basename = pointcloud
    # ext = .ply

    tmpPointCloud = os.path.join(pc_path, "{}.tmp{}".format(basename, ext))

    filterArgs = {
      'bin': filter_program,
      'inputFile': pointCloudPath,
      'outputFile': tmpPointCloud,
      'sd': standard_deviation,
      'meank': meank,
      'verbose': '--verbose' if verbose else '',
    }

    system.run('{bin} -inputFile {inputFile} '
         '-outputFile {outputFile} '
         '-sd {sd} '
         '-meank {meank} {verbose} '.format(**filterArgs))

    # Remove input file, swap temp file
    if os.path.exists(tmpPointCloud):
        os.remove(pointCloudPath)
        os.rename(tmpPointCloud, pointCloudPath)
    else:
        log.ODM_WARNING("{} not found, filtering has failed.".format(tmpPointCloud))
