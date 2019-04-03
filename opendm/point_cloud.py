import os, sys, shutil
from opendm import system
from opendm import log
from opendm import context

def filter(inputPointCloud, outputPointCloud, standard_deviation=2.5, meank=16, verbose=False):
    """
    Filters a point cloud
    """
    if standard_deviation <= 0 or meank <= 0:
        log.ODM_INFO("Skipping point cloud filtering")
        return

    log.ODM_INFO("Filtering point cloud (statistical, meanK {}, standard deviation {})".format(meank, standard_deviation))

    if not os.path.exists(inputPointCloud):
        log.ODM_ERROR("{} does not exist, cannot filter point cloud. The program will now exit.".format(inputPointCloud))
        sys.exit(1)

    filter_program = os.path.join(context.odm_modules_path, 'odm_filterpoints')
    if not os.path.exists(filter_program):
        log.ODM_WARNING("{} program not found. Will skip filtering, but this installation should be fixed.")
        shutil.copy(inputPointCloud, outputPointCloud)
        return

    filterArgs = {
      'bin': filter_program,
      'inputFile': inputPointCloud,
      'outputFile': outputPointCloud,
      'sd': standard_deviation,
      'meank': meank,
      'verbose': '--verbose' if verbose else '',
    }

    system.run('{bin} -inputFile {inputFile} '
         '-outputFile {outputFile} '
         '-sd {sd} '
         '-meank {meank} {verbose} '.format(**filterArgs))

    # Remove input file, swap temp file
    if not os.path.exists(outputPointCloud):
        log.ODM_WARNING("{} not found, filtering has failed.".format(outputPointCloud))
