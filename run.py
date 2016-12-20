#!/usr/bin/python

from opendm import log
from opendm import config
from opendm import system
from opendm import io

import sys
import ecto
import os

from scripts.odm_app import ODMApp


def usage():
    log.ODM_ERROR('USAGE: %s --project-path [project_path]' % sys.argv[0])
    log.ODM_ERROR('OpenDroneMap app finished - %s' % system.now())
    sys.exit(0)


if __name__ == '__main__':

    log.ODM_INFO('Initializing OpenDroneMap app - %s' % system.now())

    args = config.config()

    # Force to provide the images path
    if args.project_path is None:
        usage()
    elif not io.dir_exists(args.project_path):
        log.ODM_WARNING('Directory %s does not exist. Creating it now.' % args.project_path)
        system.mkdir_p(os.path.abspath(args.project_path))

    #If user asks to rerun everything, delete all of the existing progress directories.
    # TODO: Move this somewhere it's not hard-coded
    if args.rerun_all:
        os.system("rm -rf "
                  + args.project_path + "images_resize/ "
                  + args.project_path + "odm_georeferencing/ "
                  + args.project_path + "odm_meshing/ "
                  + args.project_path + "odm_orthophoto/ "
                  + args.project_path + "odm_texturing/ "
                  + args.project_path + "opensfm/ "
                  + args.project_path + "pmvs/")

    # create an instance of my App BlackBox
    # internally configure all tasks
    app = ODMApp(args=args)

    # create a plasm that only contains the BlackBox
    plasm = ecto.Plasm()
    plasm.insert(app)

    # execute the plasm
    plasm.execute(niter=1)

    log.ODM_INFO('OpenDroneMap app finished - %s' % system.now())
