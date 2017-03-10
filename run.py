#!/usr/bin/python

from opendm import log
from opendm import config
from opendm import system
from opendm import io
from opendm import context

import sys
import ecto
import os

from scripts.odm_app import ODMApp

with open(io.join_paths(context.root_path, 'VERSION')) as version_file:
    __version__ = version_file.read().strip()


def usage():
    log.ODM_ERROR('You must specify a project name:')
    log.ODM_ERROR('USAGE: %s [project name]' % sys.argv[0])
    log.ODM_ERROR('OpenDroneMap app finished - %s' % system.now())
    sys.exit(0)


if __name__ == '__main__':

    args = config.config()

    if args.version:
        log.ODM_INFO(__version__)
        sys.exit(0)

    log.ODM_INFO('Initializing OpenDroneMap app - %s' % system.now())

    # Force to provide the images path
    if args.project_path is None:
        usage()
    args.project_path = io.join_paths(args.project_path, args.name)
    if not io.dir_exists(args.project_path):
        log.ODM_WARNING('Directory %s does not exist. Creating it now.' % args.name)
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
