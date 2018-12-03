#!/usr/bin/python

from opendm import log
from opendm import config
from opendm import system
from opendm import io

import ecto
import os

from scripts.odm_app import ODMApp

if __name__ == '__main__':

    args = config.config()

    log.ODM_INFO('Initializing OpenDroneMap app - %s' % system.now())

    # Add project dir if doesn't exist
    args.project_path = io.join_paths(args.project_path, args.name)
    if not io.dir_exists(args.project_path):
        log.ODM_WARNING('Directory %s does not exist. Creating it now.' % args.name)
        system.mkdir_p(os.path.abspath(args.project_path))

    # If user asks to rerun everything, delete all of the existing progress directories.
    # TODO: Move this somewhere it's not hard-coded
    if args.rerun_all:
        log.ODM_DEBUG("Rerun all -- Removing old data")
        os.system("rm -rf "
                  + args.project_path + "/images_resize "
                  + args.project_path + "/odm_georeferencing "
                  + args.project_path + "/odm_meshing "
                  + args.project_path + "/odm_orthophoto "
                  + args.project_path + "/odm_texturing "
                  + args.project_path + "/opensfm "
                  + args.project_path + "/smvs")

    # create an instance of my App BlackBox
    # internally configure all tasks
    app = ODMApp(args=args)

    # create a plasm that only contains the BlackBox
    plasm = ecto.Plasm()
    plasm.insert(app)

    # execute the plasm
    plasm.execute(niter=1)
    
    log.ODM_INFO('MMMMMMMMMMMNNNMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMNNNMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMdo:..---../sNMMMMMMMMMMMMMMMMMMMMMMMMMMNs/..---..:odMMMMMM')
    log.ODM_INFO('MMMMy-.odNMMMMMNy/`/mMMMMMMMMMMMMMMMMMMMMMMm/`/hNMMMMMNdo.-yMMMM')
    log.ODM_INFO('MMN/`sMMMMMMMMMNNMm/`yMMMMMMMMMMMMMMMMMMMMy`/mMNNMMMMMMMMNs`/MMM')
    log.ODM_INFO('MM/ hMMMMMMMMNs.+MMM/ dMMMMMMMMMMMMMMMMMMh +MMM+.sNMMMMMMMMh +MM')
    log.ODM_INFO('MN /MMMMMMNo/./mMMMMN :MMMMMMMMMMMMMMMMMM: NMMMMm/./oNMMMMMM: NM')
    log.ODM_INFO('Mm +MMMMMN+ `/MMMMMMM`-MMMMMMMMMMMMMMMMMM-`MMMMMMM:` oNMMMMM+ mM')
    log.ODM_INFO('MM..NMMNs./mNMMMMMMMy sMMMMMMMMMMMMMMMMMMo hMMMMMMMNm/.sNMMN`-MM')
    log.ODM_INFO('MMd`:mMNomMMMMMMMMMy`:MMMMMMMNmmmmNMMMMMMN:`hMMMMMMMMMdoNMm-`dMM')
    log.ODM_INFO('MMMm:.omMMMMMMMMNh/  sdmmho/.`..`-``-/sddh+  /hNMMMMMMMMdo.:mMMM')
    log.ODM_INFO('MMMMMd+--/osss+:-:/`  ```:- .ym+ hmo``:-`   `+:-:ossso/-:+dMMMMM')
    log.ODM_INFO('MMMMMMMNmhysosydmNMo   /ds`/NMM+ hMMd..dh.  sMNmdysosyhmNMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMMMMMs .:-:``hmmN+ yNmds -:.:`-NMMMMMMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMMMMN.-mNm- //:::. -:://: +mMd`-NMMMMMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMMMM+ dMMN -MMNNN+ yNNNMN :MMMs sMMMMMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMMMM`.mmmy /mmmmm/ smmmmm``mmmh :MMMMMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMMMM``:::- ./////. -:::::` :::: -MMMMMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMMMM:`mNNd /NNNNN+ hNNNNN .NNNy +MMMMMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMMMMd`/MMM.`ys+//. -/+oso +MMN.`mMMMMMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMMMMMy /o:- `oyhd/ shys+ `-:s-`hMMMMMMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMNmdhhhdmNMMM`  +d+ sMMM+ hMMN:`hh-  sMMNmdhhhdmNMMMMMMMM')
    log.ODM_INFO('MMMMMms:::/++//::+ho    .+- /dM+ hNh- +/`   -h+:://++/::/smMMMMM')
    log.ODM_INFO('MMMN+./hmMMMMMMNds-  ./oso:.``:. :-``.:os+-  -sdNMMMMMMmy:.oNMMM')
    log.ODM_INFO('MMm-.hMNhNMMMMMMMMNo`/MMMMMNdhyyyyhhdNMMMM+`oNMMMMMMMMNhNMh.-mMM')
    log.ODM_INFO('MM:`mMMN/-sNNMMMMMMMo yMMMMMMMMMMMMMMMMMMy sMMMMMMMNNs-/NMMm`:MM')
    log.ODM_INFO('Mm /MMMMMd/.-oMMMMMMN :MMMMMMMMMMMMMMMMMM-`MMMMMMMo-./dMMMMM/ NM')
    log.ODM_INFO('Mm /MMMMMMm:-`sNMMMMN :MMMMMMMMMMMMMMMMMM-`MMMMMNs`-/NMMMMMM/ NM')
    log.ODM_INFO('MM:`mMMMMMMMMd/-sMMMo yMMMMMMMMMMMMMMMMMMy sMMMs-/dMMMMMMMMd`:MM')
    log.ODM_INFO('MMm-.hMMMMMMMMMdhMNo`+MMMMMMMMMMMMMMMMMMMM+`oNMhdMMMMMMMMMh.-mMM')
    log.ODM_INFO('MMMNo./hmNMMMMMNms--yMMMMMMMMMMMMMMMMMMMMMMy--smNMMMMMNmy/.oNMMM')
    log.ODM_INFO('MMMMMms:-:/+++/:-+hMMMMMMMMMMMMMMMMMMMMMMMMMNh+-:/+++/:-:smMMMMM')
    log.ODM_INFO('MMMMMMMMNdhhyhdmMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMmdhyhhmNMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMNNNNNMMMMMMNNNNNNMMMMMMMMNNMMMMMMMNNMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMh/-...-+dMMMm......:+hMMMMs../MMMMMo..sMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMM/  /yhy-  sMMm  -hhy/  :NMM+   oMMMy   /MMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMy  /MMMMN`  NMm  /MMMMo  +MM: .` yMd``` :MMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMM+  sMMMMM:  hMm  /MMMMd  -MM- /s `h.`d- -MMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMs  +MMMMM.  mMm  /MMMMy  /MM. +M/   yM: `MMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMN-  smNm/  +MMm  :NNdo` .mMM` oMM+/yMM/  MMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMNo-    `:yMMMm      `:sNMMM` sMMMMMMM+  NMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMNmmNMMMMMMMNmmmmNMMMMMMMNNMMMMMMMMMNNMMMMMMMMMMMM')
    log.ODM_INFO('OpenDroneMap app finished - %s' % system.now())
