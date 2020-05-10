#!/usr/bin/python

from opendm import log
from opendm import config
from opendm import system
from opendm import io
from opendm.progress import progressbc

import os
from pipes import quote

from stages.odm_app import ODMApp

if __name__ == '__main__':
    args = config.config()

    log.ODM_INFO('Initializing ODM - %s' % system.now())

    # Print args
    args_dict = vars(args)
    log.ODM_INFO('==============')
    for k in sorted(args_dict.keys()):
        # Skip _is_set keys
        if k.endswith("_is_set"):
            continue

        # Don't leak token
        if k == 'sm_cluster' and args_dict[k] is not None:
            log.ODM_INFO('%s: True' % k)
        else:
            log.ODM_INFO('%s: %s' % (k, args_dict[k]))
    log.ODM_INFO('==============')

    progressbc.set_project_name(args.name)

    # Add project dir if doesn't exist
    args.project_path = io.join_paths(args.project_path, args.name)
    if not io.dir_exists(args.project_path):
        log.ODM_WARNING('Directory %s does not exist. Creating it now.' % args.name)
        system.mkdir_p(os.path.abspath(args.project_path))

    # If user asks to rerun everything, delete all of the existing progress directories.
    if args.rerun_all:
        log.ODM_INFO("Rerun all -- Removing old data")
        os.system("rm -rf " + 
                    " ".join([
                        quote(os.path.join(args.project_path, "odm_georeferencing")),
                        quote(os.path.join(args.project_path, "odm_georeferencing_25d")),
                        quote(os.path.join(args.project_path, "odm_meshing")),
                        quote(os.path.join(args.project_path, "odm_orthophoto")),
                        quote(os.path.join(args.project_path, "odm_texturing")),
                        quote(os.path.join(args.project_path, "opensfm")),
                        quote(os.path.join(args.project_path, "odm_filterpoints")),
                        quote(os.path.join(args.project_path, "odm_texturing_25d")),
                        quote(os.path.join(args.project_path, "mve")),
                        quote(os.path.join(args.project_path, "entwine_pointcloud")),
                        quote(os.path.join(args.project_path, "submodels")),
                    ]))

    app = ODMApp(args)
    app.execute()
    
    # Do not show ASCII art for local submodels runs
    if not "submodels/submodel_" in args.project_path:
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
