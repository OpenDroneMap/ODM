#!/usr/bin/python3

# Basic check
import sys
if sys.version_info.major < 3:
    print("Ups! ODM needs to run with Python 3. It seems you launched it with Python 2. Try using: python3 run.py ... ")
    sys.exit(1)

import os
from opendm import log
from opendm import config
from opendm import system
from opendm import io
from opendm.progress import progressbc
from opendm.utils import double_quote, get_processing_results_paths
from opendm.loghelpers import args_to_dict

from stages.odm_app import ODMApp

if __name__ == '__main__':
    args = config.config()

    log.ODM_INFO('Initializing ODM - %s' % system.now())

    # Print args
    args_dict = args_to_dict(args)
    log.ODM_INFO('==============')
    for k in args_dict.keys():
        log.ODM_INFO('%s: %s' % (k, args_dict[k]))
    log.ODM_INFO('==============')

    progressbc.set_project_name(args.name)

    # Add project dir if doesn't exist
    args.project_path = os.path.join(args.project_path, args.name)
    if not io.dir_exists(args.project_path):
        log.ODM_WARNING('Directory %s does not exist. Creating it now.' % args.name)
        system.mkdir_p(os.path.abspath(args.project_path))

    # If user asks to rerun everything, delete all of the existing progress directories.
    if args.rerun_all:
        log.ODM_INFO("Rerun all -- Removing old data")
        os.system("rm -rf " + 
                    " ".join([double_quote(os.path.join(args.project_path, p)) for p in get_processing_results_paths()] + [
                        double_quote(os.path.join(args.project_path, "odm_meshing")),
                        double_quote(os.path.join(args.project_path, "opensfm")),
                        double_quote(os.path.join(args.project_path, "odm_texturing_25d")),
                        double_quote(os.path.join(args.project_path, "odm_filterpoints")),
                        double_quote(os.path.join(args.project_path, "submodels")),
                    ]))

    app = ODMApp(args)
    retcode = app.execute()
    
    # Do not show ASCII art for local submodels runs
    if retcode == 0 and not "submodels/submodel_" in args.project_path:
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
        log.ODM_INFO('ODM app finished - %s' % system.now())
    else:
        exit(retcode)