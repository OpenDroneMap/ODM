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
from opendm.utils import get_processing_results_paths, rm_r
from opendm.arghelpers import args_to_dict, save_opts, find_rerun_stage

from stages.odm_app import ODMApp

def odm_version():
    try:
        with open("VERSION") as f:
            return f.read().split("\n")[0].strip()
    except:
        return "?"

if __name__ == '__main__':
    args = config.config()

    log.ODM_INFO('Initializing ODM %s - %s' % (odm_version(), system.now()))

    progressbc.set_project_name(args.name)
    args.project_path = os.path.join(args.project_path, args.name)

    if not io.dir_exists(args.project_path):
        log.ODM_ERROR('Directory %s does not exist.' % args.name)
        exit(1)

    opts_json = os.path.join(args.project_path, "options.json")
    auto_rerun_stage, opts_diff = find_rerun_stage(opts_json, args, config.rerun_stages, config.processopts)
    if auto_rerun_stage is not None and len(auto_rerun_stage) > 0:
        log.ODM_INFO("Rerunning from: %s" % auto_rerun_stage[0])
        args.rerun_from = auto_rerun_stage

    # Print args
    args_dict = args_to_dict(args)
    log.ODM_INFO('==============')
    for k in args_dict.keys():
        log.ODM_INFO('%s: %s%s' % (k, args_dict[k], ' [changed]' if k in opts_diff else ''))
    log.ODM_INFO('==============')
    

    # If user asks to rerun everything, delete all of the existing progress directories.
    if args.rerun_all:
        log.ODM_INFO("Rerun all -- Removing old data")
        for d in [os.path.join(args.project_path, p) for p in get_processing_results_paths()] + [
                  os.path.join(args.project_path, "odm_meshing"),
                  os.path.join(args.project_path, "opensfm"),
                  os.path.join(args.project_path, "odm_texturing_25d"),
                  os.path.join(args.project_path, "odm_filterpoints"),
                  os.path.join(args.project_path, "submodels")]:
            rm_r(d)

    app = ODMApp(args)
    retcode = app.execute()

    if retcode == 0:
        save_opts(opts_json, args)
    
    # Do not show ASCII art for local submodels runs
    if retcode == 0 and not "submodels" in args.project_path:
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