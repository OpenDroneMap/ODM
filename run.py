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
                  + args.project_path + "/mve")

    # create an instance of my App BlackBox
    # internally configure all tasks
    app = ODMApp(args=args)

    # create a plasm that only contains the BlackBox
    plasm = ecto.Plasm()
    plasm.insert(app)

    # execute the plasm
    plasm.execute(niter=1)
    log.ODM_INFO('MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMdo/-.`.-+yNMMMMMMMMMMMMMMMMMMMMMMMMMMMMmy/-.`.-/odMMMMMMM')
    log.ODM_INFO('MMMMMs.`/shmmdho- /dMMMMMMMMMMMMMMMMMMMMMMMMd: -ohdmmhs/`.yMMMMM')
    log.ODM_INFO('MMMm-`sNMMMMMMMMMd/ oMMMMMMMMMMMMMMMMMMMMMM+ /mMMMMMMMMMNo`-mMMM')
    log.ODM_INFO('MMm..mMMMMMMMMMmhMMs /MMMMMMMMMMMMMMMMMMMM/ yMMhmMMMMMMMMMm..NMM')
    log.ODM_INFO('MM: mMMMMMMMMNo`oMMMo hMMMMMMMMMMMMMMMMMMy oMMMo`oNMMMMMMMMd /MM')
    log.ODM_INFO('MN /MMMMMMMs+.:dMMMMN /MMMMMMMMMMMMMMMMMM: NMMMMd:.+sMMMMMMM: NM')
    log.ODM_INFO('Md +MMMMMMy  .MMMMMMM`-MMMMMMMMMMMMMMMMMM-`MMMMMMM.  yMMMMMM+ mM')
    log.ODM_INFO('MN :MMMMN+`+ymMMMMMMm /MMMMMMMMMMMMMMMMMM/ mMMMMMMmy+`+NMMMM-`NM')
    log.ODM_INFO('MM+ hMMm.:dMMMMMMMMM/ dMMMMMMMMMMMMMMMMMMd /MMMMMMMMMd:.mMMh +MM')
    log.ODM_INFO('MMN-`hMMmMMMMMMMMMN+ sMMMMMMmdhyyyhdNMMMMMo +NMMMMMMMMMmMMy`-NMM')
    log.ODM_INFO('MMMN/`/dMMMMMMMMNy-  ohddy+.``.. .```-+hhy/  -yNMMMMMMMMd/`+NMMM')
    log.ODM_INFO('MMMMMd/.-+syyyo/.-:   ```.- `sN+ hm+ `:`    `/-./oyyys+../dMMMMM')
    log.ODM_INFO('MMMMMMMmho//:/+sdNM+   -yh`.mMM+ hMMy`:mo`  oMNds+/://ohNMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMMMMMy   /h.`mMMM+ hMMMs +y-  oMMMMMMMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMMMMM/ oh/. `:+os: +so+:  -+h/ yMMMMMMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMMMMd /MMM.`hso//. -/+oys oMMN.`NMMMMMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMMMM/ mMMN :MMMMMo hMMMMN -MMMy oMMMMMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMMMM`.NNNh +NNNNN+ yNNNNN.`NNNd :MMMMMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMMMM``---. `-----` .-----  .... -MMMMMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMMMM-`dmmy /mmmmm/ ymmmmd``dddy /MMMMMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMMMMo hMMN -MMNmm/ ymmNMm :MMM+ yMMMMMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMMMMN..NMm. :----` .----- +NMd`-MMMMMMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMMMMMh ::.- .hdmm/ ymdds `-./..mMMMMMMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMmddddmNMMMM`  om/ sMMM+ hMMN:`hd-  yMMMNmddddmMMMMMMMMM')
    log.ODM_INFO('MMMMMMdo--------:ymy    :s-`sNM+ hMm/ +s.   /ms:--------odMMMMMM')
    log.ODM_INFO('MMMMd:./ydNNMNmdo-.`  `.-..  -s/ so.  ..-.   .-sdmNMNNdy:./dMMMM')
    log.ODM_INFO('MMMy`:dMMMMMMMMMMNy` +mNNNho/-.....-/sdNNmo .yNMMMMMMMMMMd:`yMMM')
    log.ODM_INFO('MMh`/NMm/hMMMMMMMMMd`-NMMMMMMMNmmNNMMMMMMN-.dMMMMMMMMMh/mMN/`hMM')
    log.ODM_INFO('MM.`NMMNo.+mNMMMMMMMy sMMMMMMMMMMMMMMMMMMo yMMMMMMMNm/.sNMMN`-MM')
    log.ODM_INFO('Mm +MMMMMm:`-oMMMMMMM :MMMMMMMMMMMMMMMMMM-`MMMMMMMo-`:mMMMMM/ NM')
    log.ODM_INFO('Mm +MMMMMMm.``hMMMMMM`-MMMMMMMMMMMMMMMMMM-`MMMMMMh``.mMMMMMM+ mM')
    log.ODM_INFO('MM`.MMMMMMMNms./mMMMh oMMMMMMMMMMMMMMMMMM+ dMMMm/.smNMMMMMMM..MM')
    log.ODM_INFO('MMs oMMMMMMMMMm//MMN-.NMMMMMMMMMMMMMMMMMMm`-NMM//mMMMMMMMMMo yMM')
    log.ODM_INFO('MMM+`+NMMMMMMMMMMMd-.dMMMMMMMMMMMMMMMMMMMMh`-dMMMMMMMMMMMN+`oMMM')
    log.ODM_INFO('MMMMy..omNMMMMNNh/`:mMMMMMMMMMMMMMMMMMMMMMMm:`/hNNMMMMNmo..yMMMM')
    log.ODM_INFO('MMMMMNy:.-:///:..+dMMMMMMMMMMMMMMMMMMMMMMMMMMh+..:///:-.:yNMMMMM')
    log.ODM_INFO('MMMMMMMMNdyssyhdMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMdhyssydNMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMmyoooyNMMMMNsooooshNMMMMMdssdMMMMMmssdMMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMm:  ``` `+NMMm   ``` `/mMMM+  `mMMMM-  +MMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMN.  sNMm+  +MMm  /MMms` `mMM/   :MMMo   /MMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMy  /MMMMM`  NMm  /MMMMs  +MM: -` yMd `` :MMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMM+  sMMMMM:  hMm  /MMMMd  -MM- /o `m. h- -MMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMo  oMMMMM-  dMm  /MMMMh  :MM- +M- ` +M: .MMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMd  .NMMMd  .MMm  /MMMN-  sMM. oMd` -MM/ `MMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMs  .os+` `dMMm  .oo/`  +MMM` oMMmdNMM+  NMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMd+.```-oNMMMm.````.:omMMMM..yMMMMMMMo..NMMMMMMMMMMM')
    log.ODM_INFO('MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM')    
    log.ODM_INFO('OpenDroneMap app finished - %s' % system.now())
