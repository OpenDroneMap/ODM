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
    log.ODM_INFO('                                                                ')
    log.ODM_INFO('                                                                ')
    log.ODM_INFO('                                                                ')    
    log.ODM_INFO('     .ohNMNNMMmy/`                             /ymMMNNMNdo-     ')
    log.ODM_INFO('   -hMd+-`   `:omNs`                        `oNNs:.   `.+hMd:   ')
    log.ODM_INFO('  oMh-           /NN:                      .mN+           .yMy  ')
    log.ODM_INFO(' sMo          .+` `dM:                    .Nm.  /-          /Mh ')
    log.ODM_INFO('-Mh          oNy`  `Nm                    hM-   oNs`         oM+')
    log.ODM_INFO('yM-       +smd-     sM:                  .Mh     .hNso`      `Md')
    log.ODM_INFO('hM`      /MMM       +M/                  -Ms       mMMs       Nm')
    log.ODM_INFO('sM:    `sMy+-       sM-                  `Md       .+sNy.    .Mh')
    log.ODM_INFO('-Md   :Nd-         .Nm                    yM:         .yN+   yM/')
    log.ODM_INFO(' +My  .:          .mN-        ..-..       `mN-          -.  oMy ')
    log.ODM_INFO('  +Nm:          `oNMy.   .+hmMMMMMMMmho-``-yMMs`          -dMo  ')
    log.ODM_INFO('   .yMms/.```-/yNNMMMMNNNMNMMy-oM+:hMMmMMMMMMMmMh+-.``.:odMh-   ')
    log.ODM_INFO('     `/ydNMMMNhs: /MMMMNo:NN:  +M+  /Mm-sNMMM+ -ohmMMMNmy+.     ')
    log.ODM_INFO('                   NMMm.`mM-   +M+   :Md -NMM`                  ')
    log.ODM_INFO('                  `MMsdMNMd:-``oM+ `-:dMNMdyMd`                 ')
    log.ODM_INFO('                  sM+  .MNhmNMMMMMMMNmhNN.  +My                 ')
    log.ODM_INFO('                 .Md   /Ms    `oMo`    yM-   dM.                ')
    log.ODM_INFO('                 oM+   oM/     +M+     +M+   +Mo                ')
    log.ODM_INFO('                 yMs+++dMs+++++hMh+++++yMh+++sMy                ')
    log.ODM_INFO('                 sMdhhhmMdyyyyymMmhhhhhdMmhhhdMy                ')
    log.ODM_INFO('                 /M+   oM+     oM+     oM+   oM+                ')
    log.ODM_INFO('                 `Nm`  :My`-:/+hMh+/:.`dM.  `NN`                ')
    log.ODM_INFO('                  /My .+MMMNmdhmMmhdmMMMN+. hM+                 ')
    log.ODM_INFO('                   sMmMdyMh    oM+    dMydMNMo                  ')
    log.ODM_INFO('       `-/+++/-    oMMN/ hMo   oM+   sMs +NMM.   .:+++/:`       ')
    log.ODM_INFO('    `odMmhyyydNNh/-NMMMMmodMs` oM+ `yMh+mMMMMy:yNMdhyyhmMmo.    ')
    log.ODM_INFO('   oNm+.       -sNMMMMmhhmMMMNsyMysNMMMmyhNMMMMy:       `/dMy`  ')
    log.ODM_INFO(' `dM+            `yMN.    ./shdmmmdhs+.    yMd.            /Nm. ')
    log.ODM_INFO(' dM:  /h-          sMo                    :Mh          .yo  .Nm`')
    log.ODM_INFO('/Mo   `oMy`         mM`                   mN`        `oNy`   /Ms')
    log.ODM_INFO('yM.     .hNms       oM/                  .My       +mNm-      Mm')
    log.ODM_INFO('hM.      -NMM:      +M/                  -My      .NMM/       Nm')
    log.ODM_INFO('oM/       `.oNy.    hM.                  `Nm    `sMs.`       -My')
    log.ODM_INFO('`NN`         .yN.  /My                    oMo   md-         `dM.')
    log.ODM_INFO(' -Nm-             +Md`                     yMo             .dM/ ')
    log.ODM_INFO('  .dMy-         :dMs                        +Nm/`        .oNm-  ')
    log.ODM_INFO('    :hMmyo+/+ohNNs.                          `omNhs+//+smMd/    ')
    log.ODM_INFO('      `:oyhhys+-                                -+syhhyo/`      ')
    log.ODM_INFO('                                                                ')
    log.ODM_INFO('                                                                ')
    log.ODM_INFO('                                                                ')    
    log.ODM_INFO('OpenDroneMap app finished - %s' % system.now())
