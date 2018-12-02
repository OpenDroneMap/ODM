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
  
    log.ODM_INFO('     :sdNmddmNmho-                            .+ymNmddmNds/`    ')
    log.ODM_INFO('  `oNdo-`     `:sNd/                        -hNy/`      .+dNs`  ')
    log.ODM_INFO(' -mm:            `oNy`                     sMs`            -dN: ')
    log.ODM_INFO('.Nm.         -hd`  :Mh                    oM+   hd:         `hM-')
    log.ODM_INFO('sM:       /+hm+`    yM-                  `Md     /dd+/`      .Mh')
    log.ODM_INFO('hM.      +MMN`      +M/                  -My      `dMMs       Nm')
    log.ODM_INFO('oM/    :hd+:`       hM-                  `Nm       `:/hd/    -My')
    log.ODM_INFO('`mN-  :h+`         +Ms                    +Ms         `:h+  .dN.')
    log.ODM_INFO(' .hNo` `         -yMh`    -/osyhhhyso/-   `yMh:           `/md- ')
    log.ODM_INFO('  `/hmy/-.```.:+hmMMNmhyhmmMMd+yMsodMMmmddmNMMmdo:.```.-/smd+`  ')
    log.ODM_INFO('    `.+yhhhhhhys/.+MMMMmo+Nm/` +M+ `+Nm/smMMM+.:oyhhhhhhy+-`    ')
    log.ODM_INFO('        ``````     NMMN+:Nm-   +M+   -Nd-+NMM-    ``````        ')
    log.ODM_INFO('                  -Mm:ohNMmyo++yMy/+oyNMNho:mm-                 ')
    log.ODM_INFO('                 `dN.  -Mh-/+osdMdso+/-dM.  .Nm`                ')
    log.ODM_INFO('                 /Mo   oM+     +M+     oM/   oM+                ')
    log.ODM_INFO('                 sMo///hMs/////yMy/////sMh///sMy                ')
    log.ODM_INFO('                 sMhsssmMhsssssdMdsssssdMdyyyhMy                ')
    log.ODM_INFO('                 :Ms   +Mo    `oMo`    sM/   yM/                ')
    log.ODM_INFO('                  yM:  :MmoyhhhmMmhhhyoNM-  /Mh                 ')
    log.ODM_INFO('                  `dNshdmMd/:--sMs--:/mMddhsNd`                 ')
    log.ODM_INFO('      `..::/:-.`   oMMN/.hN/   oM+   +My`+NMM.  `.-:/::-.`      ')
    log.ODM_INFO('    -ohddyssyhddy/:mMMMMdsmMs. oM+ .yMdodMMMMy/yhdhyssyddhs-`   ')
    log.ODM_INFO('  -ymy:.       ./hMMMdyooymNNNhdMdhNNNmy+ohNMMd+-`      `:smh:  ')
    log.ODM_INFO(' /Nh. `-`         /mN.     .:/+ooo+/:.    `dN+          .. .yNo ')
    log.ODM_INFO('-Nh`  -dh:`        .Nd                    yM:         -ym/   sM/')
    log.ODM_INFO('yM-    `/mhy/       sM:                  .Mh       :yhm+`    `Md')
    log.ODM_INFO('hM.      -NNM:      +M/                  -My      -NNN/       Nm')
    log.ODM_INFO('+Mo       `./dh:    dN.                   mN`   -ym+.`       :Ms')
    log.ODM_INFO(' hN/          /y` `yM+                    :Nh`  s+`         -Nd`')
    log.ODM_INFO('  sNy-          `/dN/                      -mm+`          .sNy` ')
    log.ODM_INFO('   .sNds/-..-:+yNd+`                         /dNho:-..-/odNy-   ')
    log.ODM_INFO('      -+syhdhyo/.                              `:oyhdhhs+-      ')
    log.ODM_INFO('OpenDroneMap app finished - %s' % system.now())
