# Script for Time-SIFT multi-temporal images alignment with ODM
#
# This is python script for ODM, based on the following publication :
#
# D. Feurer, F. Vinatier, Joining multi-epoch archival aerial images in a single SfM block allows 3-D change detection
#    with almost exclusively image information, ISPRS Journal of Photogrammetry and Remote Sensing, Volume 146, 2018,
#    Pages 495-506, ISSN 0924-2716, https://doi.org/10.1016/j.isprsjprs.2018.10.016.

import subprocess
import json
import os
import shutil
from pathlib import Path
import sys
import argparse
import textwrap

def main(argv):
    # Parsing and checking args
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter, description=textwrap.dedent('''\
            Timesift_odm.py datasetdir [-t <timesift-dir>] [-i <imageepochs-file>] [<options passed to ODM>]

                you can add options passed to ODM, for instance [--end-with odm_filterpoints] so that the final step is point clouds
                these options are not checked for before the final runs of each epoch, so use it carefully
            '''))
    parser.add_argument('datasetdir', help='dataset directory')
    parser.add_argument('-t', '--timesift-dir',
                        help='Time-SIFT directory ; default value : "time-sift-block" # must be in the datasetdir')
    parser.add_argument('-i', '--imageepochs-file',
                        help='Text file describing epochs ; default value : "images_epochs.txt" # must be in the TIMESIFT_DIR ')
    args, additional_options_to_rerun = parser.parse_known_args()
    datasets_DIR = Path(args.datasetdir).absolute().as_posix()
    if args.timesift_dir:
        timesift_DIR = args.timesift_dir
    else:
        timesift_DIR = 'time-sift-block'
    if args.imageepochs_file:
        images_epochs_file = args.imageepochs_file
    else:
        images_epochs_file = 'images_epochs.txt'
    if '-h' in sys.argv or '--help' in sys.argv:
        parser.print_help()
        sys.exit()
    if additional_options_to_rerun:  # for instance, --end-with odm_filterpoints
        print(f'[Time-SIFT]    Options passed to ODM for the final steps: {additional_options_to_rerun}')
        print(f'[Time-SIFT]    \033[93mWARNING there is no check of these options done before the last ODM call\033[0m')

    def check_path_args(var: Path):
        if not var.exists():
            print(
                f'\033[91m[Time-SIFT]    ERROR: the {var.as_posix()} directory does not exist. Exiting program\033[0m')
            exit()

    check_path_args(Path(datasets_DIR))
    check_path_args(Path(datasets_DIR, timesift_DIR))
    check_path_args(Path(datasets_DIR, timesift_DIR, images_epochs_file))

    def clean_reconstruction_dict(subdict, key, images):
        """
        Delete subdict elements where the key do not match any name in the images list.
        To create the {epoch} block with only images of this epoch
        """
        # The list of valid images is prepared by removing any extension (to be robust to the .tif added by ODM)
        valid_images = {os.path.basename(image).split(os.extsep)[0] for image in images}
        for item_key in list(subdict[key]):
            image_name = os.path.basename(item_key).split(os.extsep)[0]
            if image_name not in valid_images:
                del subdict[key][item_key]

    ### Read images.txt file and create a dict of images/epochs
    images_epochs_dict = {}
    with open(Path(datasets_DIR, timesift_DIR, images_epochs_file), 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue  # Empty lines are skipped
            image, epoch = line.split()
            if epoch not in images_epochs_dict:
                images_epochs_dict[epoch] = []
            images_epochs_dict[epoch].append(image)

    ### Check for existing epochs directories before computing anything (these directories must be deleted by hand)
    path_exists_error = False
    for epoch in images_epochs_dict:
        if Path(datasets_DIR, epoch).exists():
            if path_exists_error:
                print(f"sudo rm -rf {Path(datasets_DIR, epoch).as_posix()}")
            else:
                print(f'\033[91m[Time-SIFT]    ERROR: {Path(datasets_DIR, epoch).as_posix()} already exists.\033[0m')
                print(f"               Other epochs probably also exist.")
                print(
                    f"               The problem is \033[93mI CAN'T\033[0m delete it by myself, it requires root privileges.")
                print(
                    f"               The good news is \033[92mYOU CAN\033[0m do it with the following command (be careful).")
                print(f'\033[91m               => Consider doing it (at your own risks). Exiting program\033[0m')
                print(f"- Commands to copy/paste (I'm kind, I prepared all the necessary commands for you).")
                print(f"sudo rm -rf {Path(datasets_DIR, epoch).as_posix()}")
                path_exists_error = True
    if path_exists_error:
        exit()

    ### LAUNCH global alignment (Time-SIFT multitemporal block)
    try:
        subprocess.run(['docker', 'run', '-i', '--rm', '-v', datasets_DIR + ':/datasets',
                        'opendronemap/odm', '--project-path', '/datasets', timesift_DIR, '--end-with', 'opensfm'])
    except:
        print(f'\033[91m[Time-SIFT]    ERROR: {sys.exc_info()[0]}\033[0m')
        exit()
    print('\033[92m[Time-SIFT]    Sfm on multi-temporal block done\033[0m')

    print('[Time-SIFT]    Going to dense matching on all epochs...')
    ### Loop on epochs for the dense matching
    for epoch in images_epochs_dict:
        #### We first duplicate the time-sift multitemporal block to save sfm results
        shutil.copytree(Path(datasets_DIR, timesift_DIR),
                        Path(datasets_DIR, epoch))

        #### Reads the datasets/{epoch}/opensfm/undistorted/reconstruction.json file that has to be modified
        with open(Path(datasets_DIR, epoch, 'opensfm', 'undistorted', 'reconstruction.json'), mode="r",
                  encoding="utf-8") as read_file:
            reconstruction_dict = json.load(read_file)

        #### Removes images in this json dict (we delete the shot and the rig_instances that do not correspond to this epoch)
        images = images_epochs_dict[epoch]
        clean_reconstruction_dict(reconstruction_dict[0], 'shots', images)
        clean_reconstruction_dict(reconstruction_dict[0], 'rig_instances', images)

        #### Makes a backup of the reconstruction.json file and writes the modified json
        shutil.copy(Path(datasets_DIR, epoch, 'opensfm', 'undistorted', 'reconstruction.json'),
                    Path(datasets_DIR, epoch, 'opensfm', 'undistorted', 'reconstruction.json.bak'))
        with open(Path(datasets_DIR, epoch, 'opensfm', 'undistorted', 'reconstruction.json'), mode="w",
                  encoding="utf-8") as write_file:
            json.dump(reconstruction_dict, write_file)

        #### Launches dense matching from the good previous step, with possible options (e.g. => to stop at the point clouds)
        command_rerun = ['docker', 'run', '-i', '--rm', '-v', datasets_DIR + ':/datasets',
                             'opendronemap/odm',
                             '--project-path', '/datasets', epoch,
                             '--rerun-from', 'openmvs']
        if additional_options_to_rerun:
            print(f'[Time-SIFT]        Epoch {epoch}: Rerun with additionnal options: {additional_options_to_rerun}')
            command_rerun.extend(additional_options_to_rerun)
        else:
            print(f'[Time-SIFT]        Epoch {epoch}: Default full rerun')
        result = subprocess.run(command_rerun)
        if result.returncode != 0:
            print(f'\033[91m[Time-SIFT]    ERROR in processing epoch {epoch}\033[0m')
            print(f'{result=}')
            exit(result.returncode)
        print(f'\033[92m[Time-SIFT]        Epoch {epoch} finished\033[0m')

    print('§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§')
    print('§§§      §§  §§§  §§§§§  §§§      §§§§§§§§§§      §§  §§      §§      §§§')
    print('§§§§§  §§§§  §§§   §§§   §§§  §§§§§§§§§§§§§   §§§§§§  §§  §§§§§§§§  §§§§§')
    print('§§§§§  §§§§  §§§    §    §§§    §§§§    §§§§§   §§§§  §§    §§§§§§  §§§§§')
    print('§§§§§  §§§§  §§§  §§§§§  §§§  §§§§§§§§§§§§§§§§§   §§  §§  §§§§§§§§  §§§§§')
    print('§§§§§  §§§§  §§§  §§§§§  §§§      §§§§§§§§§      §§§  §§  §§§§§§§§  §§§§§')
    print('§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§')
    print('  \033[92mTime-SIFT with ODM finished, congrats !\033[0m Want to cite the method ?')
    print('=> D. Feurer, F. Vinatier, Joining multi-epoch archival aerial images in ')
    print('   a single SfM block allows 3-D change detection with almost exclusively')
    print('   image information, ISPRS Journal of Photogrammetry and Remote Sensing,')
    print('   2018, https://doi.org/10.1016/j.isprsjprs.2018.10.016                ')

if __name__ == "__main__":
    main(sys.argv[1:])