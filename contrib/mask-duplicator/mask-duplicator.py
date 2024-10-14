#!/usr/bin/env python3
import sys
import os

import PIL

from PIL import Image

import shutil

from tqdm import tqdm

import argparse
parser = argparse.ArgumentParser()

# Usage:
# python exif_renamer.py <path to folder of images to rename> <output folder>

parser.add_argument("file_dir", help="input folder of images")
parser.add_argument("output_dir", help="output folder to copy images to")
parser.add_argument("mask_file", help="filename or path to Mask file to be duplicated for all images")
parser.add_argument("-f", "--force", help="don't ask for confirmation", action="store_true")

args = parser.parse_args()

file_dir = args.file_dir
mask_file_path = args.mask_file 
output_dir = args.output_dir

file_count = len(os.listdir(file_dir))

if args.force is False:
    print("Input dir: " + str(file_dir))
    print("Output folder: " + str(output_dir) + " (" + str(file_count) + " files)")
    confirmation = input("Confirm processing [Y/N]: ")
    if confirmation.lower() in ["y"]:
        pass
    else:
        sys.exit()

os.makedirs(output_dir, exist_ok=True)

no_exif_n = 0

# Uses tqdm() for the progress bar, if not needed swap with
# for filename in os.listdir(file_dir):
for filename in tqdm(os.listdir(file_dir)):
    old_path = mask_file_path
    #print(mask_file_path)
    file_name, file_ext = os.path.splitext(filename)

    try:
        img = Image.open(old_path)
    except PIL.UnidentifiedImageError as img_err:
        # if it tries importing a file it can't read as an image
        # can be commented out if you just wanna skip errors
        sys.stderr.write(str(img_err) + "\n")
        continue
    new_path = os.path.join(output_dir, file_name + "_mask" + file_ext)
    #print(new_path) # debugging
    shutil.copy(old_path, new_path)
print("Done!")
