#!/usr/bin/env python3

# Originally developed by Ming Chia at the Australian Plant Phenomics Facility (Australian National University node)

# Usage:
# exif_binner.py <args> <path to folder of images to rename> <output folder>

# standard libraries
import sys
import os
import shutil
import re
import csv
import math
import argparse

# other imports
import PIL
from PIL import Image, ExifTags
from tqdm import tqdm # optional: see "swap with this for no tqdm" below

parser = argparse.ArgumentParser()

# required args
parser.add_argument("file_dir", help="input folder of images")
parser.add_argument("output_dir", help="output folder to copy images to")

# args with defaults
parser.add_argument("-b", "--bands", help="number of expected bands per capture", type=int, default=5)
parser.add_argument("-s", "--sequential", help="use sequential capture group in filenames rather than original capture ID", type=bool, default=True)
parser.add_argument("-z", "--zero_pad", help="if using sequential capture groups, zero-pad the group number to this many digits. 0 for no padding, -1 for auto padding", type=int, default=5)
parser.add_argument("-w", "--whitespace_replace", help="replace whitespace characters with this character", type=str, default="-")

# optional args no defaults
parser.add_argument("-l", "--logfile", help="write image metadata used to this CSV file", type=str)
parser.add_argument("-r", "--replace_filename", help="use this instead of using the original filename in new filenames", type=str)
parser.add_argument("-f", "--force", help="don't ask for confirmation", action="store_true")
parser.add_argument("-g", "--no_grouping", help="do not apply grouping, only validate and add band name", action="store_true")
args = parser.parse_args()

file_dir = args.file_dir
output_dir = args.output_dir
replacement_character = args.whitespace_replace
expected_bands = args.bands
logfile = args.logfile

output_valid = os.path.join(output_dir, "valid")
output_invalid = os.path.join(output_dir, "invalid")

file_count = len(os.listdir(file_dir))

auto_zero_pad = len(str(math.ceil(float(file_count) / float(expected_bands))))

if args.zero_pad >= 1:
    if int("9" * args.zero_pad) < math.ceil(float(file_count) / float(expected_bands)):
        raise ValueError("Zero pad must have more digits than maximum capture groups! Attempted to pad " + str(args.zero_pad) + " digits with "
                         + str(file_count) + " files and " + str(expected_bands) + " bands (up to " + str(math.ceil(float(file_count) / float(expected_bands)))
                         + " capture groups possible, try at least " + str(auto_zero_pad) + " digits to zero pad)")

if args.force is False:
    print("Input dir: " + str(file_dir) + " (" + str(file_count) + " files)")
    print("Output folder: " + str(output_dir))
    if args.replace_filename:
        print("Replacing all basic filenames with: " + args.replace_filename)
    else:
        print("Replace whitespace in filenames with: " + replacement_character)
    print("Number of expected bands: " + str(expected_bands))
    if logfile:
        print("Save image processing metadata to: " + logfile)
    confirmation = input("Confirm processing [Y/N]: ")
    if confirmation.lower() in ["y"]:
        pass
    else:
        sys.exit()

no_exif_n = 0

images = []

print("Indexing images ...")


# for filename in os.listdir(file_dir): # swap with this for no tqdm
for filename in tqdm(os.listdir(file_dir)):
    old_path = os.path.join(file_dir, filename)
    file_name, file_ext = os.path.splitext(filename)
    image_entry = {"name": filename, "valid": True, "band": "-", "ID": "-", "group": 0, "DateTime": "-", "error": "-"}  # dashes to ensure CSV exports properly, can be blank
    try:
        img = Image.open(old_path)
    except PIL.UnidentifiedImageError as img_err:
        # if it tries importing a file it can't read as an image
        # uncomment to print errors
        # sys.stderr.write(str(img_err) + "\n")
        no_exif_n += 1
        if logfile:
            image_entry["valid"] = False
            image_entry["error"] = "Not readable as image: " + str(img_err)
            images.append(image_entry)
        continue
    for key, val in img.getexif().items():
        if key in ExifTags.TAGS:
            # print(ExifTags.TAGS[key] + ":" + str(val)) # debugging
            if ExifTags.TAGS[key] == "XMLPacket":
                # find bandname
                bandname_start = val.find(b'<Camera:BandName>')
                bandname_end = val.find(b'</Camera:BandName>')
                bandname_coded = val[(bandname_start + 17):bandname_end]
                bandname = bandname_coded.decode("UTF-8")
                image_entry["band"] = str(bandname)
                # find capture ID
                image_entry["ID"] = re.findall('CaptureUUID="([^"]*)"', str(val))[0]
            if ExifTags.TAGS[key] == "DateTime":
                image_entry["DateTime"] = str(val)
    image_entry["band"].replace(" ", "-")
    if len(image_entry["band"]) >= 99:  # if it's too long, wrong value (RGB pic has none)
        # no exif present
        no_exif_n += 1
        image_entry["valid"] = False
        image_entry["error"] = "Image band name appears to be too long"
    elif image_entry["ID"] == "" and expected_bands > 1:
        no_exif_n += 1
        image_entry["valid"] = False
        image_entry["error"] = "No Capture ID found"
    if (file_ext.lower() in [".jpg", ".jpeg"]) and (image_entry["band"] == "-"):  # hack for DJI RGB jpgs
        # handle = open(old_path, 'rb').read()
        # xmp_start = handle.find(b'<x:xmpmeta')
        # xmp_end = handle.find(b'</x:xmpmeta')
        # xmp_bit = handle[xmp_start:xmp_end + 12]
        # image_entry["ID"] = re.findall('CaptureUUID="([^"]*)"', str(xmp_bit))[0]
        # image_entry["band"] = "RGB"  # TODO: we assume this. may not hold true for all datasets

        no_exif_n += 1  # this is just to keep a separate invalid message, comment out this whole if block and the jpgs shoud be handled by the "no capture ID" case
        image_entry["valid"] = False
        image_entry["error"] = "RGB jpg, not counting for multispec processing"
    images.append(image_entry)
    # print(new_path) # debugging

print(str(no_exif_n) + " files were not multispectral images")
no_matching_bands_n = 0
new_capture_id = 1
capture_ids = {}

images = sorted(images, key=lambda img: (img["DateTime"], img["name"]))

# now sort and identify valid entries
if not args.no_grouping:
    # for this_img in images: # swap with this for no tqdm
    for this_img in tqdm(images):
        if not this_img["valid"]:  # prefiltered in last loop
            continue
        same_id_images = [image for image in images if image["ID"] == this_img["ID"]]
        if len(same_id_images) != expected_bands:  # defaults to True, so only need to filter out not in
            no_matching_bands_n += 1
            this_img["valid"] = False
            this_img["error"] = "Capture ID has too few/too many bands"
        else:
            if this_img["ID"] in capture_ids.keys():
                this_img["group"] = capture_ids[this_img["ID"]]
            else:
                capture_ids[this_img["ID"]] = new_capture_id
                this_img["group"] = capture_ids[this_img["ID"]]  # a little less efficient but we know it works this way
                new_capture_id += 1
    print(str(no_matching_bands_n) + " images had unexpected bands in same capture")

os.makedirs(output_valid, exist_ok=True)
os.makedirs(output_invalid, exist_ok=True)

identifier = ""

# then do the actual copy
# for this_img in images: # swap with this for no tqdm
for this_img in tqdm(images):
    old_path = os.path.join(file_dir, this_img["name"])
    file_name, file_ext = os.path.splitext(this_img["name"])

    if args.whitespace_replace:
        file_name = replacement_character.join(file_name.split())
    if args.replace_filename and not args.no_grouping:
        file_name = args.replace_filename

    if this_img["valid"]:
        prefix = output_valid
        if args.no_grouping:
            file_name_full = file_name + "-" + this_img["band"] + file_ext
        else:
            # set ID based on args
            if args.sequential:
                if args.zero_pad == 0:
                    identifier = str(this_img["group"])
                elif args.zero_pad == -1:
                    identifier = str(this_img["group"]).zfill(auto_zero_pad)
                else:
                    identifier = str(this_img["group"]).zfill(args.zero_pad)
            else:
                identifier = this_img["ID"]
            file_name_full = identifier + "-" + file_name + "-" + this_img["band"] + file_ext
    else:
        prefix = output_invalid
        file_name_full = file_name + file_ext
    new_path = os.path.join(prefix, file_name_full)
    shutil.copy(old_path, new_path)

if logfile:
    header = images[0].keys()
    with open(logfile, 'w', newline='') as logfile_handle:
        dict_writer = csv.DictWriter(logfile_handle, header)
        dict_writer.writeheader()
        dict_writer.writerows(images)

print("Done!")
