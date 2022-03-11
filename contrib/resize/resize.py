import argparse
import os
import glob
import shutil
from PIL import Image
import piexif
import multiprocessing
from multiprocessing.pool import ThreadPool
import sys
sys.path.append("../../")
from opendm.gcp import GCPFile

parser = argparse.ArgumentParser(description='Exif Image Resize')
parser.add_argument('--input', '-i',
                    metavar='<path>',
                    required=True,
                    help='Path to input image/GCP or image folder')
parser.add_argument('--output', '-o',
                    metavar='<path>',
                    required=True,
                    help='Path to output image/GCP or image folder')
parser.add_argument('--force', '-f',
                    action='store_true',
                    default=False,
                    help='Overwrite results')
parser.add_argument('amount',
                    metavar='<pixel|percentage%>',
                    type=str,
                    help='Pixel of largest side or percentage to resize images by')
args = parser.parse_args()

def die(msg):
    print(msg)
    exit(1)

class nonloc:
    errors = 0

def resize_image(image_path, out_path, resize_to, out_path_is_file=False):
    """
    :param image_path: path to the image
    :param out_path: path to the output directory or file
    :param resize_to: percentage ("perc%") or pixels
    """
    try:
        im = Image.open(image_path)
        path, ext = os.path.splitext(image_path)
        if out_path_is_file:
            resized_image_path = out_path
        else:
            resized_image_path = os.path.join(out_path, os.path.basename(image_path))

        width, height = im.size
        max_side = max(width, height)

        if isinstance(resize_to, str) and resize_to.endswith("%"):
            ratio = float(resize_to[:-1]) / 100.0
        else:
            ratio = float(resize_to) / float(max_side)

        resized_width = int(width * ratio)
        resized_height = int(height * ratio)

        im.thumbnail((resized_width, resized_height), Image.LANCZOS)

        driver = ext[1:].upper()
        if driver == 'JPG':
            driver = 'JPEG'

        if 'exif' in im.info:
            exif_dict = piexif.load(im.info['exif'])
            exif_dict['Exif'][piexif.ExifIFD.PixelXDimension] = resized_width
            exif_dict['Exif'][piexif.ExifIFD.PixelYDimension] = resized_height
            im.save(resized_image_path, driver, exif=piexif.dump(exif_dict), quality=100)
        else:
            im.save(resized_image_path, driver, quality=100)

        im.close()

        print("{} ({}x{}) --> {} ({}x{})".format(image_path, width, height, resized_image_path, resized_width, resized_height))
    except (IOError, ValueError) as e:
        print("Error: Cannot resize {}: {}.".format(image_path, str(e)))
        nonloc.errors += 1

def resize_gcp(gcp_path, out_path, resize_to, out_path_is_file=False):
    """
    :param gcp_path: path to the GCP
    :param out_path: path to the output directory or file
    :param resize_to: percentage ("perc%") or pixels
    """
    try:
        if out_path_is_file:
            resized_gcp_path = out_path
        else:
            resized_gcp_path = os.path.join(out_path, os.path.basename(gcp_path))

        if resize_to.endswith("%"):
            ratio = float(resize_to[:-1]) / 100.0
        else:
            ratio = resize_to

        gcp = GCPFile(gcp_path)
        if gcp.entries_count() > 0:
            gcp.make_resized_copy(resized_gcp_path, ratio)
        else:
            raise ValueError("No GCP entries")

        print("{} --> {}".format(gcp_path, resized_gcp_path))
    except (IOError, ValueError) as e:
        print("Error: Cannot resize {}: {}.".format(gcp_path, str(e)))
        nonloc.errors += 1

if not args.amount.endswith("%"):
    args.amount = float(args.amount)
    if args.amount <= 0:
        die("Invalid amount")
else:
    try:
        if float(args.amount[:-1]) <= 0:
            die("Invalid amount")
    except:
        die("Invalid amount")


files = []
gcps = []

if os.path.isdir(args.input):
    for ext in ["JPG", "JPEG", "PNG", "TIFF", "TIF"]:
        files += glob.glob("{}/*.{}".format(args.input, ext))
        files += glob.glob("{}/*.{}".format(args.input, ext.lower()))
    gcps = glob.glob("{}/*.txt".format(args.input))
elif os.path.exists(args.input):
    _, ext = os.path.splitext(args.input)
    if ext.lower() == ".txt":
        gcps = [args.input]
    else:
        files = [args.input]
else:
    die("{} does not exist".format(args.input))

create_dir = len(files) > 1 or args.output.endswith("/") or len(gcps) > 1

if create_dir and os.path.isdir(args.output):
    if not args.force:
        die("{} exists, pass --force to overwrite results".format(args.output))
    else:
        shutil.rmtree(args.output)
elif not create_dir and os.path.isfile(args.output):
    if not args.force:
        die("{} exists, pass --force to overwrite results".format(args.output))
    else:
        os.remove(args.output)

if create_dir:
    os.makedirs(args.output)

pool = ThreadPool(processes=multiprocessing.cpu_count())

def resize(file):
    _, ext = os.path.splitext(file)
    if ext.lower() == ".txt":
        return resize_gcp(file, args.output, args.amount, not create_dir)
    else:
        return resize_image(file, args.output, args.amount, not create_dir)
pool.map(resize, files + gcps)

print("Process completed, {} errors.".format(nonloc.errors))

