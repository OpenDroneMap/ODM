import os
from opendm.net import download
from opendm import log
import zipfile
import time
import sys
import rawpy
import cv2

def read_image(img_path):
    if img_path[-4:].lower() in [".dng", ".raw", ".nef"]:
        try:
            with rawpy.imread(img_path) as r:
                img = r.postprocess(output_bps=8, use_camera_wb=True, use_auto_wb=False)
        except:
            return None
    else:
        img = cv2.imread(img_path, cv2.IMREAD_COLOR)
        if img is None:
            return None

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    return img


def get_model(namespace, url, version, name = "model.onnx"):
    version = version.replace(".", "_")

    base_dir = os.path.join(os.path.dirname(__file__), "..")
    if sys.platform == 'win32':
        base_dir = os.path.join(os.getenv('PROGRAMDATA'),"ODM")
    base_dir = os.path.join(os.path.abspath(base_dir), "storage", "models")
    
    namespace_dir = os.path.join(base_dir, namespace)
    versioned_dir = os.path.join(namespace_dir, version)

    if not os.path.isdir(versioned_dir):
        os.makedirs(versioned_dir, exist_ok=True)
    
    # Check if we need to download it
    model_file = os.path.join(versioned_dir, name)
    if not os.path.isfile(model_file):
        log.ODM_INFO("Downloading AI model from %s ..." % url)

        last_update = 0

        def callback(progress):
            nonlocal last_update

            time_has_elapsed = time.time() - last_update >= 2

            if time_has_elapsed or int(progress) == 100:
                log.ODM_INFO("Downloading: %s%%" % int(progress))
                last_update = time.time()

        try:
            downloaded_file = download(url, versioned_dir, progress_callback=callback)
        except Exception as e:
            log.ODM_WARNING("Cannot download %s: %s" % (url, str(e)))
            return None

        if os.path.basename(downloaded_file).lower().endswith(".zip"):
            log.ODM_INFO("Extracting %s ..." % downloaded_file)
            with zipfile.ZipFile(downloaded_file, 'r') as z:
                z.extractall(versioned_dir)
            os.remove(downloaded_file)
        
        if not os.path.isfile(model_file):
            log.ODM_WARNING("Cannot find %s (is the URL to the AI model correct?)" % model_file)
            return None
        else:
            return model_file
    else:
        return model_file