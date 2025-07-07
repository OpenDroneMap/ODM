
import time
import numpy as np
import cv2
import os
import onnxruntime as ort
from opendm import log
from opendm.ai import read_image
from threading import Lock

mutex = Lock()

# Implementation based on https://github.com/danielgatis/rembg by Daniel Gatis

# Use GPU if it is available, otherwise CPU
provider = "CUDAExecutionProvider" if "CUDAExecutionProvider" in ort.get_available_providers() else "CPUExecutionProvider"

class BgFilter():
    def __init__(self, model):
        self.model = model

        log.ODM_INFO(' ?> Using provider %s' % provider)
        self.load_model()

    
    def load_model(self):
        log.ODM_INFO(' -> Loading the model')

        self.session = ort.InferenceSession(self.model, providers=[provider])

    def normalize(self, img, mean, std, size):
        im = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
        im_ary = np.array(im)
        im_ary = im_ary / np.max(im_ary)

        tmpImg = np.zeros((im_ary.shape[0], im_ary.shape[1], 3))
        tmpImg[:, :, 0] = (im_ary[:, :, 0] - mean[0]) / std[0]
        tmpImg[:, :, 1] = (im_ary[:, :, 1] - mean[1]) / std[1]
        tmpImg[:, :, 2] = (im_ary[:, :, 2] - mean[2]) / std[2]

        tmpImg = tmpImg.transpose((2, 0, 1))

        return {
            self.session.get_inputs()[0]
            .name: np.expand_dims(tmpImg, 0)
            .astype(np.float32)
        }

    def get_mask(self, img):
        height, width, c = img.shape

        with mutex:
            ort_outs = self.session.run(
                None,
                self.normalize(
                    img, (0.485, 0.456, 0.406), (0.229, 0.224, 0.225), (320, 320) # <-- image size
                ),
            )

        pred = ort_outs[0][:, 0, :, :]

        ma = np.max(pred)
        mi = np.min(pred)

        pred = (pred - mi) / (ma - mi)
        pred = np.squeeze(pred)

        pred *= 255
        pred = pred.astype("uint8")
        output = cv2.resize(pred, (width, height), interpolation=cv2.INTER_LANCZOS4)
        output[output > 127] = 255
        output[output <= 127] = 0

        return output

    def run_img(self, img_path, dest):
        img = read_image(img_path)
        mask  = self.get_mask(img)
        
        img_name = os.path.basename(img_path)
        fpath = os.path.join(dest, img_name)

        fname, _ = os.path.splitext(fpath)
        mask_name = fname + '_mask.png'
        cv2.imwrite(mask_name, mask)

        return mask_name
