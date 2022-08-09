
import time
import numpy as np
import cv2
import os
import onnx
import onnxruntime as ort
from .guidedfilter import guided_filter
from opendm import log
from threading import Lock

mutex = Lock()

# Use GPU if it is available, otherwise CPU
provider = "CUDAExecutionProvider" if "CUDAExecutionProvider" in ort.get_available_providers() else "CPUExecutionProvider"

class SkyFilter():

    def __init__(self, model, width = 384, height = 384):

        self.model = model
        self.width, self.height = width, height

        log.ODM_INFO(' ?> Using provider %s' % provider)
        self.load_model()

    
    def load_model(self):
        log.ODM_INFO(' -> Loading the model')
        onnx_model = onnx.load(self.model)

        # Check the model
        try:
            onnx.checker.check_model(onnx_model)
        except onnx.checker.ValidationError as e:
            log.ODM_INFO(' !> The model is invalid: %s' % e)
            raise
        else:
            log.ODM_INFO(' ?> The model is valid!')

        self.session = ort.InferenceSession(self.model, providers=[provider])     


    def get_mask(self, img):

        height, width, c = img.shape

        # Resize image to fit the model input
        new_img = cv2.resize(img, (self.width, self.height), interpolation=cv2.INTER_AREA)
        new_img = np.array(new_img, dtype=np.float32)

        # Input vector for onnx model
        input_v = np.expand_dims(new_img.transpose((2, 0, 1)), axis=0)
        ort_inputs = {self.session.get_inputs()[0].name: input_v}

        # Run the model
        with mutex:
            ort_outs = self.session.run(None, ort_inputs)

        # Get the output
        output = np.array(ort_outs)
        output = output[0][0].transpose((1, 2, 0))
        output = cv2.resize(output, (width, height), interpolation=cv2.INTER_LANCZOS4)
        output = np.array([output, output, output]).transpose((1, 2, 0))
        output = np.clip(output, a_max=1.0, a_min=0.0)

        return self.refine(output, img)


    def refine(self, pred, img):
        guided_filter_radius, guided_filter_eps = 20, 0.01
        refined = guided_filter(img[:,:,2], pred[:,:,0], guided_filter_radius, guided_filter_eps)

        res = np.clip(refined, a_min=0, a_max=1)
        
        # Convert res to CV_8UC1
        res = np.array(res * 255., dtype=np.uint8)
        
        # Thresholding
        res = cv2.threshold(res, 127, 255, cv2.THRESH_BINARY_INV)[1]
        
        return res
        

    def run_img(self, img_path, dest):

        img = cv2.imread(img_path, cv2.IMREAD_COLOR)
        if img is None:
            return None

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.array(img / 255., dtype=np.float32)

        mask  = self.get_mask(img)
        
        img_name = os.path.basename(img_path)
        fpath = os.path.join(dest, img_name)

        fname, _ = os.path.splitext(fpath)
        mask_name = fname + '_mask.png'
        cv2.imwrite(mask_name, mask)
        
        return mask_name
