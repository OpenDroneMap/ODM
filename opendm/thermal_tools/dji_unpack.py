from PIL import Image
import numpy as np
from opendm import system
from opendm import log

from opendm.thermal_tools.thermal_utils import sensor_vals_to_temp

def extract_temperatures_dji(photo, image, dataset_tree):
        """Extracts the DJI-encoded thermal image as 2D floating-point numpy array with temperatures in degC.
        The raw sensor values are obtained using the sample binaries provided in the official Thermal SDK by DJI.
        The executable file is run and generates a 16 bit unsigned RAW image with Little Endian byte order.
        Link to DJI Forum post: https://forum.dji.com/forum.php?mod=redirect&goto=findpost&ptid=230321&pid=2389016
        """
        # Hardcoded metadata for mean of values
        # This is added to support the possibility of extracting RJPEG from DJI M2EA 
        meta = {
            "Emissivity": 0.95,
            "ObjectDistance": 50, #This is mean value of flights for better results. Need to be changed later, or improved by bypassing options from task broker
            "AtmosphericTemperature": 20,
            "ReflectedApparentTemperature": 30,
            "IRWindowTemperature": 20,
            "IRWindowTransmission": 1,
            "RelativeHumidity": 40,
            "PlanckR1": 21106.77,
            "PlanckB": 1501,
            "PlanckF": 1,
            "PlanckO": -7340,
            "PlanckR2": 0.012545258,
        }

        if photo.camera_model == "MAVIC2-ENTERPRISE-ADVANCED":
            # Adding support for MAVIC2-ENTERPRISE-ADVANCED Camera images
            im = Image.open(f"{dataset_tree}/{photo.filename}")
            # concatenate APP3 chunks of data
            a = im.applist[3][1]
            for i in range(4, 14):
                a += im.applist[i][1]
            # create image from bytes
            try:
                img = Image.frombytes("I;16L", (640, 512), a)
            except ValueError as e:
                log.ODM_ERROR("Error during extracting temperature values for file %s : %s" % photo.filename, e)
        else:
            log.ODM_DEBUG("Only DJI M2EA currently supported, please wait for new updates")
            return image
        # Extract raw sensor values from generated image into numpy array
        raw_sensor_np = np.array(img)
        ## extracting the temperatures from thermal images
        thermal_np = sensor_vals_to_temp(raw_sensor_np, **meta)
        return thermal_np