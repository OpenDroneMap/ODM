import json
import os
import tempfile
import base64
import numpy as np
from rasterio.io import MemoryFile
from opendm.system import run
from opendm import log
from opendm.utils import double_quote

def extract_raw_thermal_image_data(image_path):
    try:
        f, tmp_file_path = tempfile.mkstemp(suffix='.json')
        os.close(f)

        try:
            output = run("exiftool -b -x ThumbnailImage -x PreviewImage -j \"%s\" > \"%s\"" % (image_path, tmp_file_path), quiet=True)

            with open(tmp_file_path) as f:
                j = json.loads(f.read())

                if isinstance(j, list):
                    j = j[0] # single file
                    
                    if "RawThermalImage" in j:
                        imageBytes = base64.b64decode(j["RawThermalImage"][len("base64:"):])

                        with MemoryFile(imageBytes) as memfile:
                            with memfile.open() as dataset:
                                img = dataset.read()
                                bands, h, w = img.shape

                                if bands != 1:
                                    raise Exception("Raw thermal image has more than one band? This is not supported")

                                # (1, 512, 640) --> (512, 640, 1)
                                img = img[0][:,:,None]

                        del j["RawThermalImage"]
                    elif "ThermalData" in j:
                        thermal_data = base64.b64decode(j["ThermalData"][len("base64:"):])
                        thermal_data_buf = np.frombuffer(thermal_data, dtype=np.int16)

                        thermal_calibration = base64.b64decode(j["ThermalCalibration"][len("base64:"):])
                        thermal_calibration_buf = np.frombuffer(thermal_calibration, dtype=np.int16)

                        # TODO: how to interpret these?
                        # https://exiftool.org/forum/index.php?topic=11401.45
                        print(thermal_data_buf.shape)
                        print(thermal_calibration_buf.shape)
                        print(" ".join("%02x" % b for b in thermal_data_buf[0:10]))
                        print(thermal_data_buf[0:10])
                        print(thermal_calibration_buf[0:10])

                        # temperatures = np.right_shift(thermal_data_buf, 2).astype(np.float32)
                        # temperatures *= 0.0625
                        # temperatures -= 273.15
                        img = thermal_data_buf.reshape((512, 640))  # notice row, column format


                        # from PIL import Image
                        # img = Image.fromarray(im)

                        # temperatures = np.right_shift(thermal_data_buf, 2).astype(np.float32)
                        # temperatures *= 0.0625
                        # temperatures -= 273.15

                        # rows = 512
                        # cols = 640
                        # im = temperatures.reshape((rows, cols))  # notice row, column format

                        # dest_path = '/datasets/dji_thermal/out.tiff'
                        # img_thermal = Image.fromarray(im)
                        # img_thermal.save(dest_path)


                    return extract_temperature_params_from(j), img
                else:
                    raise Exception("Invalid JSON (not a list)")

        except Exception as e:
            log.ODM_WARNING("Cannot extract tags using exiftool: %s" % str(e))
            return {}, None
        finally:
            if os.path.isfile(tmp_file_path):
                os.remove(tmp_file_path)
    except Exception as e:
        log.ODM_WARNING("Cannot create temporary file: %s" % str(e))
        return {}, None

def unit(unit):
    def _convert(v):
        if isinstance(v, float):
            return v
        elif isinstance(v, str):
            if not v[-1].isnumeric():
                if v[-1].upper() != unit.upper():
                    log.ODM_WARNING("Assuming %s is in %s" % (v, unit))
                return float(v[:-1])
            else:
                return float(v)
        else:
            return float(v)
    return _convert

def extract_temperature_params_from(tags):
    # Defaults

    meta = {
        "Emissivity": float,
        "ObjectDistance": unit("m"),
        "AtmosphericTemperature": unit("C"),
        "ReflectedApparentTemperature": unit("C"),
        "IRWindowTemperature": unit("C"),
        "IRWindowTransmission": float,
        "RelativeHumidity": unit("%"),
        "PlanckR1": float,
        "PlanckB": float,
        "PlanckF": float,
        "PlanckO": float,
        "PlanckR2": float,
    }

    aliases = {
        "AtmosphericTemperature": ["AmbientTemperature"],
        "ReflectedApparentTemperature": ["ReflectedTemperature"],
#        "IRWindowTemperature": ["ReflectedApparentTemperature"], #fallback
    }

    params = {}

    for m in meta:
        keys = [m]
        keys += aliases.get(m, [])
        val = None
        for k in keys:
            if k in tags:
                val = (meta[m])(tags[k])
                break
        if val is None:
            raise Exception("Cannot find %s in tags" % m)
        
        params[m] = val
    
    return params