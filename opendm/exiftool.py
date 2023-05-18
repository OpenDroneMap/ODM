import json
import os
import tempfile
import base64
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

    params = {}

    for m in meta:
        if m not in tags:
            # All or nothing
            raise Exception("Cannot find %s in tags" % m)
        params[m] = (meta[m])(tags[m])
    
    return params