from opendm import log

# Make Model (lowercase) --> readout time (ms)
RS_DATABASE = {
    'dji phantom vision fc200': 74, # Phantom 2
    
    'dji fc300s': 33, # Phantom 3 Advanced
    'dji fc300c': 33, # Phantom 3 Standard
    'dji fc300x': 33, # Phantom 3 Professional

    'dji fc330': 33, # Phantom 4
    'dji fc6310': 33, # Phantom 4 Professional

    'dji fc7203': 20, # Mavic Mini v1
    'dji fc3170': 27, # DJI Mavic Air 2
    'dji fc3411': 32, # DJI Mavic Air 2S

    'dji fc3582': lambda p: 26 if p.get_capture_megapixels() < 48 else 60, # DJI Mini 3 pro (at 48MP readout is 60ms, at 12MP it's 26ms) 

    'dji fc350': 30, # Inspire 1

    'gopro hero4 black': 30, # GoPro Hero 4 Black
    'gopro hero8 black': 17, # GoPro Hero 8 Black

    'teracube teracube one': 32 # TeraCube TeraCube_One TR1907Q Mobile Phone
    

    # Help us add more! 
    # See: https://github.com/OpenDroneMap/RSCalibration for instructions
}
DEFAULT_RS_READOUT = 30 # Just a guess

def make_model_key(make, model):
    return ("%s %s" % (make.strip(), model.strip())).lower().strip()

warn_db_missing = {}
info_db_found = {}

def get_rolling_shutter_readout(photo, override_value=0):
    global warn_db_missing
    global info_db_found

    make, model = photo.camera_make, photo.camera_model

    if override_value > 0:
        return override_value
    
    key = make_model_key(make, model)
    if key in RS_DATABASE:
        rsd = RS_DATABASE[key]
        val = DEFAULT_RS_READOUT

        if isinstance(rsd, int) or isinstance(rsd, float):
            val = float(rsd)
        elif callable(rsd):
            val = float(rsd(photo))
        else:
            log.ODM_WARNING("Invalid rolling shutter calibration entry, returning default of %sms" % DEFAULT_RS_READOUT)

        if not key in info_db_found:
            log.ODM_INFO("Rolling shutter profile for \"%s %s\" selected, using %sms as --rolling-shutter-readout." % (make, model, val))
            info_db_found[key] = True
        
        return val
    else:
        # Warn once
        if not key in warn_db_missing:
            log.ODM_WARNING("Rolling shutter readout time for \"%s %s\" is not in our database, using default of %sms which might be incorrect. Use --rolling-shutter-readout to set an actual value (see https://github.com/OpenDroneMap/RSCalibration for instructions on how to calculate this value)" % (make, model, DEFAULT_RS_READOUT))
            warn_db_missing[key] = True
        return float(DEFAULT_RS_READOUT)
