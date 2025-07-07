from opendm import log

# Make Model (lowercase) --> readout time (ms)
RS_DATABASE = {
    'autel robotics xt701': 25, # Autel Evo II 8k 
    'dji phantom vision fc200': 74, # Phantom 2
    
    'dji fc300s': 33, # Phantom 3 Advanced
    'dji fc300c': 33, # Phantom 3 Standard
    'dji fc300x': 33, # Phantom 3 Professional

    'dji fc330': 33, # Phantom 4
    'dji fc6310': 33, # Phantom 4 Professional

    'dji fc7203': lambda p: 19 if p.get_capture_megapixels() < 10 else 25, # DJI Mavic Mini v1 (at 16:9 => 9MP 19ms, at 4:3 => 12MP 25ms)
    'dji fc2103': 32, # DJI Mavic Air 1
    'dji fc3170': 27, # DJI Mavic Air 2
    'dji fc3411': 32, # DJI Mavic Air 2S
    
    'dji fc220': 64, # DJI Mavic Pro (Platinum)
    'hasselblad l1d-20c': lambda p: 47 if p.get_capture_megapixels() < 17 else 56, # DJI Mavic 2 Pro (at 16:10 => 16.8MP 47ms, at 3:2 => 19.9MP 56ms. 4:3 has 17.7MP with same image height as 3:2 which can be concluded as same sensor readout)
    'hasselblad l2d-20c': 16.6, # DJI Mavic 3 (not enterprise version)

    'dji fc3582': lambda p: 26 if p.get_capture_megapixels() < 48 else 60, # DJI Mini 3 pro (at 48MP readout is 60ms, at 12MP it's 26ms) 
    'dji fc8482': lambda p: (
        16 if p.get_capture_megapixels() < 12 else  # 12MP 16:9 mode (actual 9.1MP)
        21 if p.get_capture_megapixels() < 20 else  # 12MP 4:3 mode (actual 12.2MP)
        43 if p.get_capture_megapixels() < 45 else  # 48MP 16:9 mode (actual 36.6MP)
        58                                          # 48MP 4:3 mode (actual 48.8MP)
    ), # DJI Mini 4 Pro (readout varies by resolution and aspect ratio, image heights all different)
    
    'dji fc350': 30, # Inspire 1
    
    'dji mavic2-enterprise-advanced': 31, # DJI Mavic 2 Enterprise Advanced
    'dji zenmuse z30': 8, # DJI Zenmuse Z30
    
    'yuneec e90': 44, # Yuneec E90

    'gopro hero4 black': 30, # GoPro Hero 4 Black
    'gopro hero8 black': 17, # GoPro Hero 8 Black

    'teracube teracube one': 32, # TeraCube TeraCube_One TR1907Q Mobile Phone
    
    'fujifilm x-a5': 186, # FUJIFILM X-A5 Mirrorless Interchangeable Lens Camera

    'fujifilm x-t2': 35, # FUJIFILM X-T2 Mirrorless Interchangeable Lens Camera

    'autel robotics xl724': 29, # Autel Nano+

    'parrot anafi': 39, # Parrot Anafi

    'autel robotics xt705': 30, # Autel EVO II pro

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

