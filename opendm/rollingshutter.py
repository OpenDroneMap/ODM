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

    'dji fc350': 30, # Inspire 1

    'gopro hero4 black': 30, # GoPro Hero 4 Black
    'gopro hero8 black': 17 # GoPro Hero 8 Black

    # Help us add more! Open a pull request.
}
DEFAULT_RS_READOUT = 30 # Just a guess

def make_model_key(make, model):
    return ("%s %s" % (make.strip(), model.strip())).lower().strip()

warn_db_missing = {}

def get_rolling_shutter_readout(make, model, override_value=0):
    global warn_db_missing

    if override_value > 0:
        return override_value
    
    key = make_model_key(make, model)
    if key in RS_DATABASE:
        return float(RS_DATABASE[key])
    else:
        # Warn once
        if not key in warn_db_missing:
            log.ODM_WARNING("Rolling shutter readout time for \"%s %s\" is not in our database, using default of %sms which might be incorrect. Use --rolling-shutter-readout to set an actual value." % (make, model, DEFAULT_RS_READOUT))
            warn_db_missing[key] = True
        return float(DEFAULT_RS_READOUT)