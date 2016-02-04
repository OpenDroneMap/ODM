from opendm import config
from opendm import context
from opendm import log
from opendm import system

import sys
import ecto
from scripts.odm_app import ODMApp
from scripts.opensfm import ODMOpenSfMCell

app = None

def setup_prematching(self):
    args = {'end_with':'resize','project_path':'tests/test_data'}
    global app
    app = ODMApp(args=config.args)

def teardown_prematching(self):
    # Delete the test folders
    # if folders exist
        # delete all but "images" folder

def setUp():
    # create opensfm cell
    # opensfm = ODMOpenSfMCell(use_exif_size=False,
    #                          feature_process_size=config.args['resize_to'],
    #                          feature_min_frames=config.args['min_num_features'],
    #                          processes=context.num_cores,
    #                          matching_gps_neighbors=config.args['matcher_k']),

def test_prematching_config():
    # assert that relevant run args are what they say they are

    def test_opensfm_config_file(self):
        # assert that the args i nthe opensfm config file match those it is set to
        # when metcher_distance > 0
        # when the arg is out of bounds
