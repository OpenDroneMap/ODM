from opendm import config
from opendm import context
from opendm import log
from opendm import system

import sys
import ecto

from scripts.odm_app import ODMApp
from scripts.opensfm import ODMOpenSfMCell

def setup_app(self):
    # # set up test folder
    # mkdir_p test folder
    # git clone test_photos.git
    # # create ODMApp object
    # app = ODMApp(args=config.args)

def teardown_app(self):
    # Delete the test folder


class TestOpenSfM:
    def setUp(self):
        # # create opensfm cell
        # opensfm = ODMOpenSfMCell(use_exif_size=False,
        #                          feature_process_size=config.args['resize_to'],
        #                          feature_min_frames=config.args['min_num_features'],
        #                          processes=context.num_cores,
        #                          matching_gps_neighbors=config.args['matcher_k']),

    def test_prematching_config(self):
        # assert that relevant run args are what they say they are

    def test_opensfm_config_file(self):
        # assert that the args i nthe opensfm config file match those it is set to
        # when metcher_distance > 0
        # when the arg is out of bounds
