import sys
from opendm import config
import ecto
from scripts.odm_app import ODMApp
from scripts.resize import ODMResizeCell
err = sys.stderr
# Fixture functions
app = None


def setup_odm():
    # initialize ecto
    global app
    app = ODMApp(args=config.args)
    plasm = ecto.Plasm()
    plasm.insert(app)
    err.write('MODULE SETUP\n')


def teardown_odm():
    # teardown ecto
    err.write('MODULE TEARDOWN\n')


# The tests
def test_config():
    # check the args
    test_args = 'tests/test_data' # sample test args
    assert test_args == config.args.get('project_path')


class TestResize:
    def __init__(self):
        # Initialize whatever

    def setUp(self):
        #call cell
        resizeCell = ODMResizeCell()

    def tearDown(self):
        #teardown functions


    # assert proper config/args

    # Check images in images_resize

    # Check images are resized (imagemagick?)
