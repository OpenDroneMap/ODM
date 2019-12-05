import unittest
from opendm import types

class ODMPhotoMock:
    def __init__(self, filename):
        self.filename = filename
    
class TestTypes(unittest.TestCase):
    def setUp(self):
        pass

    def test_reconstruction(self):
        # Multi camera setup
        micasa_redsense_files = ['IMG_0298_1.tif','IMG_0298_2.tif','IMG_0298_3.tif','IMG_0298_4.tif','IMG_0298_5.tif','IMG_0299_1.tif','IMG_0299_2.tif','IMG_0299_3.tif','IMG_0299_4.tif','IMG_0299_5.tif','IMG_0300_1.tif','IMG_0300_2.tif','IMG_0300_3.tif','IMG_0300_4.tif', 'IMG_0300_5.tif']
        photos = [ODMPhotoMock(f) for f in micasa_redsense_files]
        recon = types.ODM_Reconstruction(photos)

        self.assertTrue(recon.multi_camera is not None)

        # Found all 5 bands
        for b in ["1", "2", "3", "4", "5"]:
            self.assertTrue(b in recon.multi_camera)
        self.assertTrue([p.filename for p in recon.multi_camera["1"]] == ['IMG_0298_1.tif', 'IMG_0299_1.tif', 'IMG_0300_1.tif'])

        # Missing a file
        micasa_redsense_files = ['IMG_0298_1.tif','IMG_0298_3.tif','IMG_0298_4.tif','IMG_0298_5.tif','IMG_0299_1.tif','IMG_0299_2.tif','IMG_0299_3.tif','IMG_0299_4.tif','IMG_0299_5.tif','IMG_0300_1.tif','IMG_0300_2.tif','IMG_0300_3.tif','IMG_0300_4.tif', 'IMG_0300_5.tif']
        photos = [ODMPhotoMock(f) for f in micasa_redsense_files]
        recon = types.ODM_Reconstruction(photos)

        self.assertTrue(recon.multi_camera is None)

        # Parrot Sequoia pattern
        sequoia_files = ['IMG_180822_140144_0613_GRE.TIF','IMG_180822_140144_0613_NIR.TIF','IMG_180822_140144_0613_RED.TIF','IMG_180822_140144_0613_REG.TIF','IMG_180822_140146_0614_GRE.TIF','IMG_180822_140146_0614_NIR.TIF','IMG_180822_140146_0614_RED.TIF','IMG_180822_140146_0614_REG.TIF']
        photos = [ODMPhotoMock(f) for f in sequoia_files]
        recon = types.ODM_Reconstruction(photos)
        self.assertTrue(recon.multi_camera is not None)

        # Found 4 bands
        self.assertEqual(len(recon.multi_camera), 4)
        
        # Single camera
        dji_files = ['DJI_0018.JPG','DJI_0019.JPG','DJI_0020.JPG','DJI_0021.JPG','DJI_0022.JPG','DJI_0023.JPG']
        photos = [ODMPhotoMock(f) for f in dji_files]
        recon = types.ODM_Reconstruction(photos)
        self.assertTrue(recon.multi_camera is None)

if __name__ == '__main__':
    unittest.main()