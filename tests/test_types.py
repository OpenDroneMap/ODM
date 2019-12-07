import unittest
from opendm import types

class ODMPhotoMock:
    def __init__(self, filename, band_name):
        self.filename = filename
        self.band_name = band_name
    
    def __str__(self):
        return "%s (%s)" % (self.filename, self.band_name)

    def __repr__(self):
        return self.__str__()
    
class TestTypes(unittest.TestCase):
    def setUp(self):
        pass

    def test_reconstruction(self):
        # Multi camera setup
        micasa_redsense_files = [('IMG_0298_1.tif', 'Red'), ('IMG_0298_2.tif', 'Green'), ('IMG_0298_3.tif', 'Blue'), ('IMG_0298_4.tif', 'NIR'), ('IMG_0298_5.tif', 'Rededge'), 
                                 ('IMG_0299_1.tif', 'Red'), ('IMG_0299_2.tif', 'Green'), ('IMG_0299_3.tif', 'Blue'), ('IMG_0299_4.tif', 'NIR'), ('IMG_0299_5.tif', 'Rededge'), 
                                 ('IMG_0300_1.tif', 'Red'), ('IMG_0300_2.tif', 'Green'), ('IMG_0300_3.tif', 'Blue'), ('IMG_0300_4.tif', 'NIR'), ('IMG_0300_5.tif', 'Rededge')]
        photos = [ODMPhotoMock(f, b) for f, b in micasa_redsense_files]
        recon = types.ODM_Reconstruction(photos)

        self.assertTrue(recon.multi_camera is not None)

        # Found all 5 bands
        for b in ["Red", "Blue", "Green", "NIR", "Rededge"]:
            self.assertTrue(b in recon.multi_camera)
        self.assertTrue([p.filename for p in recon.multi_camera["Red"]] == ['IMG_0298_1.tif', 'IMG_0299_1.tif', 'IMG_0300_1.tif'])

        # Missing a file
        micasa_redsense_files = [('IMG_0298_1.tif', 'Red'), ('IMG_0298_2.tif', 'Green'), ('IMG_0298_3.tif', 'Blue'), ('IMG_0298_4.tif', 'NIR'), ('IMG_0298_5.tif', 'Rededge'), 
                                 ('IMG_0299_2.tif', 'Green'), ('IMG_0299_3.tif', 'Blue'), ('IMG_0299_4.tif', 'NIR'), ('IMG_0299_5.tif', 'Rededge'), 
                                 ('IMG_0300_1.tif', 'Red'), ('IMG_0300_2.tif', 'Green'), ('IMG_0300_3.tif', 'Blue'), ('IMG_0300_4.tif', 'NIR'), ('IMG_0300_5.tif', 'Rededge')]
        photos = [ODMPhotoMock(f, b) for f,b in micasa_redsense_files]
        self.assertRaises(RuntimeError, types.ODM_Reconstruction, photos)

        # Single camera
        dji_files = ['DJI_0018.JPG','DJI_0019.JPG','DJI_0020.JPG','DJI_0021.JPG','DJI_0022.JPG','DJI_0023.JPG']
        photos = [ODMPhotoMock(f, 'RGB') for f in dji_files]
        recon = types.ODM_Reconstruction(photos)
        self.assertTrue(recon.multi_camera is None)

if __name__ == '__main__':
    unittest.main()