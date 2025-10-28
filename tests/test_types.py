import unittest
from opendm import types

class ODMPhotoMock:
    def __init__(self, filename, band_name, band_index):
        self.filename = filename
        self.band_name = band_name
        self.band_index = band_index
    
    def __str__(self):
        return "%s (%s)" % (self.filename, self.band_name)

    def __repr__(self):
        return self.__str__()
    
class TestTypes(unittest.TestCase):
    def setUp(self):
        pass

    def test_reconstruction(self):
        # Multi camera setup
        micasa_redsense_files = [('IMG_0298_1.tif', 'Red', 1), ('IMG_0298_2.tif', 'Green', 2), ('IMG_0298_3.tif', 'Blue', 3), ('IMG_0298_4.tif', 'NIR', 4), ('IMG_0298_5.tif', 'Rededge', 5), 
                                 ('IMG_0299_1.tif', 'Red', 1), ('IMG_0299_2.tif', 'Green', 2), ('IMG_0299_3.tif', 'Blue', 3), ('IMG_0299_4.tif', 'NIR', 4), ('IMG_0299_5.tif', 'Rededge', 5), 
                                 ('IMG_0300_1.tif', 'Red', 1), ('IMG_0300_2.tif', 'Green', 2), ('IMG_0300_3.tif', 'Blue', 3), ('IMG_0300_4.tif', 'NIR', 4), ('IMG_0300_5.tif', 'Rededge', 5)]
        photos = [ODMPhotoMock(f, b, i) for f, b, i in micasa_redsense_files]
        recon = types.ODM_Reconstruction(photos)

        self.assertTrue(recon.multi_camera is not None)

        # Found all 5 bands
        bands = ["Red", "Green", "Blue", "NIR", "Rededge"]
        for i in range(len(bands)):
            self.assertEqual(bands[i], recon.multi_camera[i]['name'])
        self.assertTrue([p.filename for p in recon.multi_camera[0]['photos']] == ['IMG_0298_1.tif', 'IMG_0299_1.tif', 'IMG_0300_1.tif'])

        # Single camera
        dji_files = ['DJI_0018.JPG','DJI_0019.JPG','DJI_0020.JPG','DJI_0021.JPG','DJI_0022.JPG','DJI_0023.JPG']
        photos = [ODMPhotoMock(f, 'RGB', 0) for f in dji_files]
        recon = types.ODM_Reconstruction(photos)
        self.assertTrue(recon.multi_camera is None)

if __name__ == '__main__':
    unittest.main()