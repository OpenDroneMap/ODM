import datetime
import unittest

import piexif

from opendm.video.video2dataset import get_gps_location

TIME = datetime.datetime(2026, 1, 20, 12, 0, 0)


def rational(r):
    return r[0] / r[1]


def dump_and_load(gps_ifd):
    # Round-trip through piexif, as Video2Dataset does when saving a frame
    return piexif.load(piexif.dump({"GPS": gps_ifd}))["GPS"]


def dms(r):
    d, m, s = r
    return rational(d) + rational(m) / 60.0 + rational(s) / 3600.0


class TestGpsLocation(unittest.TestCase):
    def test_altitude_keeps_millimeters(self):
        gps = dump_and_load(get_gps_location(TIME, 30.213635, 120.083799, 192.048))
        self.assertEqual(gps[piexif.GPSIFD.GPSAltitudeRef], 0)
        self.assertAlmostEqual(rational(gps[piexif.GPSIFD.GPSAltitude]), 192.048, places=6)

    def test_altitude_rounded_to_millimeters(self):
        gps = get_gps_location(TIME, 30.213635, 120.083799, 192.04849999)
        self.assertEqual(gps[piexif.GPSIFD.GPSAltitude], (24006, 125))

    def test_negative_altitude(self):
        gps = dump_and_load(get_gps_location(TIME, 52.370216, 4.890444, -3.215))
        self.assertEqual(gps[piexif.GPSIFD.GPSAltitudeRef], 1)
        self.assertAlmostEqual(rational(gps[piexif.GPSIFD.GPSAltitude]), 3.215, places=6)

    def test_coordinates_precision(self):
        lat, lon = -22.72455512, -47.60241478
        gps = dump_and_load(get_gps_location(TIME, lat, lon, 549.679))
        self.assertEqual(gps[piexif.GPSIFD.GPSLatitudeRef], b"S")
        self.assertEqual(gps[piexif.GPSIFD.GPSLongitudeRef], b"W")
        # seconds are kept to 1e-5", i.e. well below a millimeter
        self.assertAlmostEqual(dms(gps[piexif.GPSIFD.GPSLatitude]), abs(lat), places=8)
        self.assertAlmostEqual(dms(gps[piexif.GPSIFD.GPSLongitude]), abs(lon), places=8)


if __name__ == "__main__":
    unittest.main()
