import unittest

from opendm.video.srtparser import SrtFileParser, match_single

# A verbatim excerpt of a real recording, kept as a file so that reading one is
# covered too: https://github.com/OpenDroneMap/drone_dataset_dji_video
EXCERPT = "tests/assets/dji_0317_excerpt.SRT"


def parse_blocks(*blocks):
    parser = SrtFileParser()
    lines = []
    for block in blocks:
        lines.extend(block)
        # a block is only committed when the parser reaches the blank line ending it
        lines.append("")
    parser.parse_lines(lines)
    return parser


def parse_block(*lines):
    return parse_blocks(lines).data[0]


class TestSrtFormats(unittest.TestCase):
    """SRT metadata is not standardised. One sample of every layout we parse."""

    def test_dji_mavic_air_2(self):
        entry = parse_block(
            "1",
            "00:00:00,000 --> 00:00:00,016",
            '<font size="36">SrtCnt : 1, DiffTime : 16ms',
            "2023-01-06 18:56:48,380,821",
            "[iso : 3200] [shutter : 1/60.0] [fnum : 280] [ev : 0] [ct : 3925] [color_md : default] [focal_len : 240] [latitude: 0.000000] [longitude: 0.000000] [altitude: 0.000000] </font>",
            "</font>",
        )
        self.assertEqual(entry["iso"], 3200)
        self.assertEqual(entry["shutter"], 60.0)
        self.assertEqual(entry["fnum"], 2.8)
        self.assertEqual(entry["focal_len"], 240)
        self.assertIsNone(entry["latitude"])
        self.assertIsNone(entry["longitude"])
        self.assertIsNone(entry["altitude"])

    def test_dji_mavic_mini(self):
        entry = parse_block(
            "1",
            "00:00:00,000 --> 00:00:01,000",
            "F/2.8, SS 206.14, ISO 150, EV 0, GPS (-82.6669, 27.7716, 10), D 2.80m, H 0.00m, H.S 0.00m/s, V.S 0.00m/s",
        )
        self.assertEqual(entry["iso"], 150)
        self.assertEqual(entry["shutter"], 206.14)
        self.assertEqual(entry["fnum"], 2.8)
        self.assertIsNone(entry["focal_len"])
        self.assertEqual(entry["latitude"], 27.7716)
        self.assertEqual(entry["longitude"], -82.6669)
        self.assertEqual(entry["altitude"], 10.0)

    def test_dji_phantom4_rtk(self):
        entry = parse_block(
            "36",
            "00:00:35,000 --> 00:00:36,000",
            "F/6.3, SS 60, ISO 100, EV 0, RTK (120.083799, 30.213635, 28), HOME (120.084146, 30.214243, 103.55m), D 75.36m, H 76.19m, H.S 0.30m/s, V.S 0.00m/s, F.PRY (-5.3°, 2.1°, 28.3°), G.PRY (-40.0°, 0.0°, 28.2°)",
        )
        self.assertEqual(entry["iso"], 100)
        self.assertEqual(entry["shutter"], 60.0)
        self.assertEqual(entry["fnum"], 6.3)
        self.assertIsNone(entry["focal_len"])
        self.assertEqual(entry["latitude"], 30.213635)
        self.assertEqual(entry["longitude"], 120.083799)
        self.assertEqual(entry["altitude"], 28.0)

    def test_dji_rtk_decimal_altitude(self):
        # RTK receivers report altitude with decimals; an integer-only pattern
        # dropped the altitude and, with it, the latitude and longitude
        entry = parse_block(
            "36",
            "00:00:35,000 --> 00:00:36,000",
            "F/6.3, SS 60, ISO 100, EV 0, RTK (120.083799, 30.213635, 192.048), HOME (120.084146, 30.214243, 103.55m), D 75.36m, H 76.19m, H.S 0.30m/s, V.S 0.00m/s",
        )
        self.assertEqual(entry["latitude"], 30.213635)
        self.assertEqual(entry["longitude"], 120.083799)
        self.assertEqual(entry["altitude"], 192.048)

    def test_dji_rtk_negative_altitude(self):
        entry = parse_block(
            "1",
            "00:00:00,000 --> 00:00:01,000",
            "F/6.3, SS 60, ISO 100, EV 0, RTK (4.890444, 52.370216, -3.215), HOME (4.890000, 52.370000, 0.00m)",
        )
        self.assertEqual(entry["latitude"], 52.370216)
        self.assertEqual(entry["longitude"], 4.890444)
        self.assertEqual(entry["altitude"], -3.215)

    def test_dji_unknown_model_1(self):
        entry = parse_block(
            "1",
            "00:00:00,000 --> 00:00:00,033",
            '<font size="28">SrtCnt : 1, DiffTime : 33ms',
            "2024-01-18 10:23:26.397",
            "[iso : 150] [shutter : 1/5000.0] [fnum : 170] [ev : 0] [ct : 5023] [color_md : default] [focal_len : 240] [dzoom_ratio: 10000, delta:0],[latitude: -22.724555] [longitude: -47.602414] [rel_alt: 0.300 abs_alt: 549.679] </font>",
        )
        self.assertEqual(entry["iso"], 150)
        self.assertEqual(entry["shutter"], 5000.0)
        self.assertEqual(entry["fnum"], 1.7)
        self.assertEqual(entry["focal_len"], 240)
        self.assertEqual(entry["latitude"], -22.724555)
        self.assertEqual(entry["longitude"], -47.602414)
        self.assertEqual(entry["altitude"], 549.679)

    def test_dji_mavic_2_zoom(self):
        entry = parse_block(
            "1",
            "00:00:00,000 --> 00:00:00,041",
            '<font size="36">FrameCnt : 1, DiffTime : 41ms',
            "2023-07-15 11:55:16,320,933",
            "[iso : 100] [shutter : 1/400.0] [fnum : 280] [ev : 0] [ct : 5818] [color_md : default] [focal_len : 240] [latitude : 0.000000] [longtitude : 0.000000] [altitude: 0.000000] </font>",
        )
        self.assertEqual(entry["iso"], 100)
        self.assertEqual(entry["shutter"], 400.0)
        self.assertEqual(entry["fnum"], 2.8)
        self.assertEqual(entry["focal_len"], 240)
        self.assertIsNone(entry["latitude"])
        self.assertIsNone(entry["longitude"])
        self.assertIsNone(entry["altitude"])

    def test_dji_unknown_model_2(self):
        # kept verbatim: the noise around the fields, such as FaceDetectRect
        # (0,0,0,0,), is what the coordinate patterns have to avoid matching
        entry = parse_block(
            "1",
            "00:00:00,000 --> 00:00:00,033",
            "No:1, F/2.8, SS 155.55, ISO 100, EV 0, M.M AE_METER_CENTER, A.T (126,109), Luma 106, Coef(1.000000, 1.000000, 1.000000), FaceDetectTag (0), FaceDetectRect (0,0,0,0,), Gain (1.000000,4096), Index (Ev:10085,Nf:0), E.M 0, AERect(n/a), AeAdvScene (GR:91.000000,GWR:1.000000,LLR:0.196683,RR:0.870551), AfSpd 0/0, Af Rect(X:0, Y:0, W:0, H:0), AfPos 0, AwbMode WB_AUTOMATIC, Awb Gain(R:8206, G:4096, B:7058), ColorTemp 5241, B.L (-1020, -1020, -1020, -1020), IQS (39253, 208), Isp Info (PIPE 1,ADJ 0,De 0) GPS (-2.5927, 52.0035, 15), D 0.61m, H 1.00m, H.S 0.00m/s, V.S 0.00m/s",
        )
        self.assertEqual(entry["iso"], 100)
        self.assertEqual(entry["shutter"], 155.55)
        self.assertEqual(entry["fnum"], 2.8)
        self.assertIsNone(entry["focal_len"])
        self.assertEqual(entry["latitude"], 52.0035)
        self.assertEqual(entry["longitude"], -2.5927)
        self.assertEqual(entry["altitude"], 15.0)

    def test_dji_unknown_model_3(self):
        entry = parse_block(
            "1",
            "00:00:00,000 --> 00:00:00,016",
            '<font size="36">SrtCnt : 1, DiffTime : 16ms',
            "2023-01-11 17:01:55,343,173",
            "[iso : 3020] [shutter : 1/60.0] [fnum : 280] [ev : 0] [ct : 8366] [color_md : default] [focal_len : 240] [dzoom_ratio: 10000, delta:0],[latitude: 45.499044] [longitude: 9.040211] [altitude: 29.600000] </font>",
        )
        self.assertEqual(entry["iso"], 3020)
        self.assertEqual(entry["shutter"], 60.0)
        self.assertEqual(entry["fnum"], 2.8)
        self.assertEqual(entry["focal_len"], 240)
        self.assertEqual(entry["latitude"], 45.499044)
        self.assertEqual(entry["longitude"], 9.040211)
        self.assertEqual(entry["altitude"], 29.6)


def gps_block(index, start, end, latitude, longitude, altitude):
    return (
        str(index),
        "%s --> %s" % (start, end),
        '<font size="36">SrtCnt : %d, DiffTime : 16ms' % index,
        "2023-01-11 17:01:55,343,173",
        "[iso : 3020] [shutter : 1/60.0] [fnum : 280] [focal_len : 240] "
        "[latitude: %s] [longitude: %s] [altitude: %s] </font>" % (latitude, longitude, altitude),
    )


NO_FIX = gps_block(1, "00:00:00,000", "00:00:00,016", "0.000000", "0.000000", "0.000000")
FIRST_FIX = gps_block(2, "00:00:00,033", "00:00:00,050", "45.499044", "9.040211", "29.600000")
SECOND_FIX = gps_block(3, "00:00:00,066", "00:00:00,083", "45.499045", "9.040212", "29.700000")


class TestSrtGps(unittest.TestCase):
    def test_coordinates_are_none_before_gps_lock(self):
        entry = parse_blocks(NO_FIX, FIRST_FIX).data[0]
        self.assertIsNone(entry["latitude"])
        self.assertIsNone(entry["longitude"])
        self.assertIsNone(entry["altitude"])

    def test_no_gps_before_first_lock(self):
        parser = parse_blocks(NO_FIX, FIRST_FIX, SECOND_FIX)
        self.assertIsNone(parser.get_gps(parser.data[0]["start"]))

    def test_gps_after_lock_is_not_null_island(self):
        parser = parse_blocks(NO_FIX, FIRST_FIX, SECOND_FIX)
        lon, lat, alt = parser.get_gps(parser.data[1]["start"])
        self.assertAlmostEqual(lat, 45.499044, places=5)
        self.assertAlmostEqual(lon, 9.040211, places=5)

    def test_interpolates_between_fixes(self):
        parser = SrtFileParser(EXCERPT)
        parser.parse()
        self.assertEqual(len(parser.data), 5)
        lon, lat, alt = parser.get_gps(parser.data[1]["start"])
        self.assertGreater(lat, 45.499044)
        self.assertLess(lat, 45.499045)

    def test_unparseable_value_is_skipped(self):
        self.assertIsNone(match_single(r"iso : (\w+)", "[iso : abc]"))


if __name__ == "__main__":
    unittest.main()
