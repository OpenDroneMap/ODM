from datetime import datetime
from opendm import location, log
import re


def match_single(regexes, line, dtype=int):
    if isinstance(regexes, str):
        regexes = [(regexes, dtype)]
    
    for i in range(len(regexes)):
        if isinstance(regexes[i], str):
            regexes[i] = (regexes[i], dtype)
    
    try:
        for r, transform in regexes:
            match = re.search(r, line)
            if match:
                res = match.group(1)
                return transform(res)
    except Exception as e:
        log.ODM_WARNING("Cannot parse SRT line \"%s\": %s", (line, str(e)))

    return None

class SrtFileParser:
    def __init__(self, filename):
        self.filename = filename
        self.data = []
        self.gps_data = []
        self.ll_to_utm = None
        self.utm_to_ll = None

    def get_entry(self, timestamp: datetime):
        if not self.data:
            self.parse()

        # check min and max
        if timestamp < self.data[0]["start"] or timestamp > self.data[len(self.data) - 1]["end"]:
            return None

        for entry in self.data:
            if entry["start"] <= timestamp and entry["end"] >= timestamp:
                return entry

        return None

    def get_gps(self, timestamp):
        if not self.data:
            self.parse()
        
        # Initialize on first call
        prev_coords = None

        if not self.gps_data:
            for d in self.data:
                lat, lon, alt = d.get('latitude'), d.get('longitude'), d.get('altitude')
                if alt is None:
                    alt = 0
                tm = d.get('start')
                
                if lat is not None and lon is not None:
                    if self.ll_to_utm is None:
                        self.ll_to_utm, self.utm_to_ll = location.utm_transformers_from_ll(lon, lat)

                    coords = self.ll_to_utm.TransformPoint(lon, lat, alt)

                    # First or new (in X/Y only)
                    add = (not len(self.gps_data)) or (coords[0], coords[1]) != (self.gps_data[-1][1][0], self.gps_data[-1][1][1])
                    if add:
                        self.gps_data.append((tm, coords))
        
        # No data available
        if not len(self.gps_data) or self.gps_data[0][0] > timestamp:
            return None

        # Interpolate
        start = None
        for i in range(len(self.gps_data)):
            tm, coords = self.gps_data[i]

            # Perfect match
            if timestamp == tm:
                return self.utm_to_ll.TransformPoint(*coords)

            elif tm > timestamp:
                end = i
                start = i - 1
                if start < 0:
                    return None

                gd_s = self.gps_data[start]
                gd_e = self.gps_data[end]
                sx, sy, sz = gd_s[1]
                ex, ey, ez = gd_e[1]
                
                dt = (gd_e[0] - gd_s[0]).total_seconds()
                if dt >= 10:
                    return None

                dx = (ex - sx) / dt
                dy = (ey - sy) / dt
                dz = (ez - sz) / dt
                t = (timestamp - gd_s[0]).total_seconds()

                return self.utm_to_ll.TransformPoint(
                    sx + dx * t,
                    sy + dy * t,
                    sz + dz * t
                )

    def parse(self):

        # SRT metadata is not standarized, we support the following formats:

        # DJI mavic air 2
        # 1
        # 00:00:00,000 --> 00:00:00,016
        # <font size="36">SrtCnt : 1, DiffTime : 16ms
        # 2023-01-06 18:56:48,380,821
        # [iso : 3200] [shutter : 1/60.0] [fnum : 280] [ev : 0] [ct : 3925] [color_md : default] [focal_len : 240] [latitude: 0.000000] [longitude: 0.000000] [altitude: 0.000000] </font>

        # DJI Mavic Mini
        # 1
        # 00:00:00,000 --> 00:00:01,000
        # F/2.8, SS 206.14, ISO 150, EV 0, GPS (-82.6669, 27.7716, 10), D 2.80m, H 0.00m, H.S 0.00m/s, V.S 0.00m/s 

        # DJI Phantom4 RTK
        # 36
        # 00:00:35,000 --> 00:00:36,000
        # F/6.3, SS 60, ISO 100, EV 0, RTK (120.083799, 30.213635, 28), HOME (120.084146, 30.214243, 103.55m), D 75.36m, H 76.19m, H.S 0.30m/s, V.S 0.00m/s, F.PRY (-5.3°, 2.1°, 28.3°), G.PRY (-40.0°, 0.0°, 28.2°)

        # DJI Unknown Model #1
        # 1
        # 00:00:00,000 --> 00:00:00,033
        # <font size="28">SrtCnt : 1, DiffTime : 33ms
        # 2024-01-18 10:23:26.397
        # [iso : 150] [shutter : 1/5000.0] [fnum : 170] [ev : 0] [ct : 5023] [color_md : default] [focal_len : 240] [dzoom_ratio: 10000, delta:0],[latitude: -22.724555] [longitude: -47.602414] [rel_alt: 0.300 abs_alt: 549.679] </font>

        # DJI Mavic 2 Zoom
        # 1
        # 00:00:00,000 --> 00:00:00,041
        # <font size="36">FrameCnt : 1, DiffTime : 41ms
        # 2023-07-15 11:55:16,320,933
        # [iso : 100] [shutter : 1/400.0] [fnum : 280] [ev : 0] [ct : 5818] [color_md : default] [focal_len : 240] [latitude : 0.000000] [longtitude : 0.000000] [altitude: 0.000000] </font>

        with open(self.filename, 'r') as f:

            iso = None
            shutter = None
            fnum = None
            focal_len = None
            latitude = None
            longitude = None
            altitude = None
            start = None
            end = None

            for line in f:

                # Check if line is empty
                if not line.strip():
                    if start is not None:
                        self.data.append({
                            "start": start,
                            "end": end,
                            "iso": iso,
                            "shutter": shutter,
                            "fnum": fnum,
                            "focal_len": focal_len,
                            "latitude": latitude,
                            "longitude": longitude,
                            "altitude": altitude
                        })

                    iso = None
                    shutter = None
                    fnum = None
                    ct = None
                    focal_len = None
                    latitude = None
                    longitude = None
                    altitude = None
                    start = None
                    end = None

                    continue

                # Remove html tags
                line = re.sub('<[^<]+?>', '', line)

                # Search this "00:00:00,000 --> 00:00:00,016"
                match = re.search("(\d{2}:\d{2}:\d{2},\d+) --> (\d{2}:\d{2}:\d{2},\d+)", line)
                if match:
                    start = datetime.strptime(match.group(1), "%H:%M:%S,%f")
                    end = datetime.strptime(match.group(2), "%H:%M:%S,%f")

                iso = match_single([
                    "iso : (\d+)",
                    "ISO (\d+)"
                ], line)

                shutter = match_single([
                    "shutter : \d+/(\d+\.?\d*)"
                    "SS (\d+\.?\d*)"
                ], line)

                fnum = match_single([
                    ("fnum : (\d+)", lambda v: float(v)/100.0),
                    ("F/([\d\.]+)", float),
                ], line)

                focal_len = match_single("focal_len : (\d+)", line)

                latitude = match_single([
                    ("latitude: ([\d\.\-]+)", lambda v: float(v) if v != 0 else None),
                    ("latitude : ([\d\.\-]+)", lambda v: float(v) if v != 0 else None),
                    ("GPS \([\d\.\-]+,? ([\d\.\-]+),? [\d\.\-]+\)", lambda v: float(v) if v != 0 else None),
                    ("RTK \([-+]?\d+\.\d+, (-?\d+\.\d+), -?\d+\)", lambda v: float(v) if v != 0 else None),
                ], line)

                longitude = match_single([
                    ("longitude: ([\d\.\-]+)", lambda v: float(v) if v != 0 else None),
                    ("longtitude : ([\d\.\-]+)", lambda v: float(v) if v != 0 else None),
                    ("GPS \(([\d\.\-]+),? [\d\.\-]+,? [\d\.\-]+\)", lambda v: float(v) if v != 0 else None),
                    ("RTK \((-?\d+\.\d+), [-+]?\d+\.\d+, -?\d+\)", lambda v: float(v) if v != 0 else None),
                ], line)

                altitude = match_single([
                    ("altitude: ([\d\.\-]+)", lambda v: float(v) if v != 0 else None),
                    ("GPS \([\d\.\-]+,? [\d\.\-]+,? ([\d\.\-]+)\)", lambda v: float(v) if v != 0 else None),
                    ("RTK \([-+]?\d+\.\d+, [-+]?\d+\.\d+, (-?\d+)\)", lambda v: float(v) if v != 0 else None),
                    ("abs_alt: ([\d\.\-]+)", lambda v: float(v) if v != 0 else None),
                ], line)