from datetime import datetime
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
                match = re.search("(\d{2}:\d{2}:\d{2},\d{3}) --> (\d{2}:\d{2}:\d{2},\d{3})", line)
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
                    ("GPS \(([\d\.\-]+),? [\d\.\-]+,? [\d\.\-]+\)", lambda v: float(v) if v != 0 else None),
                ], line)

                longitude = match_single([
                    ("longitude: ([\d\.\-]+)", lambda v: float(v) if v != 0 else None),
                    ("GPS \(([\d\.\-]+),? [\d\.\-]+,? [\d\.\-]+\)", lambda v: float(v) if v != 0 else None),
                ], line)

                altitude = match_single([
                    ("altitude: ([\d\.\-]+)", lambda v: float(v) if v != 0 else None),
                    ("GPS \([\d\.\-]+,? [\d\.\-]+,? ([\d\.\-]+)\)", lambda v: float(v) if v != 0 else None),
                ], line)