from datetime import datetime
import re
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

            srtcnt = None
            difftime = None
            timestamp = None
            iso = None
            shutter = None
            fnum = None
            ev = None
            ct = None
            color_md = None
            focal_len = None
            latitude = None
            longitude = None
            altitude = None
            start = None
            end = None

            for line in f:

                # Check if line is empty
                if not line.strip():
                    if srtcnt is not None:
                        self.data.append({
                            "start": start,
                            "end": end,
                            "srtcnt": srtcnt,
                            "difftime": difftime,
                            "timestamp": timestamp,
                            "iso": iso,
                            "shutter": shutter,
                            "fnum": fnum,
                            "ev": ev,
                            "ct": ct,
                            "color_md": color_md,
                            "focal_len": focal_len,
                            "latitude": latitude,
                            "longitude": longitude,
                            "altitude": altitude
                        })

                    srtcnt = None
                    difftime = None
                    timestamp = None
                    iso = None
                    shutter = None
                    fnum = None
                    ev = None
                    ct = None
                    color_md = None
                    focal_len = None
                    latitude = None
                    longitude = None
                    altitude = None
                    start = None
                    end = None

                    continue

                # Remove the html font tag
                line = re.sub('<[^<]+?>', '', line)

                # Search this "00:00:00,000 --> 00:00:00,016"
                match = re.search("(\d{2}:\d{2}:\d{2},\d{3}) --> (\d{2}:\d{2}:\d{2},\d{3})", line)
                if match:
                    start = datetime.strptime(match.group(1), "%H:%M:%S,%f")
                    end = datetime.strptime(match.group(2), "%H:%M:%S,%f")


                match = re.search("SrtCnt : (\d+)", line)
                if match:
                    srtcnt = int(match.group(1))

                match = re.search("DiffTime : (\d+)ms", line)
                if match:
                    difftime = int(match.group(1))

                match = re.search("(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3})", line)
                if match:
                    timestamp = match.group(1)
                    timestamp = datetime.strptime(timestamp, "%Y-%m-%d %H:%M:%S,%f")

                match = re.search("iso : (\d+)", line)
                if match:
                    iso = int(match.group(1))

                match = re.search("shutter : (\d+/\d+.\d+)", line)
                if match:
                    shutter = match.group(1)

                match = re.search("fnum : (\d+)", line)
                if match:
                    fnum = int(match.group(1))

                match = re.search("ev : (\d+)", line)
                if match:
                    ev = int(match.group(1))

                match = re.search("ct : (\d+)", line)
                if match:
                    ct = int(match.group(1))

                match = re.search("color_md : (\w+)", line)
                if match:
                    color_md = match.group(1)

                match = re.search("focal_len : (\d+)", line)
                if match:
                    focal_len = int(match.group(1))

                match = re.search("latitude: (\d+.\d+)", line)
                if match:
                    latitude = float(match.group(1))
                    latitude = latitude if latitude != 0 else None

                match = re.search("longitude: (\d+.\d+)", line)
                if match:
                    longitude = float(match.group(1))
                    longitude = longitude if longitude != 0 else None

                match = re.search("altitude: (\d+.\d+)", line)
                if match:
                    altitude = float(match.group(1))
                    altitude = altitude if altitude != 0 else None
