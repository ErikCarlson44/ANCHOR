import time
import threading
import os
import csv
import datetime
from dataclasses import dataclass
from typing import Optional

import serial
import pynmea2

# -----------------------------
# Settings
# -----------------------------
PORT = "/dev/serial0"      # Try "/dev/ttyACM0" or "/dev/ttyUSB0" if using USB GPS
BAUD = 9600
PRINT_SECONDS = 1.0

LOG_TO_CSV = True
LOG_DIR = "gps_logs"       # folder created next to your script


# -----------------------------
# Data Container
# -----------------------------
@dataclass
class GPSFix:
    utc_time: str = ""
    lat: Optional[float] = None
    lon: Optional[float] = None
    alt_m: Optional[float] = None
    sats: Optional[int] = None
    speed_knots: Optional[float] = None
    speed_mps: Optional[float] = None
    fix_ok: bool = False
    last_sent: str = ""


class GPSReader:
    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.fix = GPSFix()
        self._lock = threading.Lock()
        self._stop = False
        self._thread = None
        self._ser = None

    def start(self):
        self._ser = serial.Serial(self.port, self.baud, timeout=1)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop = True
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass

    def get_fix(self) -> GPSFix:
        with self._lock:
            return GPSFix(**self.fix.__dict__)

    def _update_from_gga(self, msg):
        # GGA: fix quality, sats, altitude, lat/lon
        lat = msg.latitude if hasattr(msg, "latitude") else None
        lon = msg.longitude if hasattr(msg, "longitude") else None

        sats = int(msg.num_sats) if getattr(msg, "num_sats", None) not in (None, "") else None
        alt = float(msg.altitude) if getattr(msg, "altitude", None) not in (None, "") else None
        fix_quality = int(msg.gps_qual) if getattr(msg, "gps_qual", None) not in (None, "") else 0

        fix_ok = (fix_quality > 0) and (lat is not None) and (lon is not None)

        with self._lock:
            self.fix.lat = lat
            self.fix.lon = lon
            self.fix.sats = sats
            self.fix.alt_m = alt
            self.fix.fix_ok = fix_ok
            self.fix.last_sent = "GGA"

    def _update_from_rmc(self, msg):
        # RMC: status A=valid, lat/lon, speed
        status = getattr(msg, "status", "")
        lat = msg.latitude if hasattr(msg, "latitude") else None
        lon = msg.longitude if hasattr(msg, "longitude") else None

        spd_knots = float(msg.spd_over_grnd) if getattr(msg, "spd_over_grnd", None) not in (None, "") else None
        spd_mps = (spd_knots * 0.514444) if spd_knots is not None else None

        fix_ok = (status == "A") and (lat is not None) and (lon is not None)

        utc_time = ""
        try:
            if msg.timestamp:
                utc_time = msg.timestamp.isoformat()
        except Exception:
            pass

        with self._lock:
            self.fix.utc_time = utc_time
            self.fix.lat = lat
            self.fix.lon = lon
            self.fix.speed_knots = spd_knots
            self.fix.speed_mps = spd_mps
            self.fix.fix_ok = self.fix.fix_ok or fix_ok
            self.fix.last_sent = "RMC"

    def _run(self):
        while not self._stop:
            try:
                line = self._ser.readline().decode("ascii", errors="ignore").strip()
                if not line.startswith("$"):
                    continue

                msg = pynmea2.parse(line)

                if msg.sentence_type == "GGA":
                    self._update_from_gga(msg)
                elif msg.sentence_type == "RMC":
                    self._update_from_rmc(msg)

            except pynmea2.ParseError:
                continue
            except Exception:
                time.sleep(0.2)


def main():
    gps = GPSReader(PORT, BAUD)
    gps.start()

    csv_file = None
    writer = None
    csv_path = None

    if LOG_TO_CSV:
        os.makedirs(LOG_DIR, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(LOG_DIR, f"gps_log_{ts}.csv")

        csv_file = open(csv_path, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow(["local_time", "utc_time", "lat", "lon", "alt_m", "sats", "speed_mps", "fix_ok", "last_sent"])
        csv_file.flush()

        print(f"Logging GPS to: {csv_path}")

    try:
        while True:
            f = gps.get_fix()

            if f.fix_ok:
                print(
                    f"FIX lat={f.lat:.6f}, lon={f.lon:.6f}"
                    + (f", alt={f.alt_m:.1f}m" if f.alt_m is not None else "")
                    + (f", sats={f.sats}" if f.sats is not None else "")
                    + (f", speed={f.speed_mps:.2f} m/s" if f.speed_mps is not None else "")
                    + f" ({f.last_sent})"
                )
            else:
                print("NO FIX (waiting for satellites...)")

            if LOG_TO_CSV and writer:
                writer.writerow([
                    datetime.datetime.now().isoformat(timespec="seconds"),
                    f.utc_time,
                    f.lat,
                    f.lon,
                    f.alt_m,
                    f.sats,
                    f.speed_mps,
                    f.fix_ok,
                    f.last_sent
                ])
                csv_file.flush()

            time.sleep(PRINT_SECONDS)

    finally:
        gps.stop()
        if csv_file:
            csv_file.close()
        print("Stopped.")


if __name__ == "__main__":
    main()
