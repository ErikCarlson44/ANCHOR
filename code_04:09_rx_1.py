"""
=============================================================================
ANCHOR — GPS ONLY TEST (NEO-M8N / u-blox UART NMEA)
=============================================================================
Same wiring and parsing as March3Working/code.py. Save as code.py on the Pico
for a bare GPS check, or run from Thonny as gps_pico_test.py.

Wiring (matches code.py):
  GPS TX  -> GP1 (Pico RX — data from module)
  GPS RX  -> GP0 (Pico TX — optional; module may not need for RX-only test)
  GPS VCC -> 3.3V
  GPS GND -> GND

Antenna with sky view; first fix can take several minutes cold start.

If status shows rx_bytes=0 forever: no UART data — swap GPS TX ↔ GP1 / GPS RX ↔ GP0,
check 3.3 V + GND, try DEBUG_NMEA = True. If rx_bytes grows but nmea=0: wrong baud
(try 38400) or electrical noise.
=============================================================================
"""

import board
import busio
import time

# --- Same as code.py ---
GPS_TX_PIN = board.GP0
GPS_RX_PIN = board.GP1
GPS_BAUD = 9600

# Set True to print every raw NMEA line (noisy).
DEBUG_NMEA = False

PRINT_INTERVAL_S = 1.0


def nmea_to_decimal(raw, hemisphere):
    """Convert NMEA coordinate format (DDDMM.MMMM) to decimal degrees."""
    if not raw:
        return None
    try:
        value = float(raw)
        degrees = int(value // 100)
        minutes = value - (degrees * 100)
        decimal = degrees + (minutes / 60.0)
        if hemisphere in ("S", "W"):
            decimal = -decimal
        return decimal
    except Exception:
        return None


def parse_gga(sentence):
    """Parse $GPGGA / $GNGGA for fix quality, sats, lat, lon, alt."""
    parts = sentence.split(",")
    if len(parts) < 10:
        return None
    try:
        fix_quality = int(parts[6]) if parts[6] else 0
        satellites = int(parts[7]) if parts[7] else 0
        latitude = nmea_to_decimal(parts[2], parts[3])
        longitude = nmea_to_decimal(parts[4], parts[5])
        altitude = float(parts[9]) if parts[9] else None
        return (fix_quality, satellites, latitude, longitude, altitude)
    except Exception:
        return None


def parse_rmc(sentence):
    """Parse $GPRMC / $GNRMC for speed and heading."""
    parts = sentence.split(",")
    if len(parts) < 10:
        return None
    try:
        status = parts[2]
        latitude = nmea_to_decimal(parts[3], parts[4])
        longitude = nmea_to_decimal(parts[5], parts[6])
        speed_knots = float(parts[7]) if parts[7] else 0.0
        heading = float(parts[8]) if parts[8] else 0.0
        return (status, latitude, longitude, speed_knots, heading)
    except Exception:
        return None


def parse_gsv(sentence):
    """
    Parse $..GSV — satellites in view (proves RF front-end is working).
    Returns (total_in_view, list of C/N0 dB-Hz values from this line, up to 4).
    """
    parts = sentence.split(",")
    if len(parts) < 4:
        return None
    try:
        total = int(parts[3]) if parts[3] else 0
        snrs = []
        i = 4
        while i + 3 < len(parts):
            snr_field = parts[i + 3]
            if snr_field and snr_field.strip():
                try:
                    v = int(snr_field.split("*")[0])
                    if v > 0:
                        snrs.append(v)
                except ValueError:
                    pass
            i += 4
        return (total, snrs)
    except Exception:
        return None


def parse_gsa(sentence):
    """
    Parse $..GSA — fix dimension: 1=no fix, 2=2D, 3=3D.
    Returns int 1..3 or 0 if unknown.
    """
    parts = sentence.split(",")
    if len(parts) < 3:
        return None
    try:
        mode = parts[2].strip()
        if not mode:
            return 1
        return int(mode)
    except Exception:
        return None


class GPS:
    """NEO-M8N UART reader — logic copied from code.py GPS class."""

    def __init__(self):
        self.uart = busio.UART(GPS_TX_PIN, GPS_RX_PIN, baudrate=GPS_BAUD, timeout=0.1)
        self.buffer = b""
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.speed_knots = 0.0
        self.heading = 0.0
        self.satellites = 0
        self.fix_quality = 0
        self.has_fix = False
        self.rx_bytes_total = 0
        self.nmea_lines_total = 0
        # Acquisition detail (GSV/GSA — not in code.py main GPS, test-only)
        self.sats_in_view = 0
        self.best_snr_db = 0
        self.gsa_mode = 0
        self._cnt = {"GGA": 0, "RMC": 0, "GSV": 0, "GSA": 0, "OTHER": 0}

    def update(self):
        chunk = self.uart.read(64)
        if chunk:
            self.rx_bytes_total += len(chunk)
            self.buffer += chunk

        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            try:
                sentence = line.decode("ascii", "ignore").strip()
                if sentence.startswith("$"):
                    self.nmea_lines_total += 1
                if DEBUG_NMEA and sentence.startswith("$"):
                    print(sentence[:100])

                if "*" in sentence:
                    sentence_clean = sentence.split("*")[0]
                else:
                    sentence_clean = sentence

                kind = sentence_clean[3:6] if len(sentence_clean) >= 6 else ""
                if kind in self._cnt:
                    self._cnt[kind] += 1
                elif sentence_clean.startswith("$") and len(sentence_clean) >= 6:
                    self._cnt["OTHER"] += 1

                sentence = sentence_clean

                if sentence.startswith("$GNGGA") or sentence.startswith("$GPGGA"):
                    result = parse_gga(sentence)
                    if result:
                        self.fix_quality, self.satellites, lat, lon, alt = result
                        self.has_fix = self.fix_quality > 0
                        if lat is not None:
                            self.latitude = lat
                        if lon is not None:
                            self.longitude = lon
                        if alt is not None:
                            self.altitude = alt

                elif sentence.startswith("$GNRMC") or sentence.startswith("$GPRMC"):
                    result = parse_rmc(sentence)
                    if result:
                        status, lat, lon, speed, hdg = result
                        self.has_fix = self.has_fix or (status == "A")
                        if lat is not None:
                            self.latitude = lat
                        if lon is not None:
                            self.longitude = lon
                        if speed is not None:
                            self.speed_knots = speed
                        if hdg is not None:
                            self.heading = hdg

                elif "GSV" == kind:
                    r = parse_gsv(sentence)
                    if r:
                        total, snrs = r
                        self.sats_in_view = total
                        for s in snrs:
                            if s > self.best_snr_db:
                                self.best_snr_db = s

                elif "GSA" == kind:
                    m = parse_gsa(sentence)
                    if m is not None:
                        self.gsa_mode = m
            except Exception:
                pass

    def get_data(self):
        return {
            "lat": self.latitude,
            "lon": self.longitude,
            "alt": self.altitude,
            "hdg": self.heading,
            "spd": self.speed_knots,
            "sats": self.satellites,
            "fix": self.has_fix,
        }

    def gsa_mode_str(self):
        m = self.gsa_mode
        if m == 0:
            return "?"
        if m == 1:
            return "none(1)"
        if m == 2:
            return "2D"
        if m == 3:
            return "3D"
        return str(m)


def main():
    print("")
    print("=" * 52)
    print("  ANCHOR GPS TEST  GP0=TX(to GPS RX)  GP1=RX(from GPS TX)  9600")
    print("=" * 52)
    gps = GPS()
    print("UART OK — waiting for NMEA (use antenna outdoors)...")
    print("Fields: GSV=sat-search telemetry; in_view / best_SNR = from GSV;")
    print("        GSA_mode none(1)/2D/3D = solution type; gga_sats/qual = from GGA when fixing.")
    print("")

    t_start = time.monotonic()
    last_print = t_start
    warned_no_rx = False
    while True:
        gps.update()
        now = time.monotonic()
        if (now - last_print) >= PRINT_INTERVAL_S:
            last_print = now
            d = gps.get_data()
            fix = "FIX" if d["fix"] else "no fix"
            rxb = gps.rx_bytes_total
            nmea = gps.nmea_lines_total
            if rxb == 0:
                human = "STATUS: No data from GPS — check TX->GP1, RX->GP0, 3.3V, GND"
            elif nmea == 0:
                human = "STATUS: UART noise/wrong baud — try GPS_BAUD=38400 or DEBUG_NMEA=True"
            elif d["fix"]:
                human = "STATUS: Working — satellite fix acquired"
            elif gps._cnt["GSV"] == 0:
                human = (
                    "STATUS: NMEA OK but no GSV sentences — chip may be in wrong NMEA config; "
                    "still try clear sky. Enable DEBUG_NMEA to see sentence types."
                )
            elif gps.sats_in_view == 0:
                human = (
                    "STATUS: GSV says 0 satellites in view — antenna/sky blocked or weak; "
                    "receiver is running but not seeing usable signals."
                )
            else:
                human = (
                    "STATUS: Searching/acquiring — GSV reports {} sat(s) above horizon; "
                    "strongest C/N0 ~{} dB-Hz (need fix for lat/lon). Go outdoors / wait.".format(
                        gps.sats_in_view,
                        gps.best_snr_db if gps.best_snr_db > 0 else "?",
                    )
                )

            print(
                "[{}] rx={}B nmea={} | GGA={} RMC={} GSV={} GSA={} | "
                "in_view={} best_SNR={}dB GSA_mode={} | "
                "gga_sats={} qual={} lat={} lon={}".format(
                    fix,
                    rxb,
                    nmea,
                    gps._cnt["GGA"],
                    gps._cnt["RMC"],
                    gps._cnt["GSV"],
                    gps._cnt["GSA"],
                    gps.sats_in_view,
                    gps.best_snr_db if gps.best_snr_db > 0 else "-",
                    gps.gsa_mode_str(),
                    d["sats"],
                    gps.fix_quality,
                    d["lat"],
                    d["lon"],
                )
            )
            print("    " + human)
            if not warned_no_rx and gps.rx_bytes_total == 0 and (now - t_start) >= 5.0:
                warned_no_rx = True
                print("")
                print("  ! No bytes on UART — wiring/power/baud. Swap: GPS TX<->GP1, GPS RX<->GP0")
                print("  ! Or set DEBUG_NMEA = True to see raw traffic.")
                print("")
        time.sleep(0.02)


try:
    main()
except KeyboardInterrupt:
    print("\nStopped.")
