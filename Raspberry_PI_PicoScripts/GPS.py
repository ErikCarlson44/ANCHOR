"""
GPS Module - Parses NMEA data from GPS receiver
Import this module and create a GPSReader instance.
"""

import time
import board
import busio

# -----------------------------
# Configuration - CHANGE THESE
# -----------------------------
GPS_TX_PIN = board.GP4   # Pico RX <- GPS TX
GPS_RX_PIN = board.GP5   # Pico TX -> GPS RX  
GPS_BAUDRATE = 9600

# -----------------------------
# Helper Functions
# -----------------------------
def nmea_deg_to_decimal(raw, hemi):
    """Convert NMEA format (DDMM.MMMM) to decimal degrees."""
    if not raw:
        return None
    v = float(raw)
    deg = int(v // 100)
    minutes = v - deg * 100
    dec = deg + minutes / 60.0
    if hemi in ("S", "W"):
        dec = -dec
    return dec

def parse_gga(s):
    """Parse $GPGGA sentence for position fix data."""
    p = s.split(",")
    if len(p) < 10:
        return None
    fixq = int(p[6] or "0")
    sats = int(p[7] or "0") if p[7] else None
    lat = nmea_deg_to_decimal(p[2], p[3])
    lon = nmea_deg_to_decimal(p[4], p[5])
    alt = float(p[9]) if p[9] else None
    return fixq, sats, lat, lon, alt

def parse_rmc(s):
    """Parse $GPRMC sentence for speed and heading."""
    p = s.split(",")
    if len(p) < 10:
        return None
    status = p[2]
    lat = nmea_deg_to_decimal(p[3], p[4])
    lon = nmea_deg_to_decimal(p[5], p[6])
    spd_knots = float(p[7]) if p[7] else None
    heading = float(p[8]) if p[8] else 0.0
    return status, lat, lon, spd_knots, heading

# -----------------------------
# GPS Reader Class
# -----------------------------
class GPSReader:
    """
    Reads and parses GPS NMEA data.
    
    Usage:
        gps = GPSReader()
        gps.update()  # Call frequently
        data = gps.get_data()  # Get current position
    """
    
    def __init__(self, tx_pin=GPS_TX_PIN, rx_pin=GPS_RX_PIN, baudrate=GPS_BAUDRATE):
        self.uart = busio.UART(tx_pin, rx_pin, baudrate=baudrate, timeout=0.1)
        self.buffer = b""
        
        # Current GPS data
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.speed_knots = 0.0
        self.heading = 0.0
        self.satellites = 0
        self.fix_quality = 0
        self.has_fix = False
        
        print("GPS: Initialized")
    
    def update(self):
        """Read and parse any available GPS data. Call this frequently."""
        # Read available data
        chunk = self.uart.read(64)
        if chunk:
            self.buffer += chunk
        
        # Process complete sentences
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            try:
                sentence = line.decode("ascii", "ignore").strip()
                
                if sentence.startswith("$GNGGA") or sentence.startswith("$GPGGA"):
                    # Remove checksum
                    if "*" in sentence:
                        sentence = sentence.split("*")[0]
                    out = parse_gga(sentence)
                    if out:
                        self.fix_quality, self.satellites, lat, lon, alt = out
                        self.has_fix = self.fix_quality > 0
                        if lat is not None:
                            self.latitude = lat
                        if lon is not None:
                            self.longitude = lon
                        if alt is not None:
                            self.altitude = alt
                
                elif sentence.startswith("$GNRMC") or sentence.startswith("$GPRMC"):
                    if "*" in sentence:
                        sentence = sentence.split("*")[0]
                    out = parse_rmc(sentence)
                    if out:
                        status, lat, lon, spd, hdg = out
                        self.has_fix = self.has_fix or (status == "A")
                        if lat is not None:
                            self.latitude = lat
                        if lon is not None:
                            self.longitude = lon
                        if spd is not None:
                            self.speed_knots = spd
                        if hdg is not None:
                            self.heading = hdg
                            
            except Exception:
                pass
    
    def get_data(self):
        """Return current GPS data as dictionary."""
        return {
            "lat": self.latitude,
            "lon": self.longitude,
            "hdg": self.heading,
            "spd": self.speed_knots,
            "alt": self.altitude,
            "sats": self.satellites,
            "fix": self.has_fix
        }


# -----------------------------
# Standalone Test
# -----------------------------
if __name__ == "__main__":
    print("GPS Test Mode")
    gps = GPSReader()
    
    while True:
        gps.update()
        data = gps.get_data()
        
        if data["fix"] and data["lat"] is not None:
            print("FIX lat=%.6f lon=%.6f hdg=%.1f spd=%.1f sats=%s" % (
                data["lat"], data["lon"], data["hdg"], data["spd"], data["sats"]))
        else:
            print("NO FIX")
        
        time.sleep(1)
