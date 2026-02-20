"""
Radar Module - Reads obstacle data from RD-03D radar
Import this module and create a RadarReader instance.
"""

import time
import board
import busio

# -----------------------------
# Configuration - CHANGE THESE
# -----------------------------
RADAR_TX_PIN = board.GP0   # Pico RX <- Radar TX
RADAR_RX_PIN = board.GP1   # Pico TX -> Radar RX
RADAR_BAUDRATE = 115200

# -----------------------------
# Radar Reader Class
# -----------------------------
class RadarReader:
    """
    Reads obstacle data from RD-03D radar module.
    
    Usage:
        radar = RadarReader()
        radar.update()  # Call frequently
        obstacles = radar.get_obstacles()  # [[dist, angle, size], ...]
    """
    
    def __init__(self, tx_pin=RADAR_TX_PIN, rx_pin=RADAR_RX_PIN, baudrate=RADAR_BAUDRATE):
        self.uart = busio.UART(tx_pin, rx_pin, baudrate=baudrate, timeout=0.1)
        self.buffer = b""
        self.obstacles = []
        self.max_obstacles = 10
        
        print("Radar: Initialized")
    
    def update(self):
        """Read and parse radar data. Call this frequently."""
        # Read available data
        chunk = self.uart.read(64)
        if chunk:
            self.buffer += chunk
        
        # Clear old obstacles
        self.obstacles = []
        
        # Process complete lines
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            try:
                s = line.decode("ascii", "ignore").strip()
                self._parse_line(s)
            except Exception:
                pass
    
    def _parse_line(self, s):
        """
        Parse a radar output line.
        ADJUST THIS FOR YOUR RADAR'S ACTUAL OUTPUT FORMAT!
        """
        # Example format 1: "D:10.5,A:30"
        if "D:" in s and "A:" in s:
            parts = s.split(",")
            distance = None
            angle = None
            for part in parts:
                if part.startswith("D:"):
                    distance = float(part[2:])
                elif part.startswith("A:"):
                    angle = float(part[2:])
            
            if distance is not None and angle is not None:
                self.obstacles.append([distance, angle, 1.0])
        
        # Example format 2: "TARGET,10.5,30,1.0"
        elif s.startswith("TARGET"):
            parts = s.split(",")
            if len(parts) >= 3:
                distance = float(parts[1])
                angle = float(parts[2])
                size = float(parts[3]) if len(parts) > 3 else 1.0
                self.obstacles.append([distance, angle, size])
        
        # Keep only closest obstacles
        self.obstacles = self.obstacles[:self.max_obstacles]
    
    def get_obstacles(self):
        """Return list of obstacles as [[distance, angle, size], ...]"""
        return self.obstacles
    
    def get_raw_line(self):
        """Get raw line for debugging."""
        if b"\n" in self.buffer:
            line, _ = self.buffer.split(b"\n", 1)
            return line.decode("ascii", "ignore").strip()
        return None


# -----------------------------
# Standalone Test
# -----------------------------
if __name__ == "__main__":
    print("Radar Test Mode - Reading raw data...")
    radar = RadarReader()
    
    while True:
        # Read raw data for debugging
        chunk = radar.uart.read(64)
        if chunk:
            radar.buffer += chunk
            
            while b"\n" in radar.buffer:
                line, radar.buffer = radar.buffer.split(b"\n", 1)
                try:
                    s = line.decode("ascii", "ignore").strip()
                    print("RAW:", s)
                except Exception:
                    pass
        
        time.sleep(0.01)
