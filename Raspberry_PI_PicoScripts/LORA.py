"""
LORA Module - Handles RFM9x LORA communication with GUI
Import this module and create a LoraTransceiver instance.
"""

import time
import json
import board
import busio
import digitalio

import adafruit_rfm9x

# -----------------------------
# Configuration - CHANGE THESE
# -----------------------------
FREQ_MHZ = 915.0          # 915 MHz for US, 868 MHz for EU
TX_POWER = 13             # Transmit power (5-23 dBm)

SPI_SCK = board.GP18      # SPI Clock
SPI_MOSI = board.GP19     # SPI MOSI  
SPI_MISO = board.GP16     # SPI MISO
SPI_CS = board.GP17       # Chip Select
RESET_PIN = board.GP20    # Reset pin

# -----------------------------
# LORA Transceiver Class
# -----------------------------
class LoraTransceiver:
    """
    Handles LORA communication with the GUI.
    Sends telemetry (JSON) and receives commands (JSON).
    
    Usage:
        lora = LoraTransceiver()
        lora.send_telemetry({"lat": 32.78, "lon": -117.23, ...})
        cmd = lora.receive_command()
    """
    
    def __init__(self, freq=FREQ_MHZ, tx_power=TX_POWER):
        # Setup SPI
        self.spi = busio.SPI(SPI_SCK, MOSI=SPI_MOSI, MISO=SPI_MISO)
        cs = digitalio.DigitalInOut(SPI_CS)
        reset = digitalio.DigitalInOut(RESET_PIN)
        
        # Initialize RFM9x
        self.rfm9x = adafruit_rfm9x.RFM9x(self.spi, cs, reset, freq)
        self.rfm9x.tx_power = tx_power
        
        print("LORA: Initialized at %.1f MHz, TX power %d dBm" % (freq, tx_power))
    
    def send_telemetry(self, telemetry_dict):
        """
        Send telemetry to GUI as JSON.
        
        Args:
            telemetry_dict: Dictionary with keys: lat, lon, hdg, spd, bat, obs
        """
        try:
            json_str = json.dumps(telemetry_dict)
            self.rfm9x.send(bytes(json_str, "utf-8"))
        except Exception as e:
            print("LORA TX Error:", e)
    
    def send_pong(self):
        """
        Send pong response to GUI (handshake acknowledgement).
        Called when boat receives a ping from the GUI.
        """
        try:
            pong_msg = json.dumps({"type": "pong"})
            self.rfm9x.send(bytes(pong_msg, "utf-8"))
        except Exception as e:
            print("LORA Pong Error:", e)
    
    def receive_command(self, timeout=0.05):
        """
        Check for incoming command from GUI.
        
        Returns:
            Dictionary with command data, or None if no command.
            Expected format: {"type": "control", "throttle": 0.5, "steering": -1}
        """
        try:
            packet = self.rfm9x.receive(timeout=timeout)
            if packet is not None:
                json_str = packet.decode("utf-8", "ignore").strip()
                return json.loads(json_str)
        except Exception:
            pass
        return None
    
    def get_rssi(self):
        """Get signal strength of last received packet."""
        return self.rfm9x.last_rssi


# -----------------------------
# Standalone Test
# -----------------------------
if __name__ == "__main__":
    print("LORA Test Mode - Waiting for packets...")
    lora = LoraTransceiver()
    
    while True:
        # Try to receive
        cmd = lora.receive_command(timeout=0.5)
        if cmd:
            print("Received:", cmd, "RSSI:", lora.get_rssi())
        
        # Send test telemetry
        test_telemetry = {
            "lat": 32.7872,
            "lon": -117.2350,
            "hdg": 45.0,
            "spd": 5.0,
            "bat": 85.0,
            "obs": []
        }
        lora.send_telemetry(test_telemetry)
        print("Sent telemetry")
        
        time.sleep(1)
