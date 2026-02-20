"""
=============================================================================
RC BOAT MAIN CONTROLLER - code.py
=============================================================================
This is the main file that runs on boot. It imports and uses all subsystems:

    GPS.py    -> GPSReader       - Reads position, speed
    IMU.py    -> IMUReader       - Reads heading, pitch, roll
    radar.py  -> RadarReader     - Detects obstacles
    LORA.py   -> LoraTransceiver - Communicates with GUI
    motors.py -> MotorController - Controls rudder and throttle

Data Flow:
----------
    GPS ──────┐
              │
    IMU ──────┤
              │
    Radar ────┼──► code.py ──► LORA ──► GUI (Computer)
              │       │
    Battery ──┘       │
                      ▼
              ┌───────────────┐
    GUI ──► LORA ──► code.py ──► Motors (Rudder, Throttle)
              └───────────────┘

=============================================================================
"""

import time
import analogio
import board

# Import our modules
from GPS import GPSReader
from radar import RadarReader
from LORA import LoraTransceiver
from motors import MotorController
from IMU import IMUReader

# =============================================================================
# CONFIGURATION
# =============================================================================

# Battery monitoring (ADC)
BATTERY_PIN = board.GP26      # ADC pin for voltage divider
BATTERY_VREF = 3.3            # Pico ADC reference voltage
BATTERY_DIVIDER = 4.0         # Voltage divider ratio (e.g., 4:1)
BATTERY_FULL = 12.6           # Full voltage (3S LiPo)
BATTERY_EMPTY = 10.5          # Empty voltage

# Timing
TELEMETRY_RATE_MS = 100       # Send telemetry every 100ms (10 Hz)
FAILSAFE_TIMEOUT_S = 0.6      # Stop if no command for 0.6 seconds

# =============================================================================
# BATTERY MONITOR
# =============================================================================
class BatteryMonitor:
    """Simple battery voltage monitor via ADC."""
    
    def __init__(self, pin, vref=3.3, divider=4.0, v_full=12.6, v_empty=10.5):
        self.adc = analogio.AnalogIn(pin)
        self.vref = vref
        self.divider = divider
        self.v_full = v_full
        self.v_empty = v_empty
    
    def get_percentage(self):
        """Get battery percentage (0-100)."""
        adc_voltage = (self.adc.value / 65535) * self.vref
        voltage = adc_voltage * self.divider
        pct = (voltage - self.v_empty) / (self.v_full - self.v_empty) * 100
        return max(0, min(100, pct))

# =============================================================================
# MAIN CONTROLLER
# =============================================================================
def main():
    print("=" * 50)
    print("RC BOAT CONTROLLER")
    print("=" * 50)
    
    # --- Initialize all subsystems ---
    print("\nInitializing subsystems...")
    
    try:
        gps = GPSReader()
    except Exception as e:
        print("GPS FAILED:", e)
        gps = None
    
    try:
        radar = RadarReader()
    except Exception as e:
        print("Radar FAILED:", e)
        radar = None
    
    try:
        lora = LoraTransceiver()
    except Exception as e:
        print("LORA FAILED:", e)
        lora = None
    
    try:
        motors = MotorController()
    except Exception as e:
        print("Motors FAILED:", e)
        motors = None
    
    try:
        imu = IMUReader()
    except Exception as e:
        print("IMU FAILED:", e)
        imu = None
    
    try:
        battery = BatteryMonitor(BATTERY_PIN, BATTERY_VREF, BATTERY_DIVIDER, 
                                  BATTERY_FULL, BATTERY_EMPTY)
        print("Battery: Initialized")
    except Exception as e:
        print("Battery FAILED:", e)
        battery = None
    
    print("\n" + "=" * 50)
    print("Starting main loop...")
    print("=" * 50 + "\n")
    
    # --- Timing variables ---
    last_telemetry = 0
    last_command = time.monotonic()
    
    # --- Main loop ---
    while True:
        now = time.monotonic()
        
        # ----- UPDATE SENSORS -----
        if gps:
            gps.update()
        
        if radar:
            radar.update()
        
        if imu:
            imu.update()
        
        # ----- RECEIVE COMMANDS -----
        if lora:
            cmd = lora.receive_command(timeout=0.02)
            if cmd:
                last_command = now
                cmd_type = cmd.get("type", "")
                
                if cmd_type == "ping":
                    # Respond to ping with pong (handshake)
                    lora.send_pong()
                    print("CMD: PING -> PONG sent")
                
                elif cmd_type == "control" and motors:
                    throttle = cmd.get("throttle", 0.0)
                    steering = cmd.get("steering", 0)
                    
                    # Arm on first throttle
                    if throttle > 0:
                        motors.arm()
                    
                    motors.set_controls(throttle, steering)
                    print("CMD: thr=%.2f steer=%d" % (throttle, steering))
                
                elif cmd_type == "stop" and motors:
                    motors.stop()
                    print("CMD: STOP")
        
        # ----- FAILSAFE CHECK -----
        if motors and (now - last_command) > FAILSAFE_TIMEOUT_S:
            motors.stop()
        
        # ----- SEND TELEMETRY -----
        if (now - last_telemetry) * 1000 >= TELEMETRY_RATE_MS:
            last_telemetry = now
            
            # Build telemetry packet
            telemetry = {
                "lat": 0.0,
                "lon": 0.0,
                "hdg": 0.0,
                "spd": 0.0,
                "bat": 100.0,
                "obs": []
            }
            
            # GPS data
            if gps:
                data = gps.get_data()
                if data["lat"] is not None:
                    telemetry["lat"] = round(data["lat"], 6)
                if data["lon"] is not None:
                    telemetry["lon"] = round(data["lon"], 6)
                telemetry["spd"] = round(data["spd"], 1)
                # Use GPS heading as fallback
                telemetry["hdg"] = round(data["hdg"], 1)
            
            # IMU data (more accurate heading than GPS at low speeds)
            if imu:
                telemetry["hdg"] = round(imu.get_heading(), 1)
            
            # Battery
            if battery:
                telemetry["bat"] = round(battery.get_percentage(), 0)
            
            # Radar obstacles
            if radar:
                telemetry["obs"] = radar.get_obstacles()
            
            # Send via LORA
            if lora:
                lora.send_telemetry(telemetry)
            
            # Debug print
            fix = "FIX" if (gps and gps.has_fix) else "NO FIX"
            print("TX: lat=%.5f lon=%.5f hdg=%.0f spd=%.1f bat=%.0f%% [%s]" % (
                telemetry["lat"], telemetry["lon"], telemetry["hdg"],
                telemetry["spd"], telemetry["bat"], fix))
        
        # Small delay
        time.sleep(0.01)

# =============================================================================
# RUN
# =============================================================================
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nShutdown...")
