"""
=============================================================================
ANCHOR - RC BOAT MAIN CONTROLLER (USB Serial Version)
=============================================================================
Combines working radar and IMU code to send telemetry to GUI via USB serial.

Working Components:
  - Radar (RD-03D) with servo sweep - from pico_radar_servo.py
  - IMU (ICM-20948) for heading - from icm20948_pico.py

Wiring:
  RADAR:
    Radar TX  -> GP1 (Pico RX)
    Radar RX  -> GP0 (Pico TX)
    Radar VCC -> 5V (VBUS)
    Radar GND -> GND
  
  SERVO:
    Servo Sig -> GP12
    Servo VCC -> 5V
    Servo GND -> GND
  
  IMU (ICM-20948):
    IMU SDA   -> GP20
    IMU SCL   -> GP21
    IMU VCC   -> 3.3V
    IMU GND   -> GND

=============================================================================
"""

import board
import busio
import pwmio
import time
import math
import json
import supervisor
import sys

# =============================================================================
# CONFIGURATION
# =============================================================================

# Radar pins (from pico_radar_servo.py)
RADAR_TX_PIN = board.GP0
RADAR_RX_PIN = board.GP1
RADAR_BAUD = 256000

# Servo pin
SERVO_PIN = board.GP12
SERVO_MIN = 0
SERVO_MAX = 180
SERVO_STEP = 5
SERVO_DELAY = 0.05

# IMU pins (from icm20948_pico.py)
I2C_SDA = board.GP20
I2C_SCL = board.GP21
IMU_ADDRESS = 0x69

# Telemetry rate
TELEMETRY_RATE_HZ = 10

# Placeholder GPS (Mission Bay, San Diego)
DEFAULT_LAT = 32.7872
DEFAULT_LON = -117.2350

# =============================================================================
# SERVO (from pico_radar_servo.py)
# =============================================================================
class Servo:
    def __init__(self, pin):
        self.pwm = pwmio.PWMOut(pin, frequency=50, duty_cycle=0)
        self.angle = 90
        self.set_angle(90)
    
    def set_angle(self, angle):
        angle = max(0, min(180, angle))
        self.angle = angle
        pulse_us = 500 + (angle / 180.0) * 2000
        self.pwm.duty_cycle = int((pulse_us / 20000) * 65535)

# =============================================================================
# RADAR (from pico_radar_servo.py)
# =============================================================================
class Radar:
    MULTI_TARGET_CMD = bytes([0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01])
    
    def __init__(self):
        self.uart = busio.UART(RADAR_TX_PIN, RADAR_RX_PIN, baudrate=RADAR_BAUD, timeout=0.05)
        self.buffer = b''
        time.sleep(0.3)
        
        # Enable multi-target mode
        self.uart.write(self.MULTI_TARGET_CMD)
        time.sleep(0.2)
        while self.uart.read(64):
            pass
        self.buffer = b''
        print("Radar: OK (GP0/GP1, 256000 baud)")
    
    def _parse_signed(self, raw):
        magnitude = raw & 0x7FFF
        return magnitude if (raw & 0x8000) else -magnitude
    
    def read_targets(self):
        """Returns list of (dist_mm, x, y, speed)"""
        data = self.uart.read(64)
        if data:
            self.buffer += data
        
        if len(self.buffer) > 300:
            self.buffer = self.buffer[-100:]
        
        # Find frame start (AA FF)
        start = -1
        for i in range(len(self.buffer) - 1):
            if self.buffer[i] == 0xAA and self.buffer[i+1] == 0xFF:
                start = i
                break
        
        if start == -1:
            self.buffer = b''
            return []
        
        if start > 0:
            self.buffer = self.buffer[start:]
        
        # Find frame end (55 CC)
        for i in range(2, min(len(self.buffer) - 1, 40)):
            if self.buffer[i] == 0x55 and self.buffer[i+1] == 0xCC:
                frame = self.buffer[:i+2]
                self.buffer = self.buffer[i+2:]
                
                targets = []
                if len(frame) >= 30:
                    for t in range(3):
                        base = 4 + t * 8
                        if base + 7 < len(frame) - 2:
                            x_raw = frame[base] + (frame[base+1] << 8)
                            y_raw = frame[base+2] + (frame[base+3] << 8)
                            speed_raw = frame[base+4] + (frame[base+5] << 8)
                            
                            x = self._parse_signed(x_raw)
                            y = self._parse_signed(y_raw)
                            speed = self._parse_signed(speed_raw)
                            dist_mm = math.sqrt(x*x + y*y)
                            
                            if dist_mm > 100:  # Min 10cm
                                targets.append((int(dist_mm), x, y, speed))
                
                return targets
        return []

# =============================================================================
# IMU (from icm20948_pico.py)
# =============================================================================
class IMU:
    def __init__(self):
        self._i2c = busio.I2C(I2C_SCL, I2C_SDA)
        self._icm = None
        self.acceleration = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.magnetic = (0.0, 0.0, 0.0)
        self.heading = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        
        try:
            from adafruit_icm20x import ICM20948
            self._icm = ICM20948(self._i2c, address=IMU_ADDRESS)
            print("IMU: OK (GP20/GP21, addr 0x69)")
        except ImportError:
            print("IMU: adafruit_icm20x not found")
        except Exception as e:
            print("IMU: Failed -", e)
    
    def update(self):
        if self._icm is None:
            return
        
        try:
            self.acceleration = self._icm.acceleration
            self.gyro = self._icm.gyro
            self.magnetic = self._icm.magnetic
            
            # Calculate pitch/roll from accelerometer
            ax, ay, az = self.acceleration
            if az != 0 or ay != 0:
                self.pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 180 / math.pi
                self.roll = math.atan2(ay, az) * 180 / math.pi
            
            # Tilt-compensated heading from magnetometer
            if self.magnetic[0] is not None:
                mx, my, mz = self.magnetic
                pitch_rad = math.radians(self.pitch)
                roll_rad = math.radians(self.roll)
                
                mx_comp = mx * math.cos(pitch_rad) + mz * math.sin(pitch_rad)
                my_comp = (mx * math.sin(roll_rad) * math.sin(pitch_rad) + 
                          my * math.cos(roll_rad) - 
                          mz * math.sin(roll_rad) * math.cos(pitch_rad))
                
                self.heading = math.atan2(-my_comp, mx_comp) * 180 / math.pi
                if self.heading < 0:
                    self.heading += 360
        except OSError:
            pass
    
    def get_heading(self):
        return self.heading

# =============================================================================
# MAIN
# =============================================================================
def main():
    print("=" * 50)
    print("ANCHOR - RC Boat Controller")
    print("USB Serial Mode")
    print("=" * 50)
    
    # Initialize
    print("\nInitializing...")
    
    servo = None
    radar = None
    imu = None
    
    try:
        servo = Servo(SERVO_PIN)
        print("Servo: OK (GP12)")
    except Exception as e:
        print("Servo: FAILED -", e)
    
    try:
        radar = Radar()
    except Exception as e:
        print("Radar: FAILED -", e)
    
    try:
        imu = IMU()
    except Exception as e:
        print("IMU: FAILED -", e)
    
    print("\nStarting main loop...\n")
    
    # State
    sweep_direction = 1
    servo_angle = SERVO_MIN
    obstacles = []
    last_telemetry = time.monotonic()
    telemetry_interval = 1.0 / TELEMETRY_RATE_HZ
    
    # Send initial packets for handshake (multiple to ensure backend catches one)
    for _ in range(5):
        print(json.dumps({"lat": DEFAULT_LAT, "lon": DEFAULT_LON, "hdg": 0, "spd": 0, "bat": 100, "obs": []}))
        time.sleep(0.1)
    
    # Buffer for incoming commands
    cmd_buffer = ""
    paused = False
    
    while True:
        now = time.monotonic()
        
        # Check for commands via USB (character by character)
        while supervisor.runtime.serial_bytes_available:
            try:
                char = sys.stdin.read(1)
                if char in ('\n', '\r'):
                    if cmd_buffer:
                        try:
                            cmd = json.loads(cmd_buffer)
                            cmd_type = cmd.get("type", "")
                            
                            if cmd_type == "ping":
                                print(json.dumps({"type": "pong"}))
                            
                            elif cmd_type == "stop" or cmd_type == "pause":
                                paused = True
                                if servo:
                                    servo.set_angle(90)  # Center servo
                                print(json.dumps({"type": "status", "paused": True}))
                            
                            elif cmd_type == "start" or cmd_type == "resume":
                                paused = False
                                print(json.dumps({"type": "status", "paused": False}))
                            
                            elif cmd_type == "reset":
                                # Reset state
                                paused = False
                                servo_angle = SERVO_MIN
                                sweep_direction = 1
                                obstacles = []
                                print(json.dumps({"type": "status", "reset": True}))
                                
                        except:
                            pass
                        cmd_buffer = ""
                elif char:
                    cmd_buffer += char
            except:
                break
        
        # If paused, just send status and continue
        if paused:
            if (now - last_telemetry) >= telemetry_interval:
                last_telemetry = now
                heading = imu.get_heading() if imu else 0.0
                print(json.dumps({
                    "lat": DEFAULT_LAT,
                    "lon": DEFAULT_LON,
                    "hdg": round(heading, 1),
                    "spd": 0,
                    "bat": 100,
                    "obs": [],
                    "paused": True
                }))
            time.sleep(0.1)
            continue
        
        # Update IMU
        if imu:
            imu.update()
        
        # Update servo and read radar
        if servo:
            servo.set_angle(servo_angle)
        
        if radar:
            targets = radar.read_targets()
            for dist_mm, x, y, speed in targets:
                # Convert servo angle to relative angle
                # Servo 0° = right, 90° = forward, 180° = left
                # Flip: 90 - servo gives correct direction
                relative_angle = 90 - servo_angle
                dist_m = dist_mm / 1000.0
                
                obstacles.append([round(dist_m, 2), relative_angle, 1.0])
        
        # Sweep servo
        servo_angle += SERVO_STEP * sweep_direction
        if servo_angle >= SERVO_MAX:
            servo_angle = SERVO_MAX
            sweep_direction = -1
        elif servo_angle <= SERVO_MIN:
            servo_angle = SERVO_MIN
            sweep_direction = 1
        
        time.sleep(SERVO_DELAY)
        
        # Send telemetry
        if (now - last_telemetry) >= telemetry_interval:
            last_telemetry = now
            
            heading = imu.get_heading() if imu else 0.0
            
            telemetry = {
                "lat": DEFAULT_LAT,
                "lon": DEFAULT_LON,
                "hdg": round(heading, 1),
                "spd": 0,
                "bat": 100,
                "obs": obstacles[-15:]  # Last 15 obstacles
            }
            
            print(json.dumps(telemetry))
            
            # Trim obstacles
            if len(obstacles) > 30:
                obstacles = obstacles[-15:]


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped")
    except Exception as e:
        print("ERROR:", e)
        time.sleep(5)
