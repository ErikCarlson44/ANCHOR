"""
=============================================================================
RD-03D RADAR + SERVO - Pico Code
=============================================================================
Multi-target radar with servo sweep.
Sends JSON data via USB serial to PC display.

Copy this to your Pico as code.py

Wiring:
  Radar TX  -> Pico GP1 (RX)
  Radar RX  -> Pico GP0 (TX)
  Radar VCC -> 5V
  Radar GND -> GND
  Servo Sig -> Pico GP12
  Servo VCC -> 5V (or external)
  Servo GND -> GND
=============================================================================
"""

import busio
import board
import pwmio
import time
import math
import json

# =============================================================================
# CONFIGURATION
# =============================================================================
SERVO_PIN = board.GP12
SERVO_MIN = 0
SERVO_MAX = 180
SERVO_STEP = 5
SERVO_DELAY = 0.08

RADAR_BAUD = 256000

# RD-03D Commands
MULTI_TARGET_CMD = bytes([0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01])

# =============================================================================
# SERVO
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
# RADAR - Multi-Target Mode (up to 3 targets)
# =============================================================================
class Radar:
    def __init__(self):
        self.uart = busio.UART(board.GP0, board.GP1, baudrate=RADAR_BAUD, timeout=0.05)
        self.buffer = b''
        time.sleep(0.3)
        
        # Enable multi-target mode
        self.uart.write(MULTI_TARGET_CMD)
        time.sleep(0.2)
        while self.uart.read(64):
            pass
        self.buffer = b''
    
    def _parse_signed(self, raw):
        magnitude = raw & 0x7FFF
        return magnitude if (raw & 0x8000) else -magnitude
    
    def read_all(self):
        """Read all targets: [(dist_mm, x, y, speed), ...]"""
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
                
                # Multi-target: 3 targets x 8 bytes each
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
                            
                            if dist_mm > 100:
                                targets.append((int(dist_mm), x, y, speed))
                
                return targets
        
        return []

# =============================================================================
# MAIN
# =============================================================================
servo = Servo(SERVO_PIN)
radar = Radar()

time.sleep(0.5)
servo.set_angle(SERVO_MIN)
time.sleep(0.3)

sweep_direction = 1
current_angle = SERVO_MIN

while True:
    servo.set_angle(current_angle)
    time.sleep(SERVO_DELAY)
    
    targets = radar.read_all()
    
    # Send JSON for each target
    if targets:
        for i, (dist, x, y, speed) in enumerate(targets):
            print(json.dumps({
                "servo": current_angle,
                "target": i + 1,
                "dist": dist,
                "x": x,
                "y": y,
                "speed": speed
            }))
    else:
        print(json.dumps({
            "servo": current_angle,
            "target": 0,
            "dist": None,
            "x": None,
            "y": None,
            "speed": None
        }))
    
    # Update sweep
    current_angle += SERVO_STEP * sweep_direction
    
    if current_angle >= SERVO_MAX:
        current_angle = SERVO_MAX
        sweep_direction = -1
    elif current_angle <= SERVO_MIN:
        current_angle = SERVO_MIN
        sweep_direction = 1
