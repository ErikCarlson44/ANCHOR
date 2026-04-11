"""
=============================================================================
ANCHOR - RC BOAT MAIN CONTROLLER (USB Serial Version)
=============================================================================
Combines GPS, radar, and IMU to send telemetry to GUI via USB serial.

Working Components:
  - GPS (NEO-M8N) for position and speed
  - Radar (RD-03D) with servo sweep
  - IMU (ICM-20948) for heading

Wiring:
  GPS (NEO-M8N):
    GPS TX  -> GP1 (Pico RX)
    GPS RX  -> GP0 (Pico TX)
    GPS VCC -> 3.3V
    GPS GND -> GND

  RADAR (RD-03D):
    Radar TX  -> GP9 (Pico RX)
    Radar RX  -> GP8 (Pico TX)
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

  LoRa (RFM95W / SX1276 — SPI, optional air link):
    MISO  -> GP16
    CS    -> GP17
    SCK   -> GP18
    MOSI  -> GP19
    RST   -> GP22
    G0    -> GP26  (DIO0; optional, driver uses SPI IRQ internally)
    VIN   -> 3.3V
    GND   -> GND
    Needs: adafruit_rfm9x + adafruit_bus_device in /lib

  LoRaWAN ABP framing (step 2):
    The RFM95 transmits a fully-formed LoRaWAN 1.0 uplink frame so a real
    concentrator / network server can decode it.
    Crypto is pure-Python AES-128 (no extra libs needed).
    Set NWKSKEY, APPSKEY, DEVADDR to match your ABP device registration.

  Motor + rudder:
    ESC throttle (2 wires from Pico is enough):
      ESC signal (PWM) -> GP14
      ESC GND          -> Pico GND (common with battery negative)
      Do NOT tie the ESC BEC (+5V middle pin on the 3-pin plug) to the Pico 3.3V pin.
      Motor power: large red/black from battery go to the ESC power pads — not to GP14.
    Rudder servo      -> GP15 signal + 5V + GND per servo (3 wires typical)

  USB_GUI_CONTROL = True (default): throttle 0..1 and steering -1..0..+1 from PC lines:
    {"type":"control","throttle":<float>,"steering":<float>}\\n
    LoRa "control" packets are ignored when USB_GUI_CONTROL is True (avoids stray air commands).

  USB serial (GUI): With USB_SERIAL_QUIET = True (default), stdout is only JSON lines:
    Telemetry: {"lat","lon","hdg","spd","bat","sats","obs"} — obs = [[dist,angle,size], ...]
    Replies: {"type":"pong"} | {"type":"status",...}  (backend skips these for map updates)
    Set USB_SERIAL_QUIET = False for Thonny-friendly boot text (GUI may mis-read lines).

=============================================================================
"""

import board
import busio
import digitalio
import pwmio
import time
import math
import json
import supervisor
import sys

# =============================================================================
# CONFIGURATION
# =============================================================================

# GPS pins (NEO-M8N)
GPS_TX_PIN = board.GP0   # Pico TX -> GPS RX
GPS_RX_PIN = board.GP1   # Pico RX <- GPS TX
GPS_BAUD = 9600

# Radar pins (RD-03D)
RADAR_TX_PIN = board.GP8
RADAR_RX_PIN = board.GP9
RADAR_BAUD = 256000

# Servo pin
SERVO_PIN = board.GP12
SERVO_MIN = 30    # 30° = far right
SERVO_MAX = 150   # 150° = far left (90° = forward)
SERVO_STEP = 3  
SERVO_DELAY = 0.1

# IMU pins (ICM-20948)
I2C_SDA = board.GP20
I2C_SCL = board.GP21
IMU_ADDRESS = 0x69

# Telemetry rate
TELEMETRY_RATE_HZ = 10

# -----------------------------------------------------------------------------
# LoRa RFM95W (SPI) — LoRaWAN ABP uplink
# -----------------------------------------------------------------------------
LORA_ENABLED = True
LORA_SPI_SCK  = board.GP18
LORA_SPI_MOSI = board.GP19
LORA_SPI_MISO = board.GP16
LORA_CS  = board.GP17
LORA_RST = board.GP22
LORA_FREQ_MHZ = 915.0
LORA_SEND_EVERY_N  = 5        # uplink every N USB telemetry frames (save duty cycle)
LORA_RX_TIMEOUT_S  = 0.02    # short poll so main loop stays responsive
LORA_TX_INTERVAL_S = 30.0    # minimum seconds between LoRa uplinks (non-blocking)

# Maximum JSON payload bytes BEFORE LoRaWAN framing overhead (~13 bytes).
# SF11/BW125 max PHY payload = 59 bytes → leave 13 for framing → 46 bytes max.
LORA_MAX_PAYLOAD = 46

# -----------------------------------------------------------------------------
# ESC + rudder (GUI control over USB serial from FastAPI backend)
# Boat ESC mapping matches throttle_test.py: 1000 µs = stop, 2000 µs = full (50 Hz).
# -----------------------------------------------------------------------------
ESC_ENABLED = True
ESC_PIN = board.GP14
ESC_PWM_HZ = 50
ESC_STOP_US = 1000  # 0% throttle / idle / arm (PULSE_MIN / PULSE_NEUTRAL in throttle_test)
ESC_FULL_US = 2000  # 100% throttle (PULSE_MAX)
ESC_NEUTRAL_US = ESC_STOP_US  # pause / startup guard — hold stop pulse, not 1500 µs
ESC_MIN_US = ESC_STOP_US  # legacy alias (ESC_ARM_ON_BOOT block removed for boat safety)
# Hold stop pulse at boot so ESC arms (same idea as throttle_test.arm_esc). Set 0 to skip.
ESC_ARM_HOLD_S = 2.0
# After main() is ready, hold ESC neutral this long and ignore USB "control" JSON.
# Thonny/REPL often leaves garbage in USB RX that was parsed as throttle — flush + this guard fixes it.
MOTOR_STARTUP_GUARD_S = 3.0
# GUI throttle 0..1: below this → ESC neutral (reduces creep / stray packets / LoRa noise)
THROTTLE_DEADBAND = 0.03

RUDDER_ENABLED = True
RUDDER_PIN = board.GP15
RUDDER_CENTER = 1500
RUDDER_SPAN = 450  # µs each side; steering -1..+1 from GUI maps here

# Local throttle when USB_GUI_CONTROL is False. Ignored when USB_GUI_CONTROL True (GUI over USB).
THROTTLE_SOURCE = "serial"  # "serial" | "analog" | "buttons"
# True = throttle + rudder from USB JSON control lines (normal for Anchor GUI + backend).
USB_GUI_CONTROL = True
THROTTLE_ADC_PIN = board.GP27  # only used when THROTTLE_SOURCE == "analog"
THROTTLE_BTN_UP_PIN = board.GP2
THROTTLE_BTN_DN_PIN = board.GP3
THROTTLE_BTN_STEP = 0.06  # per main-loop tick while held (~10 Hz when not paused)

# Default GPS (Mission Bay, San Diego) - used when no fix
DEFAULT_LAT = 32.7872
DEFAULT_LON = -117.2350

# USB to PC: False = only newline-delimited JSON (telemetry + ping/pong/status).
# True = also print human boot/diagnostic lines (breaks GUI serial line reader).
USB_SERIAL_QUIET = True


def dbg(*args):
    if not USB_SERIAL_QUIET:
        print(*args)


def safe_json_float(x, default=0.0):
    """Avoid NaN/Infinity in JSON (browsers reject them)."""
    try:
        v = float(x)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(v):
        return default
    return v


def gui_telemetry_packet(lat, lon, hdg, spd, bat, sats, obs_list):
    """
    One dict matching FastAPI _read_serial → BoatTelemetry:
    lat, lon, hdg, spd, bat, sats, obs (list of [distance, angle, size]).
    """
    obs_out = []
    for o in obs_list:
        if not isinstance(o, (list, tuple)) or len(o) < 2:
            continue
        obs_out.append(
            [
                round(safe_json_float(o[0], 0.0), 2),
                round(safe_json_float(o[1], 0.0), 2),
                round(safe_json_float(o[2], 1.0), 2) if len(o) > 2 else 1.0,
            ]
        )
    return {
        "lat": round(safe_json_float(lat, DEFAULT_LAT), 6),
        "lon": round(safe_json_float(lon, DEFAULT_LON), 6),
        "hdg": round(safe_json_float(hdg, 0.0), 1),
        "spd": round(safe_json_float(spd, 0.0), 1),
        "bat": int(safe_json_float(bat, 100.0)),
        "sats": int(safe_json_float(sats, 0.0)),
        "obs": obs_out,
    }


def emit_usb_json(obj):
    """Single JSON line on USB stdout (what the backend readline() consumes)."""
    print(json.dumps(obj, separators=(",", ":")))


def flush_usb_rx_discard():
    """Clear USB CDC RX buffer. Thonny/host bytes must not become fake control commands."""
    try:
        import usb_cdc

        c = usb_cdc.console
        if c is not None:
            while c.in_waiting:
                c.read(c.in_waiting)
    except Exception:
        pass
    try:
        while supervisor.runtime.serial_bytes_available:
            sys.stdin.read(1)
    except Exception:
        pass


# =============================================================================
# MOTOR / RUDDER helpers (GUI throttle 0–1, steering −1..+1)
# =============================================================================
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def parse_throttle_01(value):
    """
    Expect 0..1 from GUI. JSON true -> float 1.0 would be 100% throttle — reject bool.
    If a sender uses 0..100 percent, values >1 (and <=100) are scaled down.
    """
    if value is None or isinstance(value, bool):
        return None
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(v):
        return None
    if v > 1.0:
        if v <= 100.0:
            v = v / 100.0
        else:
            return None
    return clamp(v, 0.0, 1.0)


def parse_steering_pm1(value):
    if value is None or isinstance(value, bool):
        return None
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(v):
        return None
    return clamp(v, -1.0, 1.0)


def us_to_duty(pulse_us):
    """Pulse width µs → 16-bit duty at 50 Hz (same as throttle_test.py)."""
    pulse_us = max(500, min(2500, int(pulse_us)))
    return int(pulse_us / 20000 * 65535)


def throttle_pct_to_us(pct):
    """Throttle 0–100 % → ESC pulse µs (1000 stopped .. 2000 full, like throttle_test)."""
    percent = max(0.0, min(100.0, float(pct)))
    return ESC_STOP_US + int((ESC_FULL_US - ESC_STOP_US) * percent / 100.0)


def rudder_pct_to_us(pct):
    """Rudder −100..+100 % → rudder servo µs."""
    return RUDDER_CENTER + int(clamp(pct, -100, 100) / 100.0 * RUDDER_SPAN)


def apply_motor_outputs(
    esc_pwm,
    rudder_pwm,
    paused,
    gui_throttle_01,
    gui_steering_pm1,
    *,
    force_neutral=False,
):
    """
    gui_throttle_01: 0..1 from React (W/S keys).
    gui_steering_pm1: -1..+1 from React (A/D keys).
    force_neutral: True during startup guard — ignore serial/GUI until ESC has stable neutral.
    """
    if paused or force_neutral:
        if esc_pwm and ESC_ENABLED:
            esc_pwm.duty_cycle = us_to_duty(ESC_NEUTRAL_US)
        if rudder_pwm and RUDDER_ENABLED:
            rudder_pwm.duty_cycle = us_to_duty(RUDDER_CENTER)
        return
    thr = clamp(gui_throttle_01, 0.0, 1.0)
    if thr < THROTTLE_DEADBAND:
        thr = 0.0
    pct_thr = thr * 100.0
    # Match GUI A/D: only full left / center / full right (no in-between)
    sr = clamp(gui_steering_pm1, -1.0, 1.0)
    if sr <= -0.5:
        sr = -1.0
    elif sr >= 0.5:
        sr = 1.0
    else:
        sr = 0.0
    pct_rud = sr * 100.0
    if esc_pwm and ESC_ENABLED:
        esc_pwm.duty_cycle = us_to_duty(throttle_pct_to_us(pct_thr))
    if rudder_pwm and RUDDER_ENABLED:
        rudder_pwm.duty_cycle = us_to_duty(rudder_pct_to_us(pct_rud))


def read_throttle_analog(adc):
    """Potentiometer on ADC: 0 V -> 0, 3.3 V -> 1."""
    if adc is None:
        return 0.0
    return clamp(adc.value / 65535.0, 0.0, 1.0)


def step_throttle_buttons(current, up_pressed, down_pressed, step):
    """up/down while held; gentle decay when released (like keyboard GUI)."""
    if up_pressed:
        current = min(1.0, current + step)
    if down_pressed:
        current = max(0.0, current - step)
    if not up_pressed and not down_pressed:
        current *= 0.95
        if current < 0.02:
            current = 0.0
    return clamp(current, 0.0, 1.0)


# =============================================================================
# GPS HELPER FUNCTIONS
# =============================================================================
def nmea_to_decimal(raw, hemisphere):
    """Convert NMEA coordinate format (DDDMM.MMMM) to decimal degrees."""
    if not raw:
        return None
    try:
        value = float(raw)
        degrees = int(value // 100)
        minutes = value - (degrees * 100)
        decimal = degrees + (minutes / 60.0)
        if hemisphere in ('S', 'W'):
            decimal = -decimal
        return decimal
    except:
        return None

def parse_gga(sentence):
    """Parse $GPGGA sentence for position fix data."""
    parts = sentence.split(',')
    if len(parts) < 10:
        return None
    try:
        fix_quality = int(parts[6]) if parts[6] else 0
        satellites = int(parts[7]) if parts[7] else 0
        latitude = nmea_to_decimal(parts[2], parts[3])
        longitude = nmea_to_decimal(parts[4], parts[5])
        altitude = float(parts[9]) if parts[9] else None
        return (fix_quality, satellites, latitude, longitude, altitude)
    except:
        return None

def parse_rmc(sentence):
    """Parse $GPRMC sentence for speed and heading."""
    parts = sentence.split(',')
    if len(parts) < 10:
        return None
    try:
        status = parts[2]
        latitude = nmea_to_decimal(parts[3], parts[4])
        longitude = nmea_to_decimal(parts[5], parts[6])
        speed_knots = float(parts[7]) if parts[7] else 0.0
        heading = float(parts[8]) if parts[8] else 0.0
        return (status, latitude, longitude, speed_knots, heading)
    except:
        return None

# =============================================================================
# GPS CLASS (NEO-M8N)
# =============================================================================
class GPS:
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
        
        dbg("GPS: OK (GP0/GP1, 9600 baud)")
    
    def update(self):
        """Read and parse available GPS data."""
        chunk = self.uart.read(64)
        if chunk:
            self.buffer += chunk
        
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            try:
                sentence = line.decode("ascii", "ignore").strip()
                
                if "*" in sentence:
                    sentence = sentence.split("*")[0]
                
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
            except:
                pass
    
    def get_data(self):
        """Return current GPS data."""
        return {
            "lat": self.latitude,
            "lon": self.longitude,
            "alt": self.altitude,
            "hdg": self.heading,
            "spd": self.speed_knots,
            "sats": self.satellites,
            "fix": self.has_fix
        }

# =============================================================================
# SERVO (Standard Positional)
# =============================================================================
class Servo:
    def __init__(self, pin):
        self.pwm = pwmio.PWMOut(pin, frequency=50, duty_cycle=0)
        self.angle = 90
        self.set_angle(90)
    
    def set_angle(self, angle):
        """Set servo to specific angle (0-180)"""
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
        dbg("Radar: OK (GP8/GP9, 256000 baud)")
    
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
            dbg("IMU: OK (GP20/GP21, addr 0x69)")
        except ImportError:
            dbg("IMU: adafruit_icm20x not found")
        except Exception as e:
            dbg("IMU: Failed -", e)
    
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
# PURE-PYTHON AES-128
# =============================================================================
# Full AES-128 implementation — no external libraries required.
# Used only for LoRaWAN payload encryption (AES-CTR) and CMAC/MIC generation.
# Reference: FIPS 197, RFC 4493.

_AES_SBOX = (
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16,
)

_AES_RCON = (
    0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x1b,0x36,
)

def _xtime(a):
    return ((a << 1) ^ 0x1b) & 0xff if (a & 0x80) else (a << 1) & 0xff

def _gmul(a, b):
    """Galois field GF(2^8) multiply."""
    p = 0
    for _ in range(8):
        if b & 1:
            p ^= a
        hi = a & 0x80
        a = (a << 1) & 0xff
        if hi:
            a ^= 0x1b
        b >>= 1
    return p

def _aes_key_expansion(key_bytes):
    """Expand a 16-byte key into 11 round keys (each 16 bytes)."""
    w = list(key_bytes)  # 16 bytes
    for i in range(4, 44):
        temp = w[(i - 1) * 4: i * 4]
        if i % 4 == 0:
            # RotWord + SubWord + Rcon
            temp = [
                _AES_SBOX[temp[1]] ^ _AES_RCON[i // 4],
                _AES_SBOX[temp[2]],
                _AES_SBOX[temp[3]],
                _AES_SBOX[temp[0]],
            ]
        w += [w[(i - 4) * 4 + j] ^ temp[j] for j in range(4)]
    # Return as list of 11 round keys, each a list of 16 bytes
    return [w[i * 16:(i + 1) * 16] for i in range(11)]

def _add_round_key(state, rk):
    return [state[i] ^ rk[i] for i in range(16)]

def _sub_bytes(state):
    return [_AES_SBOX[b] for b in state]

def _shift_rows(state):
    # state is column-major (AES convention): col0=[0,1,2,3], col1=[4,5,6,7]...
    # Row 0: no shift; row 1: left 1; row 2: left 2; row 3: left 3
    return [
        state[0],  state[5],  state[10], state[15],
        state[4],  state[9],  state[14], state[3],
        state[8],  state[13], state[2],  state[7],
        state[12], state[1],  state[6],  state[11],
    ]

def _mix_columns(state):
    out = [0] * 16
    for c in range(4):
        s0 = state[c * 4]
        s1 = state[c * 4 + 1]
        s2 = state[c * 4 + 2]
        s3 = state[c * 4 + 3]
        out[c * 4]     = _gmul(0x02, s0) ^ _gmul(0x03, s1) ^ s2 ^ s3
        out[c * 4 + 1] = s0 ^ _gmul(0x02, s1) ^ _gmul(0x03, s2) ^ s3
        out[c * 4 + 2] = s0 ^ s1 ^ _gmul(0x02, s2) ^ _gmul(0x03, s3)
        out[c * 4 + 3] = _gmul(0x03, s0) ^ s1 ^ s2 ^ _gmul(0x02, s3)
    return out

def aes128_encrypt_block(block_bytes, round_keys):
    """
    Encrypt one 16-byte block with pre-expanded round keys.
    block_bytes: bytes or list of 16 ints.
    round_keys: output of _aes_key_expansion().
    Returns bytearray of 16 bytes.
    """
    # AES state is stored column-major: state[col*4 + row]
    state = list(block_bytes)
    state = _add_round_key(state, round_keys[0])
    for rnd in range(1, 10):
        state = _sub_bytes(state)
        state = _shift_rows(state)
        state = _mix_columns(state)
        state = _add_round_key(state, round_keys[rnd])
    # Final round (no MixColumns)
    state = _sub_bytes(state)
    state = _shift_rows(state)
    state = _add_round_key(state, round_keys[10])
    return bytearray(state)


# =============================================================================
# AES-128-CMAC (RFC 4493) — used for LoRaWAN MIC
# =============================================================================

def _cmac_generate_subkeys(round_keys):
    """Derive CMAC subkeys K1, K2 from AES key (RFC 4493 §2.3)."""
    const_rb = 0x87
    L = aes128_encrypt_block(bytes(16), round_keys)
    # K1
    if L[0] & 0x80:
        k1 = bytearray((int.from_bytes(L, 'big') << 1) & 0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
                        for _ in range(1))  # placeholder — compute below
        k1 = bytearray([(L[i] << 1 | L[i+1] >> 7) & 0xff for i in range(15)] + [(L[15] << 1) & 0xff])
        k1[15] ^= const_rb
    else:
        k1 = bytearray([(L[i] << 1 | L[i+1] >> 7) & 0xff for i in range(15)] + [(L[15] << 1) & 0xff])
    # K2
    if k1[0] & 0x80:
        k2 = bytearray([(k1[i] << 1 | k1[i+1] >> 7) & 0xff for i in range(15)] + [(k1[15] << 1) & 0xff])
        k2[15] ^= const_rb
    else:
        k2 = bytearray([(k1[i] << 1 | k1[i+1] >> 7) & 0xff for i in range(15)] + [(k1[15] << 1) & 0xff])
    return bytes(k1), bytes(k2)

def aes128_cmac(key_bytes, msg_bytes):
    """
    Compute AES-128-CMAC over msg_bytes using key_bytes.
    Returns 16-byte MAC (bytearray).
    """
    rk = _aes_key_expansion(list(key_bytes))
    k1, k2 = _cmac_generate_subkeys(rk)
    msg = bytearray(msg_bytes)
    n = (len(msg) + 15) // 16  # number of blocks
    if n == 0:
        n = 1
        flag = False
    else:
        flag = (len(msg) % 16 == 0)

    if flag:
        # Last block complete — XOR with K1
        last = bytearray(msg[(n - 1) * 16:n * 16])
        last = bytearray(last[i] ^ k1[i] for i in range(16))
    else:
        # Pad last block and XOR with K2
        last_raw = msg[(n - 1) * 16:]
        pad = bytearray(last_raw) + bytearray([0x80]) + bytearray(16 - len(last_raw) - 1)
        last = bytearray(pad[i] ^ k2[i] for i in range(16))

    x = bytearray(16)
    for i in range(n - 1):
        block = msg[i * 16:(i + 1) * 16]
        y = bytearray(x[j] ^ block[j] for j in range(16))
        x = aes128_encrypt_block(y, rk)
    y = bytearray(x[j] ^ last[j] for j in range(16))
    return aes128_encrypt_block(y, rk)


# =============================================================================
# LORAWAN 1.0 ABP FRAMING
# =============================================================================
# Implements LoRaWAN 1.0 uplink (unconfirmed data up, MType=0x40).
# Spec reference: LoRaWAN 1.0.3 specification §4.

class LoRaWANFramer:
    """
    Builds a fully-formed LoRaWAN 1.0 PHYPayload for ABP uplink.

    Usage:
        framer = LoRaWANFramer(nwkskey_bytes, appskey_bytes, devaddr_uint32)
        phy = framer.build_uplink(fport, payload_bytes)
        rfm.send(phy)
    """

    MHDR_UNCONFIRMED_UP = 0x40  # MType=010 (Unconfirmed Data Up), RFU=0, Major=00

    def __init__(self, nwkskey, appskey, devaddr):
        """
        nwkskey  : 16-byte bytes/bytearray  (Network Session Key)
        appskey  : 16-byte bytes/bytearray  (Application Session Key)
        devaddr  : uint32                   (device address, big-endian constant)
        """
        self._nwkskey = bytes(nwkskey)
        self._appskey = bytes(appskey)
        self._devaddr = devaddr & 0xFFFFFFFF
        self._fcnt    = 0  # uplink frame counter (0..65535, wraps)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _devaddr_le(self):
        """DevAddr as 4 bytes, little-endian (as transmitted in FHDR)."""
        d = self._devaddr
        return bytes([d & 0xff, (d >> 8) & 0xff, (d >> 16) & 0xff, (d >> 24) & 0xff])

    def _encrypt_payload(self, plaintext, fcnt, direction=0):
        """
        LoRaWAN payload encryption: AES-128-CTR with a synthetic IV.
        direction: 0 = uplink, 1 = downlink.
        Returns bytearray of same length as plaintext.
        """
        if not plaintext:
            return bytearray()
        rk = _aes_key_expansion(list(self._appskey))
        ciphertext = bytearray()
        k = 0           # block counter (1-based per spec)
        pos = 0
        while pos < len(plaintext):
            k += 1
            # Ai block (16 bytes) — LoRaWAN spec §4.3.3
            ai = bytearray(16)
            ai[0]  = 0x01
            # ai[1..3] = 0x00 (already zeroed)
            ai[4]  = direction & 0xff
            da = self._devaddr_le()
            ai[5]  = da[0]
            ai[6]  = da[1]
            ai[7]  = da[2]
            ai[8]  = da[3]
            ai[9]  = fcnt & 0xff
            ai[10] = (fcnt >> 8) & 0xff
            ai[11] = (fcnt >> 16) & 0xff
            ai[12] = (fcnt >> 24) & 0xff
            # ai[13] = 0x00
            ai[15] = k & 0xff
            si = aes128_encrypt_block(ai, rk)
            for j in range(16):
                if pos >= len(plaintext):
                    break
                ciphertext.append(plaintext[pos] ^ si[j])
                pos += 1
        return ciphertext

    def _compute_mic(self, mhdr, fhdr, fport, frm_payload, fcnt):
        """
        Compute 4-byte LoRaWAN MIC using AES-128-CMAC over B0 || msg.
        B0 is the synthetic block defined in LoRaWAN spec §4.4.
        """
        # Build MACPayload: FHDR + FPort + FRMPayload
        mac_payload = bytes(fhdr) + bytes([fport]) + bytes(frm_payload)
        # B0 block
        b0 = bytearray(16)
        b0[0]  = 0x49
        # b0[1..3] = 0x00
        b0[4]  = 0x00  # direction: uplink
        da = self._devaddr_le()
        b0[5]  = da[0]
        b0[6]  = da[1]
        b0[7]  = da[2]
        b0[8]  = da[3]
        b0[9]  = fcnt & 0xff
        b0[10] = (fcnt >> 8) & 0xff
        b0[11] = (fcnt >> 16) & 0xff
        b0[12] = (fcnt >> 24) & 0xff
        b0[13] = 0x00
        b0[15] = len(mac_payload) & 0xff
        msg = bytes(b0) + bytes([mhdr]) + mac_payload
        full_mac = aes128_cmac(self._nwkskey, msg)
        return bytes(full_mac[:4])  # MIC = first 4 bytes

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def build_uplink(self, fport, payload_bytes):
        """
        Build a complete LoRaWAN PHYPayload for an unconfirmed uplink.

        fport         : int 1..223  (application port; 0 reserved for MAC commands)
        payload_bytes : bytes/bytearray, the plaintext application payload

        Returns bytearray ready to pass directly to rfm.send().
        """
        fcnt = self._fcnt
        self._fcnt = (self._fcnt + 1) & 0xFFFF  # increment, wrap at 16-bit

        mhdr = self.MHDR_UNCONFIRMED_UP

        # FHDR: DevAddr(4) + FCtrl(1) + FCnt(2) + FOpts(0)
        da = self._devaddr_le()
        fctrl = 0x00  # no ADR, no ACK, no FPending, FOptsLen=0
        fhdr = bytearray(da) + bytes([fctrl, fcnt & 0xff, (fcnt >> 8) & 0xff])

        # Encrypt FRMPayload
        frm_payload = self._encrypt_payload(bytearray(payload_bytes), fcnt)

        # Compute MIC
        mic = self._compute_mic(mhdr, fhdr, fport, frm_payload, fcnt)

        # Assemble PHYPayload: MHDR(1) + MACPayload(FHDR+FPort+FRMPayload) + MIC(4)
        phy = bytearray([mhdr]) + fhdr + bytes([fport]) + frm_payload + mic
        return phy

    @property
    def frame_counter(self):
        return self._fcnt


# =============================================================================
# LoRa RFM95W (adafruit_rfm9x) — LoRaWAN ABP uplink
# =============================================================================
class LoraRFM95:
    """
    Semtech SX1276/RFM95 over SPI, transmitting full LoRaWAN 1.0 ABP uplink frames.

    ABP credentials — must match your network server device registration exactly:
      NWKSKEY  : Network Session Key (16 bytes)
      APPSKEY  : Application Session Key (16 bytes)
      DEVADDR  : Device Address (uint32, big-endian constant)

    The framer handles AES-128 payload encryption and CMAC MIC generation so
    a real LoRa concentrator + network server can decode every packet.
    """

    # --- ABP Credentials (must match network server / concentrator registration) ---
    NWKSKEY = bytes([
        0xDE, 0xB1, 0xDA, 0xD2, 0xFA, 0xD3, 0xBA, 0xD4,
        0xFA, 0xB5, 0xFE, 0xD6, 0xAB, 0xBA, 0xDB, 0xBA,
    ])
    APPSKEY = bytes([
        0xDE, 0xB1, 0xDA, 0xD2, 0xFA, 0xD3, 0xBA, 0xD4,
        0xFA, 0xB5, 0xFE, 0xD6, 0xAB, 0xBA, 0xDB, 0xBA,
    ])
    DEVADDR = 0xC33F2026  # big-endian constant; transmitted little-endian in FHDR

    FPORT = 1             # application port (1..223)

    def __init__(self):
        import adafruit_rfm9x

        self.spi = busio.SPI(LORA_SPI_SCK, MOSI=LORA_SPI_MOSI, MISO=LORA_SPI_MISO)
        self.cs  = digitalio.DigitalInOut(LORA_CS)
        self.cs.direction  = digitalio.Direction.OUTPUT
        self.rst = digitalio.DigitalInOut(LORA_RST)
        self.rst.direction = digitalio.Direction.OUTPUT
        self.rfm = adafruit_rfm9x.RFM9x(self.spi, self.cs, self.rst, LORA_FREQ_MHZ)
        self.rfm.signal_bandwidth  = 125000
        self.rfm.spreading_factor  = 11
        self.rfm.coding_rate       = 5
        self.rfm.enable_crc        = True
        self.rfm.tx_power          = 17
        self.rfm.receive_timeout   = LORA_RX_TIMEOUT_S
        self.last_rssi = None

        # LoRaWAN framer — owns the frame counter
        self._framer = LoRaWANFramer(self.NWKSKEY, self.APPSKEY, self.DEVADDR)

        dbg(
            "LoRa RFM95: OK  {:.1f} MHz  SF{}  BW125k  CRC on  LoRaWAN ABP".format(
                LORA_FREQ_MHZ, self.rfm.spreading_factor
            )
        )

    def get_actual_freq(self):
        frf = (
            self.rfm._read_u8(0x06) << 16
            | self.rfm._read_u8(0x07) << 8
            | self.rfm._read_u8(0x08)
        )
        return (frf * 32_000_000) / (2 ** 19) / 1_000_000

    def send_lorawan(self, payload_dict):
        """
        Encode payload_dict as JSON, build a LoRaWAN ABP uplink frame, and transmit.

        Steps:
          1. Serialise dict → UTF-8 JSON bytes (truncated to LORA_MAX_PAYLOAD)
          2. LoRaWANFramer.build_uplink() → encrypt with AppSKey (AES-CTR) +
             compute MIC with NwkSKey (AES-CMAC) → full PHYPayload
          3. rfm.send(PHYPayload)

        The concentrator/LNS will:
          - Validate the MIC using NwkSKey
          - Decrypt the payload using AppSKey
          - Forward the plaintext JSON to your application
        """
        raw = json.dumps(payload_dict, separators=(",", ":")).encode("utf-8")
        if len(raw) > LORA_MAX_PAYLOAD:
            raw = raw[:LORA_MAX_PAYLOAD]

        phy = self._framer.build_uplink(self.FPORT, raw)

        dbg(
            "LoRa TX  FCnt={}  payload={} B  PHY={} B  freq={:.4f} MHz".format(
                self._framer.frame_counter - 1,
                len(raw),
                len(phy),
                self.get_actual_freq(),
            )
        )
        return self.rfm.send(phy)

    def try_receive_json(self):
        """
        Poll for one raw LoRa packet; return dict if it is a valid UTF-8 JSON object.

        NOTE: Downlink LoRaWAN frames from the network server are also encrypted
        and MACed.  For the ANCHOR use case (one-way telemetry uplink) this path
        is only used to receive raw JSON control packets from a matching RFM95
        ground station (i.e. another Pico running the same code without a
        concentrator in the path).  Full downlink LoRaWAN decryption is not
        implemented here.
        """
        packet = self.rfm.receive()
        if packet is None:
            return None
        try:
            self.last_rssi = self.rfm.last_rssi
        except Exception:
            self.last_rssi = None
        try:
            obj = json.loads(packet.decode("utf-8"))
            if isinstance(obj, dict):
                dbg("LoRa RX JSON RSSI={} dBm".format(self.last_rssi))
                return obj
        except Exception:
            pass
        dbg("LoRa RX (non-JSON) {} bytes RSSI={}".format(len(packet), self.last_rssi))
        return None


# =============================================================================
# MAIN
# =============================================================================
def main():
    dbg("=" * 50)
    dbg("ANCHOR - RC Boat Controller")
    dbg("USB Serial Mode")
    dbg("=" * 50)
    dbg("\nInitializing...")
    
    gps   = None
    servo = None
    radar = None
    imu   = None
    lora  = None

    if LORA_ENABLED:
        try:
            lora = LoraRFM95()
        except ImportError:
            dbg("LoRa: adafruit_rfm9x missing — copy from CircuitPython bundle to /lib")
        except Exception as e:
            dbg("LoRa RFM95: FAILED -", e)
            lora = None

    try:
        gps = GPS()
    except Exception as e:
        dbg("GPS: FAILED -", e)
    
    try:
        servo = Servo(SERVO_PIN)
        dbg("Servo: OK (GP12)")
    except Exception as e:
        dbg("Servo: FAILED -", e)
    
    try:
        radar = Radar()
    except Exception as e:
        dbg("Radar: FAILED -", e)
    
    try:
        imu = IMU()
    except Exception as e:
        dbg("IMU: FAILED -", e)

    esc_pwm    = None
    rudder_pwm = None
    if ESC_ENABLED:
        try:
            esc_pwm = pwmio.PWMOut(
                ESC_PIN,
                frequency=ESC_PWM_HZ,
                duty_cycle=us_to_duty(ESC_NEUTRAL_US),
            )
            dbg(
                "ESC: OK (GP14) — GUI 0..1 → {}..{} µs (throttle_test mapping)".format(
                    ESC_STOP_US,
                    ESC_FULL_US,
                )
            )
            if ESC_ARM_HOLD_S > 0:
                esc_pwm.duty_cycle = us_to_duty(ESC_NEUTRAL_US)
                time.sleep(ESC_ARM_HOLD_S)
        except Exception as e:
            dbg("ESC: FAILED -", e)
            esc_pwm = None
    if RUDDER_ENABLED:
        try:
            rudder_pwm = pwmio.PWMOut(
                RUDDER_PIN, frequency=50, duty_cycle=us_to_duty(RUDDER_CENTER)
            )
            dbg("Rudder: OK (GP15) — GUI steering -1..1")
        except Exception as e:
            dbg("Rudder: FAILED -", e)
            rudder_pwm = None

    throttle_adc    = None
    throttle_btn_up = None
    throttle_btn_dn = None
    if USB_GUI_CONTROL:
        dbg("Motor: USB host — throttle + rudder from JSON control lines (GUI/backend)")
    elif THROTTLE_SOURCE == "analog":
        try:
            import analogio
            throttle_adc = analogio.AnalogIn(THROTTLE_ADC_PIN)
            dbg("Throttle: ANALOG on", THROTTLE_ADC_PIN, "(0 V = stop, 3.3 V = full)")
        except Exception as e:
            dbg("Throttle ADC FAILED:", e)
    elif THROTTLE_SOURCE == "buttons":
        if THROTTLE_BTN_UP_PIN == ESC_PIN or THROTTLE_BTN_DN_PIN == ESC_PIN:
            dbg(
                "Throttle: BUTTONS ignored — THROTTLE_BTN_* cannot be ESC_PIN (GP14). "
                "ESC needs that pin for PWM; wire throttle switches to e.g. GP2/GP3."
            )
        else:
            try:
                throttle_btn_up = digitalio.DigitalInOut(THROTTLE_BTN_UP_PIN)
                throttle_btn_up.direction = digitalio.Direction.INPUT
                throttle_btn_up.pull      = digitalio.Pull.UP
                throttle_btn_dn = digitalio.DigitalInOut(THROTTLE_BTN_DN_PIN)
                throttle_btn_dn.direction = digitalio.Direction.INPUT
                throttle_btn_dn.pull      = digitalio.Pull.UP
                dbg(
                    "Throttle: BUTTONS —",
                    THROTTLE_BTN_UP_PIN,
                    "= up,",
                    THROTTLE_BTN_DN_PIN,
                    "= down (connect pin to GND when pressed)",
                )
            except Exception as e:
                dbg("Throttle buttons FAILED:", e)
    else:
        dbg("Throttle: SERIAL — JSON control over USB")

    dbg("\nStarting main loop...\n")

    # State
    sweep_direction = 1
    servo_angle     = SERVO_MIN
    obstacles       = []
    last_telemetry  = time.monotonic()
    telemetry_interval = 1.0 / TELEMETRY_RATE_HZ

    # Non-blocking LoRa TX timer — replaces time.sleep(30) which froze the loop.
    # Initialise to 0 so the very first eligible frame is transmitted immediately.
    lora_next_tx_mono = 0.0

    # Initial telemetry lines for backend handshake (same schema as periodic telemetry)
    for _ in range(5):
        emit_usb_json(
            gui_telemetry_packet(DEFAULT_LAT, DEFAULT_LON, 0.0, 0.0, 100, 0, [])
        )
        time.sleep(0.1)

    # Drop any bytes Thonny/PC queued before we start parsing commands
    flush_usb_rx_discard()
    # ESC neutral + ignore {"type":"control",...} until this time (full window from *here*)
    esc_arm_until_mono = time.monotonic() + MOTOR_STARTUP_GUARD_S

    # Buffer for incoming commands
    cmd_buffer  = ""
    paused      = False
    lora_tx_counter = 0
    gui_throttle = 0.0   # 0..1 from GUI WebSocket → backend → USB
    gui_steering = 0.0   # -1..1

    def process_command(cmd):
        """Handle one JSON object from USB or LoRa (ping, stop, control, …)."""
        nonlocal paused, servo_angle, sweep_direction, obstacles, gui_throttle, gui_steering
        if not isinstance(cmd, dict):
            return
        cmd_type = cmd.get("type", "")
        if cmd_type == "ping":
            emit_usb_json({"type": "pong"})
        elif cmd_type in ("stop", "pause"):
            paused = True
            gui_throttle = 0.0
            gui_steering = 0.0
            if servo:
                servo.set_angle(90)
            emit_usb_json({"type": "status", "paused": True})
        elif cmd_type in ("start", "resume"):
            paused = False
            emit_usb_json({"type": "status", "paused": False})
        elif cmd_type == "reset":
            paused       = False
            gui_throttle = 0.0
            gui_steering = 0.0
            servo_angle  = SERVO_MIN
            sweep_direction = 1
            obstacles    = []
            emit_usb_json({"type": "status", "reset": True})
        elif cmd_type == "control":
            # Throttle ignored until ESC arm window; steering may still update
            t = cmd.get("throttle")
            s = cmd.get("steering")
            armed = time.monotonic() >= esc_arm_until_mono
            use_usb_throttle = USB_GUI_CONTROL or THROTTLE_SOURCE == "serial"
            if use_usb_throttle and armed:
                nv = parse_throttle_01(t)
                if nv is not None:
                    gui_throttle = nv
            ns = parse_steering_pm1(s)
            if ns is not None:
                gui_steering = ns

    USB_CMD_BUF_MAX = 512

    def drain_usb_commands():
        """Read all pending USB host bytes (GUI/backend); parse newline-delimited JSON."""
        nonlocal cmd_buffer
        text = ""
        try:
            import usb_cdc
            cons = usb_cdc.console
            if cons is not None:
                nw = cons.in_waiting
                if nw:
                    text = cons.read(min(nw, 256)).decode("utf-8", "replace")
        except Exception:
            pass
        if not text:
            try:
                while supervisor.runtime.serial_bytes_available:
                    c = sys.stdin.read(1)
                    if not c:
                        break
                    text += c
            except Exception:
                return
        for char in text:
            if char in "\n\r":
                if cmd_buffer:
                    line = cmd_buffer.strip()
                    cmd_buffer = ""
                    if line and len(line) <= USB_CMD_BUF_MAX:
                        try:
                            process_command(json.loads(line))
                        except Exception:
                            pass
                else:
                    cmd_buffer = ""
            elif len(cmd_buffer) < USB_CMD_BUF_MAX:
                cmd_buffer += char
            else:
                cmd_buffer = ""

    # ------------------------------------------------------------------
    # Helper: attempt a LoRa uplink if the non-blocking timer has elapsed.
    # Call this once per telemetry frame (not inside the LORA_SEND_EVERY_N
    # gate — that gate is now the ONLY rate limiter; LORA_TX_INTERVAL_S is
    # the hard floor between any two TX calls).
    # ------------------------------------------------------------------
    def maybe_lora_tx(pkt):
        nonlocal lora_tx_counter, lora_next_tx_mono
        if not lora:
            return
        lora_tx_counter += 1
        if lora_tx_counter < LORA_SEND_EVERY_N:
            return
        lora_tx_counter = 0
        now2 = time.monotonic()
        if now2 < lora_next_tx_mono:
            return  # still within quiet period — skip without blocking
        lora_next_tx_mono = now2 + LORA_TX_INTERVAL_S
        try:
            lora.send_lorawan(pkt)
        except Exception as e:
            dbg("LoRa TX:", e)

    # ==========================================================================
    # MAIN LOOP
    # ==========================================================================
    while True:
        now = time.monotonic()

        # LoRa RX: defer until ESC arm window done
        if lora and now >= esc_arm_until_mono:
            lcmd = lora.try_receive_json()
            if lcmd:
                try:
                    if USB_GUI_CONTROL and lcmd.get("type") == "control":
                        pass
                    else:
                        process_command(lcmd)
                except Exception:
                    pass

        # USB: always drain (ping/pong for GUI handshake)
        drain_usb_commands()

        if not USB_GUI_CONTROL and THROTTLE_SOURCE == "analog" and throttle_adc:
            gui_throttle = read_throttle_analog(throttle_adc)
        elif (
            not USB_GUI_CONTROL
            and THROTTLE_SOURCE == "buttons"
            and throttle_btn_up is not None
            and throttle_btn_dn is not None
        ):
            gui_throttle = step_throttle_buttons(
                gui_throttle,
                not throttle_btn_up.value,
                not throttle_btn_dn.value,
                THROTTLE_BTN_STEP,
            )

        apply_motor_outputs(
            esc_pwm,
            rudder_pwm,
            paused,
            gui_throttle,
            gui_steering,
            force_neutral=(now < esc_arm_until_mono),
        )

        # ------------------------------------------------------------------
        # PAUSED branch
        # ------------------------------------------------------------------
        if paused:
            if gps:
                gps.update()
            if (now - last_telemetry) >= telemetry_interval:
                last_telemetry = now
                heading  = imu.get_heading() if imu else 0.0
                gps_data = gps.get_data() if gps else {}
                lat  = gps_data.get("lat") or DEFAULT_LAT
                lon  = gps_data.get("lon") or DEFAULT_LON
                spd  = gps_data.get("spd") or 0
                sats = gps_data.get("sats") or 0
                pkt  = gui_telemetry_packet(lat, lon, heading, spd, 100, sats, [])
                emit_usb_json(pkt)
                maybe_lora_tx(pkt)
            time.sleep(0.1)
            continue

        # ------------------------------------------------------------------
        # ACTIVE branch
        # ------------------------------------------------------------------

        # Update GPS
        if gps:
            gps.update()

        # Update IMU
        if imu:
            imu.update()

        # Update servo and read radar
        if servo:
            servo.set_angle(servo_angle)

        if radar:
            targets = radar.read_targets()
            for dist_mm, x, y, speed in targets:
                # Servo 0° = right, 90° = forward, 180° = left
                relative_angle = 90 - servo_angle
                dist_m = dist_mm / 1000.0
                obstacles.append([round(dist_m, 2), relative_angle, 1.0])

        # Sweep servo
        servo_angle += SERVO_STEP * sweep_direction
        if servo_angle >= SERVO_MAX:
            servo_angle  = SERVO_MAX
            sweep_direction = -1
        elif servo_angle <= SERVO_MIN:
            servo_angle  = SERVO_MIN
            sweep_direction = 1

        time.sleep(SERVO_DELAY)

        # ------------------------------------------------------------------
        # Telemetry
        # ------------------------------------------------------------------
        if (now - last_telemetry) >= telemetry_interval:
            last_telemetry = now

            heading  = imu.get_heading() if imu else 0.0
            gps_data = gps.get_data() if gps else {}
            lat  = gps_data.get("lat")
            lon  = gps_data.get("lon")
            spd  = gps_data.get("spd") or 0
            sats = gps_data.get("sats") or 0

            if lat is None or lon is None:
                lat = DEFAULT_LAT
                lon = DEFAULT_LON

            if not imu and gps_data.get("hdg"):
                heading = gps_data.get("hdg")

            telemetry = gui_telemetry_packet(
                lat, lon, heading, spd, 100, sats, obstacles[-15:]
            )
            emit_usb_json(telemetry)
            maybe_lora_tx(telemetry)

            # Trim obstacle history
            if len(obstacles) > 30:
                obstacles = obstacles[-15:]


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        dbg("\nStopped")
    except Exception as e:
        dbg("ERROR:", e)
        time.sleep(5)
