
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
    Note: Raw LoRa is not LoRaWAN. A MultiTech mDot (ground) will not decode these
          packets unless you add a compatible LoRa bridge — use another RFM95 with
          identical frequency/SF/BW/CRC, or a LoRaWAN gateway.

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
# LoRa RFM95W (SPI) — raw packet TX/RX (must match partner radio settings)
# -----------------------------------------------------------------------------
LORA_ENABLED = True
LORA_SPI_SCK = board.GP18
LORA_SPI_MOSI = board.GP19
LORA_SPI_MISO = board.GP16
LORA_CS = board.GP17
LORA_RST = board.GP22
LORA_FREQ_MHZ = 915.0
LORA_SEND_EVERY_N = 5       # uplink every N USB telemetry frames (save duty cycle)
LORA_RX_TIMEOUT_S = 0.02  # short poll so main loop stays responsive
LORA_MAX_PACKET = 11     # bytes (LoRa MTU ~255; keep JSON small)

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
# LoRa RFM95W (adafruit_rfm9x) — raw JSON packets
# =============================================================================
class LoraRFM95:
    """
    Semtech SX1276/RFM95 over SPI. Partner radio must use same freq / SF / BW / CRC.
    """
    
    # --- ABP Credentials (must match MTDOT AT commands) ---
    NWKSKEY = bytes([
        0xDE, 0xB1, 0xDA, 0xD2, 0xFA, 0xD3, 0xBA, 0xD4,
        0xFA, 0xB5, 0xFE, 0xD6, 0xAB, 0xBA, 0xDB, 0xBA
    ])
    APPSKEY = bytes([
        0xDE, 0xB1, 0xDA, 0xD2, 0xFA, 0xD3, 0xBA, 0xD4,
        0xFA, 0xB5, 0xFE, 0xD6, 0xAB, 0xBA, 0xDB, 0xBA
    ])
    DEVADDR = 0xC33F2026   # big-endian constant; transmitted little-endian

    FPORT  = 1             # application port (must be > 0)
    DIRECTION_UP = 0       # uplink

    def __init__(self):
        import adafruit_rfm9x

        self.spi = busio.SPI(LORA_SPI_SCK, MOSI=LORA_SPI_MOSI, MISO=LORA_SPI_MISO)
        self.cs = digitalio.DigitalInOut(LORA_CS)
        self.cs.direction = digitalio.Direction.OUTPUT
        self.rst = digitalio.DigitalInOut(LORA_RST)
        self.rst.direction = digitalio.Direction.OUTPUT
        self.rfm = adafruit_rfm9x.RFM9x(self.spi, self.cs, self.rst, LORA_FREQ_MHZ)
        self.rfm.signal_bandwidth = 125000
        self.rfm.spreading_factor = 11
        self.rfm.coding_rate = 5
        self.rfm.enable_crc = True
        self.rfm.tx_power = 17
        self.rfm.receive_timeout = LORA_RX_TIMEOUT_S
        self.last_rssi = None
        
        # LoRaWAN framer — owns the frame counter
        self._framer = LoRaWANFramer(self.NWKSKEY, self.APPSKEY, self.DEVADDR)
        
        dbg(
            "LoRa RFM95: OK  {:.1f} MHz  SF{}  BW125k  CRC on".format(
                LORA_FREQ_MHZ, self.rfm.spreading_factor
            )
        )
        
    def get_actual_freq(self):
        frf = (self.rfm._read_u8(0x06) << 16 | self.rfm._read_u8(0x07) << 8  | self.rfm._read_u8(0x08))
        return (frf * 32000000) / (2**19) / 1000000

    def send_json_dict(self, payload_dict):
        """Send one LoRa packet (UTF-8 JSON)."""
        raw = json.dumps(payload_dict, separators=(",", ":")).encode("utf-8")
        if len(raw) > LORA_MAX_PACKET:
            raw = raw[:LORA_MAX_PACKET]
            
        print("LoRa TX payload:", raw.decode("utf-8"))   # <-- here
        print("LoRa TX actual freq: {:.4f} MHz".format(self.get_actual_freq()))
        return self.rfm.send(raw)

    def try_receive_json(self):
        """Poll for one LoRa packet; return dict if UTF-8 JSON (same command types as USB)."""
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
    
    gps = None
    servo = None
    radar = None
    imu = None
    lora = None

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

    esc_pwm = None
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

    throttle_adc = None
    throttle_btn_up = None
    throttle_btn_dn = None
    if USB_GUI_CONTROL:
        dbg("Motor: USB host — throttle + rudder from JSON control lines (GUI/backend)")
    elif THROTTLE_SOURCE == "analog":
        try:
            import analogio

            throttle_adc = analogio.AnalogIn(THROTTLE_ADC_PIN)
            dbg(
                "Throttle: ANALOG on",
                THROTTLE_ADC_PIN,
                "(0 V = stop, 3.3 V = full)",
            )
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
                throttle_btn_up.pull = digitalio.Pull.UP
                throttle_btn_dn = digitalio.DigitalInOut(THROTTLE_BTN_DN_PIN)
                throttle_btn_dn.direction = digitalio.Direction.INPUT
                throttle_btn_dn.pull = digitalio.Pull.UP
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
    servo_angle = SERVO_MIN
    obstacles = []
    last_telemetry = time.monotonic()
    telemetry_interval = 1.0 / TELEMETRY_RATE_HZ

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
    cmd_buffer = ""
    paused = False
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
            paused = False
            gui_throttle = 0.0
            gui_steering = 0.0
            servo_angle = SERVO_MIN
            sweep_direction = 1
            obstacles = []
            emit_usb_json({"type": "status", "reset": True})
        elif cmd_type == "control":
            # Throttle ignored until ESC arm window; steering may still update (ESC stays neutral via force_neutral)
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

    while True:
        now = time.monotonic()

        # LoRa: defer all command processing until ESC arm window done (noise + no race with neutral)
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

        # USB: always drain (ping/pong for GUI handshake). "control" ignored in process_command until armed.
        drain_usb_commands()

        if (
            not USB_GUI_CONTROL
            and THROTTLE_SOURCE == "analog"
            and throttle_adc
        ):
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

        # If paused, just send status and continue
        if paused:
            if gps:
                gps.update()
            if (now - last_telemetry) >= telemetry_interval:
                last_telemetry = now
                heading = imu.get_heading() if imu else 0.0
                gps_data = gps.get_data() if gps else {}
                lat = gps_data.get("lat") or DEFAULT_LAT
                lon = gps_data.get("lon") or DEFAULT_LON
                spd = gps_data.get("spd") or 0
                sats = gps_data.get("sats") or 0
                pkt = gui_telemetry_packet(lat, lon, heading, spd, 100, sats, [])
                emit_usb_json(pkt)
                if lora:
                    lora_tx_counter += 1
                    if lora_tx_counter >= LORA_SEND_EVERY_N:
                        lora_tx_counter = 0
                        try:
                            lora.send_json_dict(pkt)
                            time.sleep(30) #for testing
                        except Exception as e:
                            dbg("LoRa TX:", e)
            time.sleep(0.1)
            continue
        
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
                # Convert servo angle to relative angle
                # Servo 0° = right, 90° = forward, 180° = left
                # Relative: 90 - servo gives correct direction
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
            
            # Get heading from IMU (more accurate)
            heading = imu.get_heading() if imu else 0.0
            
            # Get position and speed from GPS
            gps_data = gps.get_data() if gps else {}
            lat = gps_data.get("lat")
            lon = gps_data.get("lon")
            spd = gps_data.get("spd") or 0
            sats = gps_data.get("sats") or 0
            
            # Use defaults only if no GPS fix
            if lat is None or lon is None:
                lat = DEFAULT_LAT
                lon = DEFAULT_LON
            
            # Use GPS heading as fallback if no IMU
            if not imu and gps_data.get("hdg"):
                heading = gps_data.get("hdg")
            
            telemetry = gui_telemetry_packet(
                lat, lon, heading, spd, 100, sats, obstacles[-15:]
            )
            emit_usb_json(telemetry)

            # LoRa uplink (RFM95) — partner must use same radio settings
            if lora:
                lora_tx_counter += 1
                if lora_tx_counter >= LORA_SEND_EVERY_N:
                    lora_tx_counter = 0
                    try:
                        lora.send_json_dict(telemetry)
                        time.sleep(30) #for testing
                    except Exception as e:
                        dbg("LoRa TX:", e)
            
            # Trim obstacles
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
