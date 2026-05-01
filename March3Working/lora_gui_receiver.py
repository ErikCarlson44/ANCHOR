"""
=============================================================================
ANCHOR — LoRa RFM9x RECEIVER  (guaranteed-match for transmitter code.py)
=============================================================================
Transmitter sends an 11-byte binary struct every ~30 s (due to time.sleep(30)
after each TX). This receiver decodes that exact struct and also handles
any JSON control packets sent back the other way.

Transmitter binary layout (from pack_telemetry_binary):
  struct.pack('<eeBBbBbBB', lat_f16, lon_f16,
              hdg_b,           # uint8  round(hdg * 255/359)
              r0, a0,          # uint8 dist_m, int8 angle (obs 0)
              r1, a1,          # uint8 dist_m, int8 angle (obs 1)
              spd_b, bat_b)    # uint8 speed (knots), uint8 battery %
  Total = 2+2+1+1+1+1+1+1+1 = 11 bytes  (matches LORA_MAX_PACKET = 11)

Air-interface (ALL must match transmitter exactly):
  Frequency  : 915.0 MHz
  SF         : 11
  BW         : 125 kHz
  Coding rate: 4/5   (adafruit_rfm9x value = 5)
  CRC        : True
  TX power   : +17 dBm  (set on RX side too for symmetry / any ACK TX)

Wiring:
  RFM9x SCK  -> GP18    RFM9x MOSI -> GP19
  RFM9x MISO -> GP16    RFM9x CS   -> GP17
  RFM9x RST  -> GP22    VCC -> 3.3V  GND -> GND

Notes:
  - Transmitter does time.sleep(30) after each send -> expect ~1 packet/30 s.
    Receiver stays in continuous-listen mode so nothing is missed.
  - RSSI worse than -120 dBm at SF11/BW125 usually means antenna issue.
  - If rx_count never increments: confirm SF/BW/freq match on BOTH radios.

  GUI bridge output:
  - This file now prints ONE compact JSON line per decoded packet in the exact
    format expected by backend/main.py:
      {"lat","lon","hdg","spd","bat","obs"}
  - The transmitter format remains unchanged (11-byte binary).
=============================================================================
"""

import board
import busio
import digitalio
import struct
import json
import time
import sys
import supervisor

try:
    from anchor_lora_compact import (
        LORA_BIN_LEN,
        unpack_telemetry_11,
        pack_control_11,
        is_ping_11,
        is_pong_11,
    )
    HAS_COMPACT = True
except ImportError:
    LORA_BIN_LEN = 11
    HAS_COMPACT = False

# Pin / radio constants (must match transmitter)
LORA_SPI_SCK = board.GP18
LORA_SPI_MOSI = board.GP19
LORA_SPI_MISO = board.GP16
LORA_CS = board.GP17
LORA_RST = board.GP22
LORA_FREQ_MHZ = 915.0
LORA_SF = 11
LORA_BW = 125000
LORA_CR = 5
LORA_CRC = True
LORA_TX_POWER = 17
# Non-blocking RX polling keeps GUI control latency low.
LORA_RX_TIMEOUT = 0.0

PACKET_LEN = 11
STRUCT_FMT = "<eeBBBBBBx"   # lat lon hdg r0 a0 r1 a1 spd reserved(x)
STRUCT_SIZE = struct.calcsize(STRUCT_FMT)

STATUS_INTERVAL = 10.0

# How often (seconds) to re-send throttle while running to keep the boat moving
THROTTLE_RESEND_INTERVAL = 0.5


def decode_packet(raw_bytes):
    """Decode the 12-byte binary telemetry packet from the transmitter."""
    # Preferred: decode using the shared compact protocol used by boat code.py.
    if HAS_COMPACT and len(raw_bytes) == LORA_BIN_LEN:
        if is_ping_11(raw_bytes) or is_pong_11(raw_bytes):
            return None
        decoded = unpack_telemetry_11(raw_bytes)
        if decoded is not None:
            return {
                "lat": round(float(decoded.get("lat", 0.0)), 5),
                "lon": round(float(decoded.get("lon", 0.0)), 5),
                "hdg_deg": round(float(decoded.get("hdg", 0.0)), 1),
                "obs": [
                    {"dist_m": float(o.get("r", 0)), "angle_deg": round(float(o.get("a", 0)), 1)}
                    for o in decoded.get("obs", [])
                    if isinstance(o, dict)
                ],
                "spd_knots": float(decoded.get("spd", 0.0)),
            }

    # Fallback: legacy direct-struct format.
    if len(raw_bytes) < STRUCT_SIZE:
        return None
    try:
        lat_f, lon_f, hdg_b, r0, a0, r1, a1, spd_b = struct.unpack(
            STRUCT_FMT, raw_bytes[:STRUCT_SIZE]
        )
    except Exception as e:
        print("  ! struct.unpack failed:", e)
        return None

    hdg_deg = round(hdg_b * 360.0 / 255.0, 1)

    return {
        "lat": round(float(lat_f), 5),
        "lon": round(float(lon_f), 5),
        "hdg_deg": hdg_deg,
        "obs": [
            {"dist_m": float(r0), "angle_deg": round(a0 * 360.0 / 255.0, 1)},
            {"dist_m": float(r1), "angle_deg": round(a1 * 360.0 / 255.0, 1)},
        ],
        "spd_knots": round(spd_b * 10.0 / 255.0, 2),
    }


def to_gui_telemetry(decoded):
    """
    Convert decoded LoRa packet to backend/main.py expected schema.
    Keeps transmitter format unchanged; conversion happens only on receiver.
    """
    obs = []
    for target in decoded.get("obs", []):
        dist = float(target.get("dist_m", 0.0))
        ang = float(target.get("angle_deg", 0.0))
        if dist > 0:
            obs.append([round(dist, 2), round(ang, 1), 1.0])

    return {
        "lat": round(float(decoded.get("lat", 0.0)), 6),
        "lon": round(float(decoded.get("lon", 0.0)), 6),
        "hdg": round(float(decoded.get("hdg_deg", 0.0)), 1),
        "spd": round(float(decoded.get("spd_knots", 0.0)), 1),
        "obs": obs,
    }


def emit_gui_json(decoded):
    """One single-line JSON print for backend serial readline()."""
    print(json.dumps(to_gui_telemetry(decoded), separators=(",", ":")))


def pretty_print_packet(decoded, rssi, packet_num):
    """Print a single decoded telemetry packet in a readable format."""
    o = decoded.get("obs", [])
    has_obs = len(o) > 0
    print(
        "[PKT #{:04d}]  RSSI={:>7} dBm\n"
        "  Position : lat={:.5f}  lon={:.5f}\n"
        "  Heading  : {:.1f} deg\n"
        "  Speed    : {:.1f} kn".format(
            packet_num,
            "{:.1f}".format(rssi) if rssi is not None else "?",
            decoded["lat"],
            decoded["lon"],
            decoded["hdg_deg"],
            decoded["spd_knots"],
        )
    )
    if has_obs:
        for i, target in enumerate(o):
            dist_m = float(target.get("dist_m", 0.0))
            ang = float(target.get("angle_deg", 0.0))
            if dist_m > 0:
                print("  Obs {:d}    : {:.2f} m  @  {:.1f} deg".format(i, dist_m, ang))
    else:
        print("  Obs      : none")


class LoRaReceiver:
    """RFM9x wrapper configured to match transmitter air settings."""

    def __init__(self):
        import adafruit_rfm9x

        self.spi = busio.SPI(LORA_SPI_SCK, MOSI=LORA_SPI_MOSI, MISO=LORA_SPI_MISO)

        self.cs = digitalio.DigitalInOut(LORA_CS)
        self.cs.direction = digitalio.Direction.OUTPUT

        self.rst = digitalio.DigitalInOut(LORA_RST)
        self.rst.direction = digitalio.Direction.OUTPUT

        self.rfm = adafruit_rfm9x.RFM9x(self.spi, self.cs, self.rst, LORA_FREQ_MHZ)
        self.rfm.signal_bandwidth = LORA_BW
        self.rfm.spreading_factor = LORA_SF
        self.rfm.coding_rate = LORA_CR
        self.rfm.enable_crc = LORA_CRC
        self.rfm.tx_power = LORA_TX_POWER
        self.rfm.receive_timeout = LORA_RX_TIMEOUT

        self.rx_count = 0
        self.err_count = 0
        self.last_rssi = None

    def receive_one(self):
        """Receive one packet with timeout."""
        try:
            raw = self.rfm.receive(timeout=LORA_RX_TIMEOUT, keep_listening=True)
        except Exception as e:
            self.err_count += 1
            print("  ! rfm.receive() exception:", e)
            return None, None

        if raw is None:
            return None, None

        self.rx_count += 1
        try:
            rssi = self.rfm.last_rssi
        except Exception:
            rssi = None
        self.last_rssi = rssi
        return bytes(raw), rssi

    def send_control(self, throttle, steering):
        """Send control packet matching USB semantics (type=control)."""
        try:
            if HAS_COMPACT:
                payload = pack_control_11(throttle, steering, 0)
            else:
                payload = json.dumps(
                    {"type": "control", "throttle": throttle, "steering": steering},
                    separators=(",", ":"),
                ).encode("utf-8")
            self.rfm.send(payload, keep_listening=True)
            return True
        except Exception as e:
            self.err_count += 1
            print("  ! rfm.send_control() error:", e)
            return False

    def send_stop(self):
        """Send neutral control (throttle 0, steering 0) — matches USB idle, does not pause radar."""
        try:
            if HAS_COMPACT:
                payload = pack_control_11(0.0, 0.0, 0)
            else:
                payload = json.dumps(
                    {"type": "control", "throttle": 0.0, "steering": 0.0},
                    separators=(",", ":"),
                ).encode("utf-8")
            self.rfm.send(payload, keep_listening=True)
            return True
        except Exception as e:
            self.err_count += 1
            print("  ! rfm.send_stop() error:", e)
            return False

    def status_str(self):
        rssi = (
            "{:.1f} dBm".format(self.last_rssi)
            if self.last_rssi is not None
            else "no packet yet"
        )
        return (
            "rx={:d}  err={:d}  last_rssi={}  "
            "[SF{}  BW{}k  CR4/{}  CRC={}  {:.1f} MHz]".format(
                self.rx_count,
                self.err_count,
                rssi,
                self.rfm.spreading_factor,
                self.rfm.signal_bandwidth // 1000,
                self.rfm.coding_rate,
                self.rfm.enable_crc,
                LORA_FREQ_MHZ,
            )
        )


CMD_CHARS = {"w", "a", "s", "d"}
THROTTLE_STEP = 0.2
_usb_line_buf = ""
_usb_msg_queue = []


def read_usb_char():
    """Non-blocking read of one character from USB CDC / stdin."""
    try:
        import usb_cdc

        for stream in (usb_cdc.console, getattr(usb_cdc, "data", None)):
            if stream is None:
                continue
            if stream.in_waiting:
                b = stream.read(1)
                if b:
                    return chr(b[0])
    except Exception:
        pass
    try:
        if supervisor.runtime.serial_bytes_available:
            c = sys.stdin.read(1)
            if c:
                return c
    except Exception:
        pass
    return None


def read_usb_json_line():
    """
    Non-blocking read of one JSON line from USB CDC / stdin.
    Returns decoded dict when a full line is available, otherwise None.
    """
    global _usb_line_buf, _usb_msg_queue
    if _usb_msg_queue:
        return _usb_msg_queue.pop(0)

    text = ""
    try:
        import usb_cdc

        for stream in (usb_cdc.console, getattr(usb_cdc, "data", None)):
            if stream is None:
                continue
            nw = stream.in_waiting
            if nw:
                text += stream.read(min(nw, 256)).decode("utf-8", "replace")
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
            return None

    if not text:
        return None

    for ch in text:
        # Keep keyboard shortcuts working when user types single-letter commands.
        if not _usb_line_buf and ch in CMD_CHARS:
            _usb_msg_queue.append({"_key": ch})
            continue

        if ch in ("\r", "\n"):
            line = _usb_line_buf.strip()
            _usb_line_buf = ""
            if not line:
                _usb_msg_queue.append({"_key": ch})
                continue
            try:
                obj = json.loads(line)
                if isinstance(obj, dict):
                    _usb_msg_queue.append(obj)
            except Exception:
                pass
            continue

        _usb_line_buf += ch
        if len(_usb_line_buf) > 512:
            _usb_line_buf = _usb_line_buf[-512:]

    if _usb_msg_queue:
        return _usb_msg_queue.pop(0)
    return None


def main():
    print()
    print("=" * 62)
    print("  ANCHOR LoRa RECEIVER + GUI SERIAL BRIDGE")
    print("  SF{}  BW125k  CR4/5  CRC={}  {:.1f} MHz  +{} dBm".format(
        LORA_SF, LORA_CRC, LORA_FREQ_MHZ, LORA_TX_POWER
    ))
    print("  Expects 12-byte binary struct -- ~1 packet per 30 s from TX")
    print("  Keys: w/s throttle, a/d rudder (impulse), Enter stop")
    print("=" * 62)
    print()

    print("Initialising RFM9x ... ", end="")
    try:
        radio = LoRaReceiver()
        print("OK")
        print("  Verified config:", radio.status_str())
    except Exception as e:
        print("FAILED:", e)
        print()
        print("  Checklist:")
        print("    SCK->GP18  MOSI->GP19  MISO->GP16  CS->GP17  RST->GP22")
        print("    VCC->3.3V  GND->GND")
        print("    adafruit_rfm9x.mpy  +  adafruit_bus_device  in /lib")
        while True:
            time.sleep(1)

    print()
    print("Listening for LoRa packets...")
    print()

    t_last_status = time.monotonic()
    packet_count = 0
    tx_count = 0
    ctrl_throttle = 0.0
    ctrl_steering = 0.0
    last_key_was_cr = False
    # Re-send non-zero throttle periodically so motion survives packet loss.
    throttle_running = False
    t_last_throttle_send = 0.0

    while True:
        while True:
            msg = read_usb_json_line()
            if msg is None:
                break
            if "_key" in msg:
                ch = msg["_key"]
            else:
                mtype = str(msg.get("type", "")).lower()
                if mtype == "stop":
                    ctrl_throttle = 0.0
                    ctrl_steering = 0.0
                    throttle_running = False
                    ok = radio.send_stop()
                    tx_count += 1
                    print("[TX #{:04d}] stop {}".format(tx_count, "OK" if ok else "FAIL"))
                    ch = None
                elif mtype == "control":
                    try:
                        t_in = float(msg.get("throttle", 0.0))
                    except Exception:
                        t_in = 0.0
                    try:
                        s_in = float(msg.get("steering", 0.0))
                    except Exception:
                        s_in = 0.0
                    if t_in > 1.0 and t_in <= 100.0:
                        t_in = t_in / 100.0
                    ctrl_throttle = max(0.0, min(1.0, t_in))
                    ctrl_steering = max(-1.0, min(1.0, s_in))
                    ok = radio.send_control(ctrl_throttle, ctrl_steering)
                    tx_count += 1
                    throttle_running = ctrl_throttle > 0.0
                    t_last_throttle_send = time.monotonic()
                    print(
                        "[TX #{:04d}] control throttle={:.2f} steering={:+.2f} {}".format(
                            tx_count, ctrl_throttle, ctrl_steering, "OK" if ok else "FAIL"
                        )
                    )
                    ch = None
                else:
                    # Ignore unsupported message types (including ping).
                    ch = None

            if ch is not None:
                if ch == "w":
                    last_key_was_cr = False
                    ctrl_throttle = min(1.0, ctrl_throttle + THROTTLE_STEP)
                    ok = radio.send_control(ctrl_throttle, ctrl_steering)
                    tx_count += 1
                    throttle_running = ctrl_throttle > 0.0
                    t_last_throttle_send = time.monotonic()
                    print(
                        "[TX #{:04d}] control throttle={:.2f} steering={:+.2f} {}".format(
                            tx_count, ctrl_throttle, ctrl_steering, "OK" if ok else "FAIL"
                        )
                    )
                elif ch == "s":
                    last_key_was_cr = False
                    ctrl_throttle = max(0.0, ctrl_throttle - THROTTLE_STEP)
                    ok = radio.send_control(ctrl_throttle, ctrl_steering)
                    tx_count += 1
                    throttle_running = ctrl_throttle > 0.0
                    t_last_throttle_send = time.monotonic()
                    print(
                        "[TX #{:04d}] control throttle={:.2f} steering={:+.2f} {}".format(
                            tx_count, ctrl_throttle, ctrl_steering, "OK" if ok else "FAIL"
                        )
                    )
                elif ch == "a":
                    last_key_was_cr = False
                    # Send a left-steering impulse, then immediately reset to center
                    # so subsequent w/s keystrokes do not re-send a latched rudder value.
                    ctrl_steering = -1.0
                    ok = radio.send_control(ctrl_throttle, ctrl_steering)
                    ctrl_steering = 0.0
                    tx_count += 1
                    print(
                        "[TX #{:04d}] control throttle={:.2f} steering=-1.00 (impulse) {}".format(
                            tx_count, ctrl_throttle, "OK" if ok else "FAIL"
                        )
                    )
                elif ch == "d":
                    last_key_was_cr = False
                    # Send a right-steering impulse, then immediately reset to center
                    # so subsequent w/s keystrokes do not re-send a latched rudder value.
                    ctrl_steering = 1.0
                    ok = radio.send_control(ctrl_throttle, ctrl_steering)
                    ctrl_steering = 0.0
                    tx_count += 1
                    print(
                        "[TX #{:04d}] control throttle={:.2f} steering=+1.00 (impulse) {}".format(
                            tx_count, ctrl_throttle, "OK" if ok else "FAIL"
                        )
                    )
                elif ch == "\r":
                    last_key_was_cr = True
                    ctrl_throttle = 0.0
                    ctrl_steering = 0.0
                    throttle_running = False
                    ok = radio.send_stop()
                    tx_count += 1
                    print("[TX #{:04d}] stop {}".format(tx_count, "OK" if ok else "FAIL"))
                elif ch == "\n":
                    if last_key_was_cr:
                        last_key_was_cr = False
                    else:
                        ctrl_throttle = 0.0
                        ctrl_steering = 0.0
                        throttle_running = False
                        ok = radio.send_stop()
                        tx_count += 1
                        print("[TX #{:04d}] stop {}".format(tx_count, "OK" if ok else "FAIL"))

        raw, rssi = radio.receive_one()
        
        # Re-send throttle while running so one control command is enough.
        if throttle_running and ctrl_throttle > 0.0:
            now_t = time.monotonic()
            if (now_t - t_last_throttle_send) >= THROTTLE_RESEND_INTERVAL:
                ok = radio.send_control(ctrl_throttle, ctrl_steering)
                tx_count += 1
                t_last_throttle_send = now_t
                print(
                    "[TX #{:04d}] hold throttle={:.2f} steering={:+.2f} {}".format(
                        tx_count, ctrl_throttle, ctrl_steering, "OK" if ok else "FAIL"
                    )
                )
        
        if raw is not None:
            packet_count += 1
            if len(raw) >= PACKET_LEN:
                decoded = decode_packet(raw)
                if decoded is not None:
                    pretty_print_packet(decoded, rssi, packet_count)
                    # Backend-compatible JSON line for GUI pipeline
                    emit_gui_json(decoded)
                else:
                    print(
                        "[PKT #{:04d}]  RSSI={}  decode error  raw={}".format(
                            packet_count,
                            "{:.1f}".format(rssi) if rssi is not None else "?",
                            raw.hex(),
                        )
                    )
            else:
                try:
                    obj = json.loads(raw.decode("utf-8"))
                    print(
                        "[PKT #{:04d}]  RSSI={}  JSON: {}".format(
                            packet_count,
                            "{:.1f}".format(rssi) if rssi is not None else "?",
                            obj,
                        )
                    )
                except Exception:
                    print(
                        "[PKT #{:04d}]  RSSI={}  short/unknown {} bytes: {}".format(
                            packet_count,
                            "{:.1f}".format(rssi) if rssi is not None else "?",
                            len(raw),
                            raw.hex(),
                        )
                    )

        now = time.monotonic()
        if (now - t_last_status) >= STATUS_INTERVAL:
            t_last_status = now
            print("[STATUS]  tx_sent={}  {}".format(tx_count, radio.status_str()))
        time.sleep(0.005)


try:
    main()
except KeyboardInterrupt:
    print("\nStopped.")
