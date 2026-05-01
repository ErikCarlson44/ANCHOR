"""
=============================================================================
ANCHOR — BOAT USB → GUI LINK TEST (CircuitPython)  →  save as code.py on BOAT
=============================================================================
Purpose: Hard-coded telemetry out the boat Pico's USB cable so you can verify
the PC backend + React GUI read the same JSON as real code.py.

This is NOT full boat firmware (no GPS, motors, LoRa, radar). For RF tests use
lora_test_code / lora_ground_bridge_code instead.

Steps:
  1. Copy this file to the boat's CIRCUITPY drive as code.py (replace main code.py).
  2. USB the boat Pico to the PC. Close Thonny on that COM port.
  3. Backend + frontend: pick that COM port, Pico / JSON, 115200 baud, Connect.
  4. Expect live map/telemetry; heading sweeps 0–360°, lat/lon drift slowly.

Handshake: answers {"type":"ping"} with {"type":"pong"} (normal Connect button).
Telemetry lines match FastAPI _read_serial (include "lat", no "type" key).
=============================================================================
"""

import json
import supervisor
import sys
import time

# --- Edit these if you want different fake numbers ---
LAT0 = 32.7872
LON0 = -117.2350
HDG0 = 0.0
SPD_FIXED = 4.2
BAT_FIXED = 88
SATS_FIXED = 12
OBS_FIXED = [[12.5, 15.0, 1.0], [8.0, -30.0, 1.0]]

TELEMETRY_HZ = 5.0
_heading_step = 3.0
_lat_step = 0.00002
_lon_step = 0.00001

_telemetry_tick = 0.0
_rx_buf = ""


def _emit_line(obj):
    try:
        print(json.dumps(obj, separators=(",", ":")))
    except TypeError:
        print(json.dumps(obj))


def _emit_telemetry(lat, lon, hdg):
    _emit_line(
        {
            "lat": round(lat, 6),
            "lon": round(lon, 6),
            "hdg": round(hdg, 1),
            "spd": round(SPD_FIXED, 1),
            "bat": int(BAT_FIXED),
            "sats": int(SATS_FIXED),
            "obs": OBS_FIXED,
        }
    )


def _drain_usb_for_ping():
    global _rx_buf
    try:
        while supervisor.runtime.serial_bytes_available:
            ch = sys.stdin.read(1)
            if not ch:
                break
            if ch in "\n\r":
                if _rx_buf:
                    line = _rx_buf.strip()
                    _rx_buf = ""
                    if line.startswith("{"):
                        try:
                            o = json.loads(line)
                            if isinstance(o, dict) and o.get("type") == "ping":
                                _emit_line({"type": "pong"})
                        except (ValueError, TypeError, OSError):
                            pass
                else:
                    _rx_buf = ""
            elif len(_rx_buf) < 256:
                _rx_buf += ch
            else:
                _rx_buf = ""
    except Exception:
        pass


def main():
    global _telemetry_tick
    lat = LAT0
    lon = LON0
    hdg = HDG0

    for _ in range(3):
        _emit_telemetry(lat, lon, hdg)
        time.sleep(0.05)

    interval = 1.0 / TELEMETRY_HZ
    while True:
        _drain_usb_for_ping()
        now = time.monotonic()
        if now - _telemetry_tick >= interval:
            _telemetry_tick = now
            hdg = (hdg + _heading_step) % 360.0
            lat += _lat_step
            lon += _lon_step
            _emit_telemetry(lat, lon, hdg)
        time.sleep(0.01)


main()
