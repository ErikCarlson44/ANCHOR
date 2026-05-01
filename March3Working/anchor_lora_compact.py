"""
ANCHOR LoRa compact air protocol — 11 bytes per packet (no JSON key names on RF).

Copy to CIRCUITPY on boat + ground Picos next to code.py (same folder as code runs from).

Byte layout (big-endian):
  0xA1  telemetry: lat/lon deltas from (32.0, -117.0), heading (uint8),
                   battery, obstacle0(distance_m, angle_deg), obstacle1(distance_m, angle_deg)
  0xC1  command:  throttle 0..255, steering -128..127, flags, 7 pad
  0xC2  ping:     magic + 10 zero pad
  0xC3  pong:     magic + 10 zero pad

flags (command): 0x01 stop/pause, 0x02 start/resume, 0x04 reset
"""

import struct

TEL_MAGIC = 0xA1
CMD_MAGIC = 0xC1
PING_MAGIC = 0xC2
PONG_MAGIC = 0xC3
LORA_BIN_LEN = 11

LAT_BASE = 32.0
LON_BASE = -117.0

FLAG_STOP = 0x01
FLAG_START = 0x02
FLAG_RESET = 0x04


def _encode_obs_slot(obs_item):
    """Encode one obstacle as uint8 distance meters + int8 angle degrees."""
    if not isinstance(obs_item, (list, tuple)) or len(obs_item) < 2:
        return 0, 0
    try:
        dist_m = float(obs_item[0])
    except (TypeError, ValueError):
        dist_m = 0.0
    try:
        ang_deg = float(obs_item[1])
    except (TypeError, ValueError):
        ang_deg = 0.0
    if dist_m <= 0:
        return 0, 0
    dist_u8 = max(1, min(255, int(round(dist_m))))
    ang_i8 = max(-128, min(127, int(round(ang_deg))))
    return dist_u8, ang_i8


def pack_telemetry_11(lat, lon, hdg, spd, bat, sats=0, obs=None):
    lat_d = max(-32768, min(32767, int(round((float(lat) - LAT_BASE) * 10000))))
    lon_d = max(-32768, min(32767, int(round((float(lon) - LON_BASE) * 10000))))
    hdg_u = int(round(float(hdg) * 255.0 / 359.0)) & 0xFF
    bat_u = max(0, min(100, int(bat)))
    obs = obs or []
    # Keep nearest 2 obstacles by distance.
    valid_obs = []
    for item in obs:
        if isinstance(item, (list, tuple)) and len(item) >= 2:
            try:
                d = float(item[0])
            except (TypeError, ValueError):
                d = 0.0
            if d > 0:
                valid_obs.append(item)
    valid_obs.sort(key=lambda x: float(x[0]))
    while len(valid_obs) < 2:
        valid_obs.append((0, 0))
    r0_u, a0_i = _encode_obs_slot(valid_obs[0])
    r1_u, a1_i = _encode_obs_slot(valid_obs[1])
    # 1 + 2+2 + 1+1 + 1+1 + 1+1 = 11
    return struct.pack(
        ">BhhBBBbBb", TEL_MAGIC, lat_d, lon_d, hdg_u, bat_u, r0_u, a0_i, r1_u, a1_i
    )


def unpack_telemetry_11(buf):
    if len(buf) != LORA_BIN_LEN or buf[0] != TEL_MAGIC:
        return None
    _, lat_d, lon_d, hdg_u, bat_u, r0_u, a0_i, r1_u, a1_i = struct.unpack(">BhhBBBbBb", buf)
    obs = []
    if r0_u > 0:
        obs.append([float(r0_u), float(a0_i), 1.0])
    if r1_u > 0:
        obs.append([float(r1_u), float(a1_i), 1.0])
    return {
        "lat": LAT_BASE + lat_d / 10000.0,
        "lon": LON_BASE + lon_d / 10000.0,
        "hdg": hdg_u * 359.0 / 255.0,
        "spd": 0.0,
        "bat": bat_u,
        "sats": 0,
        "obs": obs,
    }


def pack_ping_11():
    return struct.pack(">B10s", PING_MAGIC, b"\x00" * 10)


def pack_pong_11():
    return struct.pack(">B10s", PONG_MAGIC, b"\x00" * 10)


def is_ping_11(buf):
    return len(buf) == LORA_BIN_LEN and buf[0] == PING_MAGIC


def is_pong_11(buf):
    return len(buf) == LORA_BIN_LEN and buf[0] == PONG_MAGIC


def pack_control_11(throttle, steering, flags=0):
    try:
        t = float(throttle)
    except (TypeError, ValueError):
        t = 0.0
    try:
        s = float(steering)
    except (TypeError, ValueError):
        s = 0.0
    t_u = max(0, min(255, int(round(t * 255.0))))
    s_i = max(-128, min(127, int(round(s * 127.0))))
    return struct.pack(">BBbB", CMD_MAGIC, t_u, s_i, flags & 0xFF) + b"\x00" * 7


def unpack_cmd_11(buf):
    if len(buf) != LORA_BIN_LEN or buf[0] != CMD_MAGIC:
        return None
    _, t_u, s_i, fl = struct.unpack(">BBbB", buf[:4])
    if fl & FLAG_STOP:
        return {"type": "stop"}
    if fl & FLAG_START:
        return {"type": "start"}
    if fl & FLAG_RESET:
        return {"type": "reset"}
    return {
        "type": "control",
        "throttle": t_u / 255.0,
        "steering": (s_i / 127.0) if s_i != 0 else 0.0,
    }
