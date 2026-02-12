import time

SIM_NMEA = [
    b"$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*5B\r\n",
    b"$GNRMC,123520,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*4B\r\n",
]

def nmea_deg_to_decimal(raw, hemi):
    if not raw:
        return None
    v = float(raw)
    deg = int(v // 100)
    minutes = v - deg * 100
    dec = deg + minutes / 60.0
    if hemi in ("S", "W"):
        dec = -dec
    return dec

def parse_gga(s):
    p = s.split(",")
    if len(p) < 10:
        return None
    fixq = int(p[6] or "0")
    sats = int(p[7] or "0") if p[7] else None
    lat = nmea_deg_to_decimal(p[2], p[3])
    lon = nmea_deg_to_decimal(p[4], p[5])
    alt = float(p[9]) if p[9] else None
    return fixq, sats, lat, lon, alt

def parse_rmc(s):
    p = s.split(",")
    if len(p) < 10:
        return None
    status = p[2]
    lat = nmea_deg_to_decimal(p[3], p[4])
    lon = nmea_deg_to_decimal(p[5], p[6])
    spd_knots = float(p[7]) if p[7] else None
    spd_mps = spd_knots * 0.514444 if spd_knots is not None else None
    return status, lat, lon, spd_mps

print("GPS SIM mode (no hardware).")

fix_ok = False
lat = lon = alt = speed = None
sats = None

i = 0
while True:
    s = SIM_NMEA[i % len(SIM_NMEA)].decode("ascii").strip()

    if s.startswith("$GNGGA"):
        out = parse_gga(s)
        if out:
            fixq, sats, lat, lon, alt = out
            fix_ok = fixq > 0

    elif s.startswith("$GNRMC"):
        out = parse_rmc(s)
        if out:
            status, lat2, lon2, speed = out
            if lat2 is not None: lat = lat2
            if lon2 is not None: lon = lon2
            fix_ok = fix_ok or (status == "A")

    if fix_ok and lat is not None and lon is not None:
        msg = "FIX lat=%.6f lon=%.6f" % (lat, lon)
        if alt is not None: msg += " alt=%.1fm" % alt
        if sats is not None: msg += " sats=%d" % sats
        if speed is not None: msg += " speed=%.2f m/s" % speed
        print(msg)
    else:
        print("NO FIX (sim)")

    i += 1
    time.sleep(1)
