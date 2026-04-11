"""
=============================================================================
ANCHOR — LoRa RFM9x RECEIVER  (guaranteed-match for transmitter code.py)
=============================================================================
Transmitter sends an 11-byte binary struct every ~30 s (due to time.sleep(30)
after each TX).  This receiver decodes that exact struct and also handles
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
=============================================================================
"""

import board
import busio
import digitalio
import struct
import json
import time
import math

# Pin / radio constants (must match transmitter)
LORA_SPI_SCK    = board.GP18
LORA_SPI_MOSI   = board.GP19
LORA_SPI_MISO   = board.GP16
LORA_CS         = board.GP17
LORA_RST        = board.GP22
LORA_FREQ_MHZ   = 915.0          # must be identical on TX
LORA_SF         = 11             # spreading factor - must match TX
LORA_BW         = 125000         # Hz - must match TX
LORA_CR         = 5              # coding rate denominator (4/5) - must match TX
LORA_CRC        = True           # must match TX
LORA_TX_POWER   = 17             # dBm - only matters if we send ACKs
LORA_RX_TIMEOUT = 0.5            # seconds per rfm.receive() poll
                                  # Longer poll = fewer wasted SPI cycles while waiting.
                                  # Safe because transmitter sleeps 30 s between packets.

PACKET_LEN      = 11             # bytes - must match LORA_MAX_PACKET on transmitter
STRUCT_FMT      = "<eeBBbBbBB"   # little-endian: float16 lat, float16 lon,
                                  # uint8 hdg, uint8 r0, int8 a0,
                                  # uint8 r1, int8 a1, uint8 spd, uint8 bat
STRUCT_SIZE     = struct.calcsize(STRUCT_FMT)   # = 11

STATUS_INTERVAL = 10.0           # print a "still listening" line every N seconds


# Decode helpers

def decode_packet(raw_bytes):
    """
    Decode the 11-byte binary telemetry packet from the transmitter.

    Returns a plain dict with human-readable / float fields, or None on error.
    Mirrors pack_telemetry_binary() in the transmitter exactly.
    """
    if len(raw_bytes) < STRUCT_SIZE:
        return None
    try:
        lat_f, lon_f, hdg_b, r0, a0, r1, a1, spd_b, bat_b = struct.unpack(
            STRUCT_FMT, raw_bytes[:STRUCT_SIZE]
        )
    except Exception as e:
        print("  ! struct.unpack failed:", e)
        return None

    # Reverse the fixed-point encodings applied by the transmitter
    hdg_deg = round(hdg_b * 359.0 / 255.0, 1)

    # int8 angle: struct gives a signed Python int already
    obs0_dist_m  = float(r0)
    obs0_angle   = int(a0)
    obs1_dist_m  = float(r1)
    obs1_angle   = int(a1)

    spd_knots    = float(spd_b)
    bat_pct      = int(bat_b)

    # float16 lat/lon decoded by struct as Python float
    lat = float(lat_f)
    lon = float(lon_f)

    return {
        "lat":      round(lat, 5),
        "lon":      round(lon, 5),
        "hdg_deg":  hdg_deg,
        "obs": [
            {"dist_m": obs0_dist_m, "angle_deg": obs0_angle},
            {"dist_m": obs1_dist_m, "angle_deg": obs1_angle},
        ],
        "spd_knots": spd_knots,
        "bat_pct":   bat_pct,
    }


def pretty_print_packet(decoded, rssi, packet_num):
    """Print a single decoded telemetry packet in a readable format."""
    o = decoded["obs"]
    print(
        "[PKT #{:04d}]  RSSI={:>7} dBm\n"
        "  Position : lat={:.5f}  lon={:.5f}\n"
        "  Heading  : {:.1f} deg\n"
        "  Speed    : {:.1f} kn    Battery: {}%\n"
        "  Obs 0    : {:.0f} m  @  {:+d} deg\n"
        "  Obs 1    : {:.0f} m  @  {:+d} deg".format(
            packet_num,
            "{:.1f}".format(rssi) if rssi is not None else "?",
            decoded["lat"],
            decoded["lon"],
            decoded["hdg_deg"],
            decoded["spd_knots"],
            decoded["bat_pct"],
            o[0]["dist_m"],
            o[0]["angle_deg"],
            o[1]["dist_m"],
            o[1]["angle_deg"],
        )
    )


# Radio wrapper

class LoRaReceiver:
    """
    Thin wrapper around adafruit_rfm9x.RFM9x configured to match the
    transmitter's exact air-interface settings.
    """

    def __init__(self):
        import adafruit_rfm9x

        self.spi = busio.SPI(LORA_SPI_SCK, MOSI=LORA_SPI_MOSI, MISO=LORA_SPI_MISO)

        self.cs  = digitalio.DigitalInOut(LORA_CS)
        self.cs.direction  = digitalio.Direction.OUTPUT

        self.rst = digitalio.DigitalInOut(LORA_RST)
        self.rst.direction = digitalio.Direction.OUTPUT

        # Construct radio
        self.rfm = adafruit_rfm9x.RFM9x(
            self.spi, self.cs, self.rst, LORA_FREQ_MHZ
        )

        # Air-interface - EVERY setting must match the transmitter
        self.rfm.signal_bandwidth = LORA_BW       # 125 000 Hz
        self.rfm.spreading_factor = LORA_SF       # 11
        self.rfm.coding_rate      = LORA_CR       # 4/5  (value=5 in adafruit lib)
        self.rfm.enable_crc       = LORA_CRC      # True - drops corrupt frames
        self.rfm.tx_power         = LORA_TX_POWER # +17 dBm

        # Counters
        self.rx_count   = 0
        self.err_count  = 0
        self.last_rssi  = None

    def receive_one(self):
        """
        Block for up to LORA_RX_TIMEOUT seconds waiting for a packet.

        Returns (raw_bytes, rssi_float) or (None, None) on timeout/error.

        keep_listening=True keeps the radio in continuous RX mode after the
        call returns. Without it the chip goes back to STANDBY and the next
        packet preamble is missed while Python is executing.
        """
        try:
            raw = self.rfm.receive(
                timeout=LORA_RX_TIMEOUT,
                keep_listening=True,
            )
        except Exception as e:
            self.err_count += 1
            print("  ! rfm.receive() exception:", e)
            return None, None

        if raw is None:
            return None, None   # normal timeout - no packet this window

        self.rx_count += 1
        try:
            rssi = self.rfm.last_rssi
        except Exception:
            rssi = None
        self.last_rssi = rssi
        return bytes(raw), rssi

    def send_json(self, obj):
        """
        Send a JSON dict back to the boat. The transmitter's try_receive_json()
        will decode this. Keep payload small (110 bytes max).
        """
        try:
            payload = json.dumps(obj, separators=(",", ":")).encode("utf-8")
            self.rfm.send(payload)
            return True
        except Exception as e:
            self.err_count += 1
            print("  ! rfm.send() error:", e)
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


# Main

def main():
    print()
    print("=" * 62)
    print("  ANCHOR LoRa RECEIVER")
    print("  SF{}  BW125k  CR4/5  CRC={}  {:.1f} MHz  +{} dBm".format(
        LORA_SF, LORA_CRC, LORA_FREQ_MHZ, LORA_TX_POWER
    ))
    print("  Expects 11-byte binary struct -- ~1 packet per 30 s from TX")
    print("=" * 62)
    print()

    # Init radio
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
    print("Listening ... (TX sleeps 30 s between packets -- be patient)")
    print()

    t_last_status = time.monotonic()
    packet_count  = 0

    while True:
        # 1. Poll for packet
        raw, rssi = radio.receive_one()

        # 2. Decode if something arrived
        if raw is not None:
            packet_count += 1

            # Try binary telemetry first (primary format)
            if len(raw) >= STRUCT_SIZE:
                decoded = decode_packet(raw)
                if decoded is not None:
                    pretty_print_packet(decoded, rssi, packet_count)

                    # Uncomment to send a JSON ACK back to the boat:
                    # radio.send_json({"type": "ack", "n": packet_count})

                else:
                    # Correct length but unpack failed
                    print(
                        "[PKT #{:04d}]  RSSI={}  decode error  raw={}".format(
                            packet_count,
                            "{:.1f}".format(rssi) if rssi else "?",
                            raw.hex(),
                        )
                    )
            else:
                # Packet shorter than 11 bytes: try JSON (control / ping)
                try:
                    obj = json.loads(raw.decode("utf-8"))
                    print(
                        "[PKT #{:04d}]  RSSI={}  JSON: {}".format(
                            packet_count,
                            "{:.1f}".format(rssi) if rssi else "?",
                            obj,
                        )
                    )
                except Exception:
                    # Unknown short packet - print hex for diagnostics
                    print(
                        "[PKT #{:04d}]  RSSI={}  short/unknown {} bytes: {}".format(
                            packet_count,
                            "{:.1f}".format(rssi) if rssi else "?",
                            len(raw),
                            raw.hex(),
                        )
                    )

        # 3. Periodic "still alive" status line
        now = time.monotonic()
        if (now - t_last_status) >= STATUS_INTERVAL:
            t_last_status = now
            print("[STATUS]  {}".format(radio.status_str()))


try:
    main()
except KeyboardInterrupt:
    print("\nStopped.")
