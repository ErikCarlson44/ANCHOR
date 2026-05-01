"""
=============================================================================
ANCHOR — LORA GROUND BRIDGE (RFM95W + USB)  →  save as code.py on GROUND Pico
=============================================================================
Connect this Pico's USB to the PC. In ANCHOR GUI, select THIS Pico's COM port
and baud 115200 (CircuitPython REPL default) or 9600 if you changed USB CDC.

This radio must see the BOAT Pico running lora_test_code.py with MATCHING
LORA_* settings below.

Flow:
  - LoRa RX from boat → 11-byte compact telemetry (anchor_lora_compact.py) or JSON;
    expands to one JSON line on USB for FastAPI / GUI (must include "lat").
  - USB from PC: ping / control / stop / start / reset → 11-byte LoRa to boat.

Needs anchor_lora_compact.py on CIRCUITPY (same folder as code.py). Boat code.py uses same file.

MTDOT-915: not used on this link; mDot is LoRaWAN/AT, not compatible with this
raw LoRa. Use this Pico+RFM95 as the RF dongle for the GUI test.

LoRa (RFM95W) — same as boat:
  MISO  -> GP16
  CS    -> GP17
  SCK   -> GP18
  MOSI  -> GP19
  RST   -> GP22
  G0    -> GP26   (DIO0; optional for IRQ — this script uses polling RX)
  VIN   -> 3.3V
  GND   -> GND

Libraries: adafruit_rfm9x.mpy + adafruit_bus_device/
=============================================================================
"""

import board
import busio
import digitalio
import json
import supervisor
import sys
import time

try:
    from anchor_lora_compact import (
        pack_ping_11,
        pack_control_11,
        FLAG_STOP,
        FLAG_START,
        FLAG_RESET,
        unpack_telemetry_11,
        is_pong_11,
        LORA_BIN_LEN,
    )

    COMPACT_LORA = True
except ImportError:
    COMPACT_LORA = False
    LORA_BIN_LEN = 11

# =============================================================================
# LORA RF — MUST MATCH lora_test_code.py EXACTLY
# =============================================================================
try:
    from lora_hw_config import (
        LORA_SPI_SCK as SPI_SCK,
        LORA_SPI_MOSI as SPI_MOSI,
        LORA_SPI_MISO as SPI_MISO,
        LORA_CS as CS_PIN,
        LORA_RST as RESET_PIN,
        LORA_FREQ_MHZ as RADIO_FREQ_MHZ,
        LORA_SPI_BAUD,
    )
except ImportError:
    SPI_SCK = board.GP18
    SPI_MOSI = board.GP19
    SPI_MISO = board.GP16
    CS_PIN = board.GP17
    RESET_PIN = board.GP22
    RADIO_FREQ_MHZ = 915.0
    LORA_SPI_BAUD = 500_000

LORA_G0_PIN = board.GP26

TX_POWER_DBM = 13
SPREADING_FACTOR = 11
SIGNAL_BANDWIDTH = 125000
CODING_RATE = 5

LORA_PROTO = "ANCHOR_LORA_1"
RX_TIMEOUT_S = 0.35  # match lora_test_code.py so RX windows line up better
PACKET_MAX_LEN = 220

# Print every N seconds if no packets (confirms ground script is alive / listening)
HEARTBEAT_S = 12.0
# If True, extra # debug lines (Thonny). Set False for cleaner GUI serial.
DEBUG_LORA_RX = False

# =============================================================================
def apply_radio_settings(rfm):
    rfm.signal_bandwidth = SIGNAL_BANDWIDTH
    rfm.spreading_factor = SPREADING_FACTOR
    rfm.coding_rate = CODING_RATE
    rfm.enable_crc = True
    rfm.tx_power = TX_POWER_DBM


def send_json(rfm, obj):
    line = json.dumps(obj) + "\n"
    rfm.send(bytes(line[:PACKET_MAX_LEN], "utf-8"))


def forward_usb_line_to_lora(rfm, line_str):
    """PC/GUI sent a JSON line — forward over LoRa (11-byte compact when available)."""
    line_str = line_str.strip()
    if not line_str:
        return
    try:
        cmd = json.loads(line_str)
    except (ValueError, TypeError, OSError):
        return
    if COMPACT_LORA:
        if cmd.get("type") == "ping":
            rfm.send(pack_ping_11())
            print("LoRa TX: ping (11B)")
            return
        if cmd.get("type") == "control":
            rfm.send(pack_control_11(cmd.get("throttle"), cmd.get("steering"), 0))
            print("LoRa TX: control (11B)")
            return
        if cmd.get("type") in ("stop", "pause"):
            rfm.send(pack_control_11(0, 0, FLAG_STOP))
            print("LoRa TX: stop (11B)")
            return
        if cmd.get("type") in ("start", "resume"):
            rfm.send(pack_control_11(0, 0, FLAG_START))
            print("LoRa TX: start (11B)")
            return
        if cmd.get("type") == "reset":
            rfm.send(pack_control_11(0, 0, FLAG_RESET))
            print("LoRa TX: reset (11B)")
            return
    if cmd.get("type") == "ping":
        send_json(
            rfm,
            {"type": "ping", "src": "ground", "proto": LORA_PROTO},
        )
        print("LoRa TX: ping (JSON)")


def handle_lora_rx(rfm, raw_packet):
    """Print JSON to USB for ANCHOR backend (telemetry or pong)."""
    if not raw_packet:
        return
    if DEBUG_LORA_RX:
        try:
            rssi = rfm.last_rssi
        except Exception:
            rssi = "?"
        print("# LoRa RF: {} bytes  RSSI={} dBm".format(len(raw_packet), rssi))

    if COMPACT_LORA and len(raw_packet) == LORA_BIN_LEN:
        if is_pong_11(raw_packet):
            try:
                print(
                    json.dumps(
                        {"type": "pong", "src": "boat", "proto": LORA_PROTO},
                        separators=(",", ":"),
                    )
                )
            except TypeError:
                print(
                    json.dumps({"type": "pong", "src": "boat", "proto": LORA_PROTO})
                )
            return
        tel = unpack_telemetry_11(raw_packet)
        if tel:
            try:
                print(json.dumps(tel, separators=(",", ":")))
            except TypeError:
                print(json.dumps(tel))
            return

    # CircuitPython / MicroPython: bytes.decode() often has no errors= keyword
    try:
        text = raw_packet.decode("utf-8").strip()
    except UnicodeError:
        text = raw_packet.decode("latin-1").strip()
    except Exception as ex:
        print("# LoRa decode err:", str(ex)[:80])
        return
    if not text:
        print("# LoRa: empty after UTF-8 decode/strip (raw len={})".format(len(raw_packet)))
        return

    if DEBUG_LORA_RX:
        print("# LoRa text: len={} start={!r}".format(len(text), text[:72]))

    # CircuitPython json.loads raises ValueError on errors, not always JSONDecodeError
    try:
        obj = json.loads(text)
    except (ValueError, TypeError, OSError) as ex:
        print("# LoRa JSON err:", str(ex)[:100])
        print("# LoRa raw:", text[:160])
        return

    if not isinstance(obj, dict):
        print("# LoRa: JSON was not an object:", type(obj))
        return

    # Handshake: forward pong to PC
    if obj.get("type") == "pong":
        try:
            print(json.dumps(obj, separators=(",", ":")))
        except TypeError:
            print(json.dumps(obj))
        return

    # Telemetry: must include lat for backend (one bare JSON line for ANCHOR GUI)
    if "lat" in obj:
        try:
            print(json.dumps(obj, separators=(",", ":")))
        except TypeError:
            print(json.dumps(obj))
        return

    # Optional: log other protocol packets to USB debug
    if obj.get("proto") == LORA_PROTO:
        try:
            line = json.dumps(obj, separators=(",", ":"))[:200]
        except TypeError:
            line = json.dumps(obj)[:200]
        print("# LoRa:", line)
        return

    if DEBUG_LORA_RX:
        print("# LoRa JSON no lat/proto match, keys:", list(obj.keys()))


def main():
    print("\n" + "=" * 50)
    print("ANCHOR LORA GROUND BRIDGE  proto={}".format(LORA_PROTO))
    print("USB to PC | LoRa to boat")
    print("=" * 50)

    spi = busio.SPI(SPI_SCK, MOSI=SPI_MOSI, MISO=SPI_MISO)
    cs = digitalio.DigitalInOut(CS_PIN)
    reset = digitalio.DigitalInOut(RESET_PIN)

    try:
        import adafruit_rfm9x
    except ImportError:
        print("ERROR: adafruit_rfm9x missing from /lib")
        while True:
            time.sleep(1)

    try:
        try:
            rfm = adafruit_rfm9x.RFM9x(
                spi, cs, reset, RADIO_FREQ_MHZ, baudrate=LORA_SPI_BAUD
            )
        except TypeError:
            rfm = adafruit_rfm9x.RFM9x(spi, cs, reset, RADIO_FREQ_MHZ)
    except Exception as e:
        print("ERROR: RFM9x init:", e)
        while True:
            time.sleep(1)

    apply_radio_settings(rfm)
    print(
        "Radio OK  {:.1f} MHz  SF{}  BW{}  CR4/{}  SPI={} Hz".format(
            RADIO_FREQ_MHZ,
            SPREADING_FACTOR,
            SIGNAL_BANDWIDTH,
            CODING_RATE,
            LORA_SPI_BAUD,
        )
    )
    print("Ready. 915/SF11/BW125/CRC + anchor_lora_compact (11B air).")
    if COMPACT_LORA:
        print("COMPACT: telemetry/commands are 11 bytes on LoRa; USB to PC stays JSON.")
    else:
        print("WARN: add anchor_lora_compact.py for 11-byte GUI link.")
    print("=" * 50 + "\n")

    usb_buf = ""
    last_hb = time.monotonic()
    last_rx = time.monotonic()

    while True:
        # ----- USB from PC (GUI / Thonny) -----
        while supervisor.runtime.serial_bytes_available:
            try:
                ch = sys.stdin.read(1)
            except Exception:
                break
            if ch in ("\n", "\r"):
                if usb_buf:
                    forward_usb_line_to_lora(rfm, usb_buf)
                usb_buf = ""
            elif ch:
                usb_buf += ch

        # ----- LoRa from boat -----
        rfm.receive_timeout = RX_TIMEOUT_S
        try:
            pkt = rfm.receive()
            if pkt:
                last_rx = time.monotonic()
            handle_lora_rx(rfm, pkt)
        except Exception as e:
            print("# RX err:", e)

        now = time.monotonic()
        if (now - last_hb) >= HEARTBEAT_S:
            last_hb = now
            print(
                "# heartbeat: listening  (last packet {:.0f}s ago)".format(
                    now - last_rx
                )
            )

        time.sleep(0.01)


try:
    main()
except KeyboardInterrupt:
    print("\nStopped.")
except Exception as e:
    print("FATAL:", e)
    raise
