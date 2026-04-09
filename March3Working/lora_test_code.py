"""
=============================================================================
ANCHOR — LORA BOAT TEST (RFM95W on Pico)  →  save as code.py on BOAT Pico
=============================================================================
Pairs with: lora_ground_bridge_code.py on a SECOND Pico (USB to PC for GUI).

RF link: raw LoRa via adafruit_rfm9x. MTDOT-915 (mDot) does NOT use this air
protocol; use a ground Pico + RFM95 as the USB bridge to ANCHOR.

LoRa: lora_hw_config.py on CIRCUITPY (same as boat code.py).

Libraries: adafruit_rfm9x.mpy + adafruit_bus_device/ in /lib

KEEP radio settings identical to lora_ground_bridge_code.py (see LORA_* below).
=============================================================================
"""

import board
import busio
import digitalio
import json
import time

# =============================================================================
# LORA RF — lora_hw_config.py (same harness as boat + ground bridge)
# =============================================================================
try:
    from lora_hw_config import (
        LORA_SPI_SCK,
        LORA_SPI_MOSI,
        LORA_SPI_MISO,
        LORA_CS,
        LORA_RST,
        LORA_FREQ_MHZ,
        LORA_SPI_BAUD,
    )
except ImportError:
    LORA_SPI_SCK = board.GP18
    LORA_SPI_MOSI = board.GP19
    LORA_SPI_MISO = board.GP16
    LORA_CS = board.GP17
    LORA_RST = board.GP22
    LORA_FREQ_MHZ = 915.0
    LORA_SPI_BAUD = 500_000

TX_POWER_DBM = 13
SPREADING_FACTOR = 7
SIGNAL_BANDWIDTH = 125000
CODING_RATE = 5

LORA_PROTO = "ANCHOR_LORA_1"
BEACON_INTERVAL_S = 3.0
RX_TIMEOUT_S = 0.35
PACKET_MAX_LEN = 220

# =============================================================================
def apply_radio_settings(rfm):
    rfm.signal_bandwidth = SIGNAL_BANDWIDTH
    rfm.spreading_factor = SPREADING_FACTOR
    rfm.coding_rate = CODING_RATE
    rfm.enable_crc = True
    rfm.tx_power = TX_POWER_DBM


def send_json(rfm, obj):
    line = json.dumps(obj) + "\n"
    data = bytes(line[:PACKET_MAX_LEN], "utf-8")
    rfm.send(data)


def handle_rx(rfm, raw_packet):
    if not raw_packet:
        return
    try:
        text = raw_packet.decode("utf-8", errors="replace").strip()
    except Exception:
        return
    if not text:
        return
    print("LoRa RX:", text[:120])

    try:
        obj = json.loads(text)
    except (json.JSONDecodeError, TypeError):
        if text.upper().startswith("PING"):
            send_json(
                rfm,
                {"type": "pong", "src": "boat", "proto": LORA_PROTO},
            )
            print("LoRa TX: pong (plain PING)")
        return

    if obj.get("type") == "ping":
        send_json(
            rfm,
            {"type": "pong", "src": "boat", "proto": LORA_PROTO},
        )
        print("LoRa TX: pong")


def main():
    print("\n" + "=" * 50)
    print("ANCHOR LORA BOAT  proto={}".format(LORA_PROTO))
    print("=" * 50)

    spi = busio.SPI(LORA_SPI_SCK, MOSI=LORA_SPI_MOSI, MISO=LORA_SPI_MISO)
    cs = digitalio.DigitalInOut(LORA_CS)
    reset = digitalio.DigitalInOut(LORA_RST)

    try:
        import adafruit_rfm9x
    except ImportError:
        print("ERROR: adafruit_rfm9x missing from /lib")
        while True:
            time.sleep(1)

    try:
        try:
            rfm = adafruit_rfm9x.RFM9x(
                spi, cs, reset, LORA_FREQ_MHZ, baudrate=LORA_SPI_BAUD
            )
        except TypeError:
            rfm = adafruit_rfm9x.RFM9x(spi, cs, reset, LORA_FREQ_MHZ)
    except Exception as e:
        print("ERROR: RFM9x init:", e)
        while True:
            time.sleep(1)

    apply_radio_settings(rfm)
    print(
        "Radio OK  {:.1f} MHz  SF{}  BW{}  CR4/{}".format(
            LORA_FREQ_MHZ, SPREADING_FACTOR, SIGNAL_BANDWIDTH, CODING_RATE
        )
    )
    print("=" * 50 + "\n")

    last_beacon = time.monotonic()
    seq = 0

    while True:
        now = time.monotonic()
        rfm.receive_timeout = RX_TIMEOUT_S
        try:
            handle_rx(rfm, rfm.receive())
        except Exception as e:
            print("RX err:", e)

        if (now - last_beacon) >= BEACON_INTERVAL_S:
            last_beacon = now
            seq += 1
            # Same shape as main code.py / GUI backend (lat required)
            send_json(
                rfm,
                {
                    "proto": LORA_PROTO,
                    "role": "boat",
                    "seq": seq,
                    "lat": 32.7872,
                    "lon": -117.235,
                    "hdg": 0.0,
                    "spd": 0.0,
                    "bat": 100,
                    "sats": 0,
                    "obs": [],
                },
            )
            print("LoRa TX: beacon seq={}".format(seq))

        time.sleep(0.02)


try:
    main()
except KeyboardInterrupt:
    print("\nStopped.")
except Exception as e:
    print("FATAL:", e)
    raise
