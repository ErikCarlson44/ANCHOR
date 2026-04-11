"""
=============================================================================
ANCHOR — LoRa RFM9x RECEIVER (adafruit_rfm9x / CircuitPython)
=============================================================================
Listens for LoRa packets matching the transmitter config:
  SF=11, BW=125 kHz, CR=4/5, CRC=True, 915.0 MHz, Tx+17 dBm

Wiring (matches config constants below):
  RFM9x SCK  -> GP18
  RFM9x MOSI -> GP19
  RFM9x MISO -> GP16
  RFM9x CS   -> GP17
  RFM9x RST  -> GP22
  RFM9x VCC  -> 3.3V
  RFM9x GND  -> GND

  NOTE: The Pico's GP23 / GP25 are NOT used here; if your breakout has a
        DIO0 line you can wire it to any GP and use it for IRQ-based RX,
        but adafruit_rfm9x polls internally so it is not required.

Output: USB serial — human-readable status line every PRINT_INTERVAL_S,
        plus a JSON line for every decoded packet received.

Typical first-run checklist:
  1. STATUS shows "No LoRa module" → check SPI wiring / CS / RST.
  2. STATUS shows "Init OK, listening" but rx_count stays 0 →
       transmitter SF/BW/freq must EXACTLY match; check antenna on both ends.
  3. RSSI very negative (< -120 dBm) → move devices closer or improve antenna.
=============================================================================
"""

import board
import busio
import digitalio
import time
import json

# ─── LoRa pin / radio constants ──────────────────────────────────────────────
LORA_ENABLED      = True
LORA_SPI_SCK      = board.GP18
LORA_SPI_MOSI     = board.GP19
LORA_SPI_MISO     = board.GP16
LORA_CS           = board.GP17
LORA_RST          = board.GP22
LORA_FREQ_MHZ     = 915.0
LORA_SEND_EVERY_N = 5       # uplink every N USB telemetry frames (save duty cycle)
LORA_RX_TIMEOUT_S = 0.02    # short poll so main loop stays responsive
LORA_MAX_PACKET   = 110     # bytes (LoRa MTU ~255; keep JSON small)

# ─── Misc ─────────────────────────────────────────────────────────────────────
PRINT_INTERVAL_S  = 1.0     # how often to print status when no packet arrives


# ─── LoRa Radio Class ─────────────────────────────────────────────────────────

class LoRaRadio:
    """
    Thin wrapper around adafruit_rfm9x.RFM9x.
    Matches the transmitter's exact air-interface settings so packets decode.
    """

    def __init__(self):
        import adafruit_rfm9x

        self.spi = busio.SPI(LORA_SPI_SCK, MOSI=LORA_SPI_MOSI, MISO=LORA_SPI_MISO)
        self.cs  = digitalio.DigitalInOut(LORA_CS)
        self.cs.direction  = digitalio.Direction.OUTPUT
        self.rst = digitalio.DigitalInOut(LORA_RST)
        self.rst.direction = digitalio.Direction.OUTPUT

        # Core radio init — frequency must match transmitter exactly
        self.rfm = adafruit_rfm9x.RFM9x(self.spi, self.cs, self.rst, LORA_FREQ_MHZ)

        # ── Air-interface settings — ALL must match the transmitter ──────────
        self.rfm.signal_bandwidth = 125000   # 125 kHz
        self.rfm.spreading_factor = 11       # SF11
        self.rfm.coding_rate      = 5        # 4/5
        self.rfm.enable_crc       = True     # drop corrupt packets at radio level
        self.rfm.tx_power         = 17       # dBm  (matters for TX; set anyway)
        self.rfm.receive_timeout  = LORA_RX_TIMEOUT_S

        # ── State ─────────────────────────────────────────────────────────────
        self.last_rssi  = None
        self.rx_count   = 0
        self.err_count  = 0

    # ── Receive ───────────────────────────────────────────────────────────────

    def receive(self):
        """
        Poll for one packet.  Returns decoded dict (or raw bytes on JSON error),
        or None if nothing arrived within LORA_RX_TIMEOUT_S.

        Side-effects: updates self.last_rssi, self.rx_count, self.err_count.
        """
        try:
            raw = self.rfm.receive(timeout=LORA_RX_TIMEOUT_S, keep_listening=True)
        except Exception as e:
            self.err_count += 1
            print("  ! rfm.receive() error:", e)
            return None

        if raw is None:
            return None

        # adafruit_rfm9x appends the 2-byte RFM9x RSSI byte; last_rssi property
        # is updated by the library after a successful receive.
        self.last_rssi = self.rfm.last_rssi
        self.rx_count += 1

        # Trim to declared max (safety guard against garbage frames)
        payload = bytes(raw)[:LORA_MAX_PACKET]

        # Attempt UTF-8 → JSON decode
        try:
            text = payload.decode("utf-8").strip()
            data = json.loads(text)
            return data
        except (UnicodeDecodeError, ValueError):
            # Return raw hex string so caller can still log it
            return {"raw_hex": payload.hex(), "len": len(payload)}

    # ── Transmit ──────────────────────────────────────────────────────────────

    def send(self, data_dict):
        """
        Encode dict as compact JSON and transmit.
        Truncates silently if > LORA_MAX_PACKET bytes.
        Returns True on success, False on error.
        """
        try:
            payload = json.dumps(data_dict).encode("utf-8")[:LORA_MAX_PACKET]
            self.rfm.send(payload)
            return True
        except Exception as e:
            self.err_count += 1
            print("  ! rfm.send() error:", e)
            return False

    # ── Diagnostics ───────────────────────────────────────────────────────────

    def status_str(self):
        rssi = "{:.0f} dBm".format(self.last_rssi) if self.last_rssi is not None else "—"
        return (
            "rx={} err={} last_rssi={}  "
            "[SF{} BW125k CR4/5 CRC {} MHz]".format(
                self.rx_count,
                self.err_count,
                rssi,
                self.rfm.spreading_factor,
                self.rfm.signal_bandwidth // 1000,
                LORA_FREQ_MHZ,
            )
        )


# ─── Main loop ────────────────────────────────────────────────────────────────

def main():
    print()
    print("=" * 60)
    print("  ANCHOR LoRa RX  SF11 BW125k CR4/5 CRC  915.0 MHz  +17dBm")
    print("=" * 60)

    if not LORA_ENABLED:
        print("  LORA_ENABLED = False — exiting.")
        return

    # ── Init radio ────────────────────────────────────────────────────────────
    print("Initialising RFM9x …", end=" ")
    try:
        radio = LoRaRadio()
        print("OK")
    except Exception as e:
        print("FAILED:", e)
        print()
        print("  Checklist:")
        print("    • SCK→GP18  MOSI→GP19  MISO→GP16  CS→GP17  RST→GP22")
        print("    • Module powered from 3.3 V, GND connected")
        print("    • adafruit_rfm9x.mpy in /lib on the Pico")
        print()
        # Hard stop — nothing we can do without the radio
        while True:
            time.sleep(1)

    print("Listening … (SF11 / BW125k / CR4/5 / CRC / 915.0 MHz)")
    print()

    last_print   = time.monotonic()
    frame_number = 0
    uplink_count = 0

    while True:
        # ── 1. Try to receive a packet (non-blocking within RX_TIMEOUT_S) ────
        packet = radio.receive()

        if packet is not None:
            rssi_str = (
                "{:.0f} dBm".format(radio.last_rssi)
                if radio.last_rssi is not None
                else "?"
            )
            print(
                "[PKT #{:04d}] RSSI={:>9s}  payload={}".format(
                    radio.rx_count, rssi_str, packet
                )
            )

        # ── 2. Periodic status line ───────────────────────────────────────────
        now = time.monotonic()
        if (now - last_print) >= PRINT_INTERVAL_S:
            last_print = now
            frame_number += 1
            print("[STATUS {:05d}]  {}".format(frame_number, radio.status_str()))

            # ── 3. Optional uplink every LORA_SEND_EVERY_N frames ─────────────
            if frame_number % LORA_SEND_EVERY_N == 0:
                beacon = {
                    "t": "ACK",
                    "f": frame_number,
                    "rx": radio.rx_count,
                }
                ok = radio.send(beacon)
                uplink_count += 1
                print(
                    "  [TX #{:03d}] beacon sent: {}  ({})".format(
                        uplink_count,
                        beacon,
                        "OK" if ok else "FAIL",
                    )
                )

        # ── 4. Yield a small slice so CircuitPython housekeeping runs ─────────
        time.sleep(0.005)


# ─── Entry point ──────────────────────────────────────────────────────────────
try:
    main()
except KeyboardInterrupt:
    print("\nStopped.")
