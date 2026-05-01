"""
RFM9x / RFM95W smoke test for Raspberry Pi Pico (CircuitPython).

Wiring matches March3Working/code.py:
  SCK  GP18   MOSI GP19   MISO GP16   CS GP17   RST GP22
  915 MHz, SF7, BW 125 kHz, CR 4/5, CRC on, TX power 13 dBm (same as LoraRFM95).

Copy to CIRCUITPY as code.py (or run from Thonny). Needs /lib:
  adafruit_rfm9x.mpy, adafruit_bus_device/

Serial console: PASS/FAIL for chip ID + driver, then optional beacon + RX.
"""

import board
import busio
import digitalio
import time

# --- Same LoRa pins / frequency as code.py (boat firmware) ---
LORA_SPI_SCK = board.GP18
LORA_SPI_MOSI = board.GP19
LORA_SPI_MISO = board.GP16
LORA_CS = board.GP17
LORA_RST = board.GP22
LORA_FREQ_MHZ = 915.0
LORA_RX_TIMEOUT_S = 0.02

# Dupont wiring: lower SPI speed often reads 0x42 reliably; code.py uses driver default.
LORA_SPI_BAUD = 500_000

# SX1276 version register address; expect 0x12 for RFM95W-class modules.
_REG_VERSION_ADDR = 0x42
_EXPECT_VERSION = 0x12

# Set True to send a small packet every BEACON_INTERVAL_S (second Pico or SDR can verify).
SEND_TEST_BEACONS = True
BEACON_INTERVAL_S = 4.0
BEACON_PAYLOAD = b"RFM9X_PICO_TEST"


def _print(msg):
    print(msg)


def _spi_read_reg(spi, cs, reg_addr):
    while not spi.try_lock():
        pass
    try:
        spi.configure(baudrate=LORA_SPI_BAUD, polarity=0, phase=0)
        out = bytearray([reg_addr & 0x7F, 0x00])
        inp = bytearray(2)
        cs.value = False
        spi.write_readinto(out, inp)
        return inp[1]
    except Exception:
        return None
    finally:
        cs.value = True
        spi.unlock()


def _reset_radio(rst):
    rst.direction = digitalio.Direction.OUTPUT
    rst.value = True
    time.sleep(0.05)
    rst.value = False
    time.sleep(0.1)
    rst.value = True
    time.sleep(0.05)


def main():
    _print("")
    _print("=" * 56)
    _print("  RFM9x Pico test (wiring = code.py LoRa block)")
    _print("=" * 56)

    _print("[1] SPI + CS + RST")
    spi = busio.SPI(LORA_SPI_SCK, MOSI=LORA_SPI_MOSI, MISO=LORA_SPI_MISO)
    cs = digitalio.DigitalInOut(LORA_CS)
    cs.direction = digitalio.Direction.OUTPUT
    cs.value = True
    rst = digitalio.DigitalInOut(LORA_RST)
    _reset_radio(rst)
    _print("    OK")

    _print("[2] Raw SPI: read reg 0x{:02X} (chip version)".format(_REG_VERSION_ADDR))
    ver = _spi_read_reg(spi, cs, _REG_VERSION_ADDR)
    if ver is None:
        _print("    FAIL — SPI transaction error")
        while True:
            time.sleep(2)
    _print("    value 0x{:02X}  (expect 0x{:02X})".format(ver, _EXPECT_VERSION))
    if ver != _EXPECT_VERSION:
        _print("    FAIL — wrong ID or bad wiring (MISO/CS/GND/3V3). See lora_pico_simple_test.py hints.")
        while True:
            time.sleep(2)
    _print("    OK")

    _print("[3] adafruit_rfm9x RFM9x driver")
    try:
        import adafruit_rfm9x
    except ImportError:
        _print("    FAIL — add adafruit_rfm9x to /lib")
        while True:
            time.sleep(2)

    while not spi.try_lock():
        pass
    try:
        spi.configure(baudrate=LORA_SPI_BAUD, polarity=0, phase=0)
    finally:
        spi.unlock()

    try:
        rfm = adafruit_rfm9x.RFM9x(
            spi, cs, rst, LORA_FREQ_MHZ, baudrate=LORA_SPI_BAUD
        )
    except TypeError:
        rfm = adafruit_rfm9x.RFM9x(spi, cs, rst, LORA_FREQ_MHZ)

    rfm.signal_bandwidth = 125000
    rfm.spreading_factor = 7
    rfm.coding_rate = 5
    rfm.enable_crc = True
    rfm.tx_power = 13
    rfm.receive_timeout = 1.0 if SEND_TEST_BEACONS else LORA_RX_TIMEOUT_S

    _print("    OK  {:.1f} MHz  SF{}  BW125k  CRC on".format(LORA_FREQ_MHZ, rfm.spreading_factor))
    try:
        _print("    driver frequency_mhz: {:.2f}".format(rfm.frequency_mhz))
    except Exception:
        pass

    _print("")
    _print("PASS — RFM9x responds and driver initialized.")
    if SEND_TEST_BEACONS:
        _print("Beacon every {:.0f}s: {!r}".format(BEACON_INTERVAL_S, BEACON_PAYLOAD))
    _print("Listening for LoRa RX (Ctrl+C in Thonny to stop).")
    _print("=" * 56)

    next_tx = time.monotonic()
    while True:
        now = time.monotonic()
        if SEND_TEST_BEACONS and now >= next_tx:
            next_tx = now + BEACON_INTERVAL_S
            try:
                rfm.send(BEACON_PAYLOAD)
                _print("[TX] {} bytes OK".format(len(BEACON_PAYLOAD)))
            except Exception as e:
                _print("[TX] FAIL: " + str(e))

        try:
            pkt = rfm.receive()
        except Exception as e:
            _print("[RX] error: " + str(e))
            time.sleep(0.5)
            continue

        if pkt is not None:
            try:
                rssi = rfm.last_rssi
            except Exception:
                rssi = "?"
            _print("[RX] {} bytes  RSSI={}  {!r}".format(len(pkt), rssi, pkt[:64]))
        else:
            time.sleep(0.05)


main()
