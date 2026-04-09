"""
=============================================================================
LORA RFM95W — SIMPLE CONNECTION TEST (Pico)
=============================================================================
Save as code.py on the Pico to verify wiring before full ANCHOR LoRa code.

Same LoRa wiring as boat code.py — see lora_hw_config.py (copy to CIRCUITPY root).

Needs on CIRCUITPY: lora_hw_config.py (optional; defaults match if missing)
Needs in /lib: adafruit_rfm9x.mpy, adafruit_bus_device/

Open Thonny serial — you should see step-by-step OK/FAIL, then "LISTENING".
=============================================================================
"""

import board
import busio
import digitalio
import time

try:
    from lora_hw_config import (
        LORA_SPI_SCK,
        LORA_SPI_MOSI,
        LORA_SPI_MISO,
        LORA_CS,
        LORA_RST,
        LORA_DIO0,
        LORA_FREQ_MHZ,
        LORA_SPI_BAUD,
    )
except ImportError:
    LORA_SPI_SCK = board.GP18
    LORA_SPI_MOSI = board.GP19
    LORA_SPI_MISO = board.GP16
    LORA_CS = board.GP17
    LORA_RST = board.GP22
    LORA_DIO0 = board.GP26
    LORA_FREQ_MHZ = 915.0
    LORA_SPI_BAUD = 500_000
# SX1276 version register: expect 0x12 (18) for RFM95W / common HopeRF modules.
_REG_VERSION = 0x42
_EXPECT_VERSION = 0x12

# Set True only for wiring isolation: disconnect RFM from GP16 and GP19, then put
# ONE jumper wire between GP16 (MISO) and GP19 (MOSI). Proves Pico SPI pins work.
SPI_LOOPBACK_TEST = False


def line(msg):
    print(msg)


def spi_read_register(spi, cs, reg_addr, raw_miso=None):
    """
    Raw read of one SX1276 register (SPI mode 0).
    One CS-low transaction: addr byte + clock dummy → data on MISO (2nd byte).
    If raw_miso is a len-2 list, filled with both MISO-sampled bytes (for debug).
    Returns the register byte, or None if SPI fails.
    """
    while not spi.try_lock():
        pass
    try:
        spi.configure(baudrate=LORA_SPI_BAUD, polarity=0, phase=0)
        out = bytearray([reg_addr & 0x7F, 0x00])
        inp = bytearray(2)
        cs.value = False
        spi.write_readinto(out, inp)
        if raw_miso is not None and len(raw_miso) >= 2:
            raw_miso[0] = inp[0]
            raw_miso[1] = inp[1]
        return inp[1]
    except Exception:
        if raw_miso is not None and len(raw_miso) >= 2:
            raw_miso[0] = 0
            raw_miso[1] = 0
        return None
    finally:
        cs.value = True
        spi.unlock()


def main():
    line("")
    line("=" * 52)
    line("  ANCHOR — LoRa RFM95W  SIMPLE PIN / LINK TEST")
    line("=" * 52)

    # ----- 1) SPI -----
    line("")
    line("[1] SPI bus (same as boat: SCK=18, MOSI=19, MISO=16)")
    try:
        spi = busio.SPI(LORA_SPI_SCK, MOSI=LORA_SPI_MOSI, MISO=LORA_SPI_MISO)
        line("    STATUS: OK  (SPI object created)")
    except Exception as e:
        line("    STATUS: FAIL — " + str(e))
        line("    Check: every SPI wire + 3.3V/GND on module.")
        while True:
            time.sleep(1)

    # ----- 2) CS -----
    line("")
    line("[2] Chip select (CS = GP17, LORA_CS)")
    try:
        cs = digitalio.DigitalInOut(LORA_CS)
        cs.direction = digitalio.Direction.OUTPUT
        cs.value = True
        line("    STATUS: OK  (CS high)")
    except Exception as e:
        line("    STATUS: FAIL — " + str(e))
        while True:
            time.sleep(1)

    if SPI_LOOPBACK_TEST:
        line("")
        line("[2b] SPI loopback (diagnostic)")
        line("    Unplug RFM wires from GP16 and GP19. Bridge GP19 → GP16 with one jumper.")
        while not spi.try_lock():
            pass
        try:
            spi.configure(baudrate=LORA_SPI_BAUD, polarity=0, phase=0)
            out = bytearray([0x5A, 0xA5, 0x3C])
            inp = bytearray(3)
            spi.write_readinto(out, inp)
            line(
                "    Echo check: sent 0x5A 0xA5 0x3C | read 0x{:02X} 0x{:02X} 0x{:02X}".format(
                    inp[0],
                    inp[1],
                    inp[2],
                )
            )
            if inp == out:
                line("    STATUS: OK — Pico SPI + GP16/GP19 are fine. Problem is RFM side.")
            else:
                line("    STATUS: FAIL — no echo: jumper, pins, or Pico SPI issue.")
        except Exception as e:
            line("    STATUS: FAIL — " + str(e))
        finally:
            spi.unlock()
        line("    Remove jumper, reconnect RFM, set SPI_LOOPBACK_TEST = False, re-run.")
        while True:
            time.sleep(1)

    # ----- 3) Reset pulse -----
    line("")
    line("[3] Reset line (RST = GP22)")
    try:
        rst = digitalio.DigitalInOut(LORA_RST)
        rst.direction = digitalio.Direction.OUTPUT
        rst.value = True
        time.sleep(0.05)
        rst.value = False
        time.sleep(0.1)
        rst.value = True
        time.sleep(0.1)
        line("    STATUS: OK  (reset pulse sent)")
    except Exception as e:
        line("    STATUS: FAIL — " + str(e))
        while True:
            time.sleep(1)

    # ----- 3b) Raw version byte (before RFM9x driver) -----
    line("")
    line("[3b] SPI read reg 0x42 (chip ID) @ {} Hz".format(LORA_SPI_BAUD))
    cs.value = True
    time.sleep(0.01)
    _raw = [0, 0]
    ver = spi_read_register(spi, cs, _REG_VERSION, raw_miso=_raw)
    if ver is None:
        line("    STATUS: FAIL — SPI transaction error")
        line("    Check: SCK/MOSI/MISO not swapped, CS on NSS, common GND.")
        while True:
            time.sleep(1)
    line(
        "    Reg value (2nd SPI byte): 0x{:02X} — expect 0x{:02X} for RFM95/SX1276".format(
            ver,
            _EXPECT_VERSION,
        )
    )
    line(
        "    MISO sampled: 1st byte 0x{:02X}  2nd byte 0x{:02X}  (both 0x00 → line not driven)".format(
            _raw[0],
            _raw[1],
        )
    )
    if ver == 0x00 or ver == 0xFF:
        line("    HINT: 0x00 or 0xFF = Pico is not getting real bits on MISO (or CS never selects chip).")
        v1 = spi_read_register(spi, cs, 0x01)
        v0 = spi_read_register(spi, cs, 0x00)
        line(
            "    Cross-check: reg 0x01=0x{:02X}  reg 0x00=0x{:02X}  (if all 0x00/0xFF → MISO or CS)".format(
                v1 if v1 is not None else 0xFF,
                v0 if v0 is not None else 0xFF,
            )
        )
        line("    1) Meter: 3.3 V module VCC to GND while Pico runs (not 5 V).")
        line("    2) GND: module GND ↔ Pico GND (same rail).")
        line("    3) CS: RFM **NSS** → GP17 only; no other device sharing that CS line.")
        line("    4) MISO: RFM **MISO** (data from radio) → Pico **GP16** (not MOSI).")
        line("    5) MOSI: RFM **MOSI** → GP19. SCK: RFM **SCK** → GP18. RST → GP22.")
        line("    6) Try **swapping MOSI and MISO** once — mislabeled breakouts are common.")
        line("    STOP: Until 0x42 reads 0x12, LoRa code will not work.")
        while True:
            time.sleep(1)
    elif ver != _EXPECT_VERSION:
        line("    HINT: Not SX1276-class? Some boards use SX1262 / LoRaWAN modules (incompatible with adafruit_rfm9x).")
        line("    Or weak SPI: shorten wires; in lora_hw_config.py set LORA_SPI_BAUD = 100_000.")
        line("    STOP: RFM9x driver will not start until reg 0x42 reads 0x12.")
        while True:
            time.sleep(1)

    # ----- 4) G0 / DIO0 (optional input) -----
    line("")
    line("[4] G0 / DIO0 (GP26) — read pin level")
    try:
        g0 = digitalio.DigitalInOut(LORA_DIO0)
        g0.direction = digitalio.Direction.INPUT
        line("    STATUS: OK  (G0 reads as {})".format(g0.value))
    except Exception as e:
        line("    STATUS: WARN — " + str(e))
        line("    (LoRa may still work if G0 not used for IRQ in software)")

    # ----- 5) RFM9x chip over SPI -----
    line("")
    line("[5] RFM9x driver (real chip talk on SPI)")
    line(
        "    NOTE: adafruit_rfm9x defaults to 5 MHz SPI; dupont wires often need "
        "{} Hz (LORA_SPI_BAUD in lora_hw_config.py).".format(LORA_SPI_BAUD)
    )
    try:
        import adafruit_rfm9x
    except ImportError:
        line("    STATUS: FAIL — adafruit_rfm9x not in /lib")
        line("    Copy adafruit_rfm9x.mpy from CircuitPython bundle.")
        while True:
            time.sleep(1)

    try:
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
            # Very old bundle without baudrate= — will use 5 MHz default
            rfm = adafruit_rfm9x.RFM9x(spi, cs, rst, LORA_FREQ_MHZ)
        rfm.signal_bandwidth = 125000
        rfm.spreading_factor = 7
        rfm.coding_rate = 5
        rfm.enable_crc = True
        rfm.tx_power = 13
        line("    STATUS: OK")
        line("    Radio: {:.1f} MHz  SF7  BW125k  CRC on".format(LORA_FREQ_MHZ))
        try:
            line("    Chip reports frequency: {:.2f} MHz".format(rfm.frequency_mhz))
        except Exception:
            pass
    except Exception as e:
        line("    STATUS: FAIL — " + str(e))
        line("    Check: CS, RST, MOSI/MISO/SCK, 3.3V, GND.")
        line(
            "    If [3b] was 0x12: lower LORA_SPI_BAUD in lora_hw_config.py or update bundle; "
            "driver used to default 5 MHz SPI."
        )
        while True:
            time.sleep(1)

    line("")
    line("=" * 52)
    line("  ALL CHECKS PASSED — hardware path looks good.")
    line("  Listening for any LoRa packet (partner radio optional).")
    line("=" * 52)
    line("")

    # ----- 6) Listen loop -----
    rfm.receive_timeout = 1.0
    tick = 0
    while True:
        tick += 1
        if tick % 3 == 1:
            line("[LISTENING] waiting for LoRa RX... (tick {})".format(tick))

        try:
            packet = rfm.receive()
        except Exception as e:
            line("RX error: " + str(e))
            time.sleep(1)
            continue

        if packet is not None:
            line("")
            line("*** PACKET RX ***  {} bytes".format(len(packet)))
            try:
                line("    data: " + str(packet[:64]))
            except Exception:
                pass
            try:
                line("    RSSI: {} dBm".format(rfm.last_rssi))
            except Exception:
                pass
            line("")
        else:
            time.sleep(0.2)


try:
    main()
except KeyboardInterrupt:
    line("\nStopped.")
except Exception as e:
    line("FATAL: " + str(e))
    raise
