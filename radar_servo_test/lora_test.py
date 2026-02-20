"""
LoRa Module Connection Test
Adafruit RFM95W (PID 3072) - 868/915 MHz

Tests if the LoRa module is properly connected via SPI.

Wiring (Pico to RFM95W):
  Pico GP10 (SPI1 SCK)  -> RFM95W SCK
  Pico GP11 (SPI1 MOSI) -> RFM95W MOSI
  Pico GP12 (SPI1 MISO) -> RFM95W MISO
  Pico GP13             -> RFM95W CS
  Pico GP14             -> RFM95W RST (Reset)
  Pico 3.3V             -> RFM95W VIN
  Pico GND              -> RFM95W GND

Optional (for interrupt):
  Pico GP15             -> RFM95W G0 (DIO0)

Copy this to your Pico as code.py
Requires: adafruit_rfm9x library in /lib folder
"""

import board
import busio
import digitalio
import time

print("=" * 50)
print("ADAFRUIT RFM95W LORA TEST")
print("=" * 50)
print("")

# =============================================================================
# CONFIGURATION - Adjust pins if needed
# =============================================================================
SPI_SCK = board.GP18
SPI_MOSI = board.GP19
SPI_MISO = board.GP16
CS_PIN = board.GP17
RESET_PIN = board.GP22

# Radio frequency (match your module: 868 or 915 MHz)
RADIO_FREQ_MHZ = 915.0

# =============================================================================
# TEST 1: Check SPI Connection
# =============================================================================
print("TEST 1: SPI Connection")
print("-" * 30)

try:
    spi = busio.SPI(SPI_SCK, MOSI=SPI_MOSI, MISO=SPI_MISO)
    print("  [OK] SPI initialized")
except Exception as e:
    print("  [FAIL] SPI init error:", e)
    print("\nCheck wiring:")
    print("  GP10 -> SCK")
    print("  GP11 -> MOSI")
    print("  GP12 -> MISO")
    while True:
        pass

# =============================================================================
# TEST 2: Check CS and Reset pins
# =============================================================================
print("")
print("TEST 2: Control Pins")
print("-" * 30)

try:
    cs = digitalio.DigitalInOut(CS_PIN)
    cs.direction = digitalio.Direction.OUTPUT
    cs.value = True
    print("  [OK] CS pin (GP13) configured")
except Exception as e:
    print("  [FAIL] CS pin error:", e)

try:
    reset = digitalio.DigitalInOut(RESET_PIN)
    reset.direction = digitalio.Direction.OUTPUT
    print("  [OK] Reset pin (GP14) configured")
except Exception as e:
    print("  [FAIL] Reset pin error:", e)

# =============================================================================
# TEST 3: Reset the module
# =============================================================================
print("")
print("TEST 3: Module Reset")
print("-" * 30)

try:
    reset.value = False
    time.sleep(0.1)
    reset.value = True
    time.sleep(0.1)
    print("  [OK] Reset pulse sent")
except Exception as e:
    print("  [FAIL] Reset error:", e)

# =============================================================================
# TEST 4: Initialize RFM9x Library
# =============================================================================
print("")
print("TEST 4: RFM9x Library Init")
print("-" * 30)

rfm9x = None
try:
    import adafruit_rfm9x
    
    rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, RADIO_FREQ_MHZ)
    print("  [OK] RFM9x initialized!")
    print("  Frequency: {} MHz".format(RADIO_FREQ_MHZ))
    
except ImportError:
    print("  [FAIL] adafruit_rfm9x library not found!")
    print("")
    print("  To install:")
    print("  1. Download from circuitpython.org/libraries")
    print("  2. Copy adafruit_rfm9x.mpy to Pico's /lib folder")
    print("  3. Also copy adafruit_bus_device folder to /lib")
    rfm9x = None
    
except Exception as e:
    print("  [FAIL] RFM9x init error:", e)
    print("")
    print("  Common issues:")
    print("  - Wrong wiring")
    print("  - Module not powered (check 3.3V)")
    print("  - Wrong frequency for your module")
    rfm9x = None

# =============================================================================
# TEST 5: Read Module Info
# =============================================================================
if rfm9x:
    print("")
    print("TEST 5: Module Info")
    print("-" * 30)
    
    try:
        # Try to read version register
        print("  TX Power: {} dBm".format(rfm9x.tx_power))
        print("  Frequency: {} MHz".format(rfm9x.frequency_mhz))
        print("  [OK] Module responding!")
    except Exception as e:
        print("  [FAIL] Could not read module info:", e)

# =============================================================================
# TEST 6: Send Test Packet
# =============================================================================
if rfm9x:
    print("")
    print("TEST 6: Transmit Test")
    print("-" * 30)
    
    try:
        # Set transmit power (5-23 dBm)
        rfm9x.tx_power = 13
        
        test_msg = b"LORA_TEST_OK"
        print("  Sending: {}".format(test_msg))
        
        rfm9x.send(test_msg)
        print("  [OK] Packet sent successfully!")
        
    except Exception as e:
        print("  [FAIL] Transmit error:", e)

# =============================================================================
# TEST 7: Listen for Packets (5 seconds)
# =============================================================================
if rfm9x:
    print("")
    print("TEST 7: Receive Test (5 seconds)")
    print("-" * 30)
    print("  Listening for incoming packets...")
    
    rfm9x.receive_timeout = 5.0
    
    try:
        packet = rfm9x.receive()
        
        if packet is not None:
            print("  [OK] Received: {}".format(packet))
            try:
                print("  RSSI: {} dBm".format(rfm9x.last_rssi))
            except:
                pass
        else:
            print("  [--] No packets received (normal if no transmitter)")
            
    except Exception as e:
        print("  [FAIL] Receive error:", e)

# =============================================================================
# RESULTS
# =============================================================================
print("")
print("=" * 50)
if rfm9x:
    print("RESULT: LORA MODULE CONNECTED!")
    print("=" * 50)
    print("")
    print("Module is ready for use.")
    print("Frequency: {} MHz".format(RADIO_FREQ_MHZ))
else:
    print("RESULT: CONNECTION FAILED")
    print("=" * 50)
    print("")
    print("Check:")
    print("1. Wiring connections")
    print("2. 3.3V power to module")
    print("3. adafruit_rfm9x library installed")

print("")
print("Wiring Reference:")
print("  Pico GP10 -> RFM95W SCK")
print("  Pico GP11 -> RFM95W MOSI")
print("  Pico GP12 -> RFM95W MISO")
print("  Pico GP13 -> RFM95W CS")
print("  Pico GP14 -> RFM95W RST")
print("  Pico 3.3V -> RFM95W VIN")
print("  Pico GND  -> RFM95W GND")
