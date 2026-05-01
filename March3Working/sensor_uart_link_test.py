"""
Sensor Pico UART link test (no GPS/radar/IMU dependencies).

Flash this file as code.py on the SENSOR Pico to verify the inter-Pico UART link.
Expected wiring for this test:
  Sensor GP16 (TX) -> LoRa Pico RX pin
  Sensor GP17 (RX) <- LoRa Pico TX pin
  GND <-> GND
"""

import board
import busio
import json
import time

TX_PIN = board.GP16
RX_PIN = board.GP17
BAUD = 115200


def main():
    uart = busio.UART(TX_PIN, RX_PIN, baudrate=BAUD, timeout=0.02)
    count = 0
    print("Sensor UART link test started on GP16/GP17 @ 115200")
    while True:
        payload = {
            "lat": 32.7872,
            "lon": -117.2350,
            "hdg": float(count % 360),
            "spd": 0.0,
            "bat": 100,
            "sats": 0,
            "obs": [],
            "seq": count,
        }
        line = json.dumps(payload, separators=(",", ":")) + "\n"
        uart.write(line.encode("utf-8"))
        print("TX seq", count)
        count += 1
        time.sleep(1.0)


main()
