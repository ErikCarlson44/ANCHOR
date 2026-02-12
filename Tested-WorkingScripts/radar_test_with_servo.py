

import time
import board
import busio

print("RD-03D UART listener (no hardware).")

uart = busio.UART(board.GP0, board.GP1, baudrate=115200, timeout=0.1)

buffer = b""

while True:
    chunk = uart.read(64)
    if chunk:
        buffer += chunk

        # process on newline
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            try:
                s = line.decode("ascii", "ignore").strip()
                print("RAW:", s)
            except Exception:
                pass

    time.sleep(0.01)
