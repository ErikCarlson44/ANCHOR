"""
gps.py - NEO-M8N GPS driver for CircuitPython.
Tested on: Raspberry Pi Pico (CircuitPython 8+)
Pins: TX -> GP0, RX -> GP1  (UART0)
Parses: $GNGGA (fix, sats, lat, lon, alt)
        $GNRMC (status, lat, lon, speed)
"""

import board
import busio
import time


class GPS:
    def __init__(self, tx_pin=board.GP0, rx_pin=board.GP1, baudrate=9600):
        self.uart = busio.UART(
            tx=tx_pin,
            rx=rx_pin,
            baudrate=baudrate,
            timeout=0.1,
        )
        self.buffer = b""
        self.fix_ok = False
        self.lat = None
        self.lon = None
        self.alt = None
        self.speed = None
        self.sats = None

    @staticmethod
    def _strip_checksum(sentence):
        if "*" in sentence:
            return sentence.split("*", 1)[0]
        return sentence

    @staticmethod
    def _nmea_deg_to_decimal(raw, hemi):
        if not raw or not hemi:
            return None
        try:
            value = float(raw)
        except ValueError:
            return None
        degrees = int(value // 100)
        minutes = value - (degrees * 100)
        dec = degrees + (minutes / 60.0)
        if hemi in ("S", "W"):
            dec = -dec
        return dec

    def _parse_gga(self, sentence):
        sentence = self._strip_checksum(sentence)
        parts = sentence.split(",")
        if len(parts) < 10:
            return None
        try:
            fix_quality = int(parts[6]) if parts[6] else 0
            sats = int(parts[7]) if parts[7] else None
            lat = self._nmea_deg_to_decimal(parts[2], parts[3])
            lon = self._nmea_deg_to_decimal(parts[4], parts[5])
            alt = float(parts[9]) if parts[9] else None
            return fix_quality, sats, lat, lon, alt
        except (ValueError, IndexError):
            return None

    def _parse_rmc(self, sentence):
        sentence = self._strip_checksum(sentence)
        parts = sentence.split(",")
        if len(parts) < 10:
            return None
        try:
            status = parts[2]
            lat = self._nmea_deg_to_decimal(parts[3], parts[4])
            lon = self._nmea_deg_to_decimal(parts[5], parts[6])
            speed_knots = float(parts[7]) if parts[7] else None
            speed_mps = speed_knots * 0.514444 if speed_knots is not None else None
            return status, lat, lon, speed_mps
        except (ValueError, IndexError):
            return None

    def update(self):
        updated = False
        in_waiting = self.uart.in_waiting
        if in_waiting:
            chunk = self.uart.read(in_waiting)
            if chunk:
                self.buffer += chunk

        if len(self.buffer) > 512:
            self.buffer = self.buffer[-256:]

        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            try:
                sentence = line.decode("ascii").strip()
            except Exception:
                continue

            if not sentence:
                continue

            if sentence.startswith("$GNGGA") or sentence.startswith("$GPGGA"):
                result = self._parse_gga(sentence)
                if result:
                    fix_quality, sats, lat, lon, alt = result
                    self.fix_ok = fix_quality > 0
                    self.sats = sats
                    if lat is not None:
                        self.lat = lat
                    if lon is not None:
                        self.lon = lon
                    if alt is not None:
                        self.alt = alt
                    updated = True

            elif sentence.startswith("$GNRMC") or sentence.startswith("$GPRMC"):
                result = self._parse_rmc(sentence)
                if result:
                    status, lat, lon, speed = result
                    if lat is not None:
                        self.lat = lat
                    if lon is not None:
                        self.lon = lon
                    if speed is not None:
                        self.speed = speed
                    self.fix_ok = self.fix_ok or (status == "A")
                    updated = True
        return updated

    def wait_for_fix(self, timeout_s=600, print_status=True):
        """
        Block until a GPS fix is acquired or timeout is reached.

        Returns True if fix acquired, False if timed out.
        """
        start = time.monotonic()
        last_print = start

        if print_status:
            print("GPS: waiting for fix", end="")

        while True:
            self.update()

            if self.fix_ok and self.lat is not None and self.lon is not None:
                if print_status:
                    print(" OK")
                    print(
                        "GPS: lat={:.6f} lon={:.6f} sats={}".format(
                            self.lat, self.lon, self.sats
                        )
                    )
                return True

            now = time.monotonic()
            if print_status and (now - last_print) >= 1.0:
                print(".", end="")
                last_print = now

            if (now - start) >= timeout_s:
                if print_status:
                    print(" TIMEOUT ({}s) - continuing without fix".format(timeout_s))
                return False

            time.sleep(0.1)

    def to_csv_fields(self):
        if self.fix_ok and self.lat is not None and self.lon is not None:
            return "{},{:.6f},{:.6f},{},{},{}".format(
                1,
                self.lat,
                self.lon,
                "{:.1f}".format(self.alt) if self.alt is not None else "NA",
                "{:.2f}".format(self.speed) if self.speed is not None else "NA",
                self.sats if self.sats is not None else "NA",
            )
        return "0,NA,NA,NA,NA,NA"

    def __str__(self):
        if self.fix_ok and self.lat is not None and self.lon is not None:
            return "GPS(fix=YES lat={:.6f} lon={:.6f} alt={}m spd={}m/s sats={})".format(
                self.lat, self.lon, self.alt, self.speed, self.sats
            )
        return "GPS(fix=NO)"

    def close(self):
        """Release the UART peripheral."""
        self.uart.deinit()


def _run_standalone_test():
    print("GPS standalone test starting...")
    gps = GPS()
    print("Waiting for GPS data on GP0/GP1 at 9600 baud.")
    print("Press Ctrl+C to stop.")

    last_print = time.monotonic()
    try:
        while True:
            gps.update()
            now = time.monotonic()
            if (now - last_print) >= 1.0:
                last_print = now
                print(
                    "fix={} lat={} lon={} alt={} speed={} sats={}".format(
                        1 if gps.fix_ok else 0,
                        "{:.6f}".format(gps.lat) if gps.lat is not None else "NA",
                        "{:.6f}".format(gps.lon) if gps.lon is not None else "NA",
                        "{:.1f}".format(gps.alt) if gps.alt is not None else "NA",
                        "{:.2f}".format(gps.speed) if gps.speed is not None else "NA",
                        gps.sats if gps.sats is not None else "NA",
                    )
                )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Stopping GPS test.")
    finally:
        gps.close()


if __name__ == "__main__":
    _run_standalone_test()
