# SPDX-FileCopyrightText: 2025 SeniorDesignGUI - SparkFun 9DoF IMU (ICM-20948) on Pico
# SPDX-License-Identifier: MIT
"""
CircuitPython driver for SparkFun 9DoF IMU Breakout - ICM-20948 on Raspberry Pi Pico.

Uses Adafruit's adafruit_icm20x library. Copy to your Pico and ensure the lib folder has:
  - adafruit_register
  - adafruit_bus_device
  - adafruit_icm20x.mpy

Wiring (Qwiic / I2C):
  Pico GP21 (SCL) -> IMU SCL
  Pico GP20 (SDA) -> IMU SDA
  Pico GND         -> IMU GND
  Pico 3V3         -> IMU VCC
"""

import time
import busio
import board

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
# I2C pins for Raspberry Pi Pico (no default I2C on Pico - must specify)
I2C_SDA = board.GP20
I2C_SCL = board.GP21
# SparkFun ICM-20948 default I2C address (use 0x68 if ADR jumper closed)
ICM20948_ADDRESS = 0x69


# -----------------------------------------------------------------------------
# ICM20948 driver class (use when importing this file as a module)
# -----------------------------------------------------------------------------
class ICM20948_Pico:
    """
    SparkFun 9DoF IMU (ICM-20948) driver for Raspberry Pi Pico with CircuitPython.
    Usage:
        from icm20948_pico import ICM20948_Pico
        imu = ICM20948_Pico()
        imu.read()
        print(imu.acceleration, imu.gyro, imu.magnetic)
    """

    def __init__(self, sda_pin=I2C_SDA, scl_pin=I2C_SCL, address=ICM20948_ADDRESS):
        self._i2c = busio.I2C(scl_pin, sda_pin)
        self._address = address
        self._icm = None
        self.acceleration = (0.0, 0.0, 0.0)  # m/s^2
        self.gyro = (0.0, 0.0, 0.0)         # rad/s
        self.magnetic = (0.0, 0.0, 0.0)      # uT
        self._init()

    def _init(self):
        try:
            from adafruit_icm20x import ICM20948
            self._icm = ICM20948(self._i2c, address=self._address)
        except ImportError:
            raise RuntimeError(
                "Copy adafruit_icm20x.mpy, adafruit_register, adafruit_bus_device to Pico lib"
            )
        except Exception as e:
            raise RuntimeError("ICM-20948 init failed: " + str(e))

    def read(self):
        """Read accelerometer, gyroscope, and magnetometer. Updates .acceleration, .gyro, .magnetic."""
        if self._icm is None:
            return
        try:
            self.acceleration = self._icm.acceleration
            self.gyro = self._icm.gyro
            self.magnetic = self._icm.magnetic
        except OSError:
            pass

    @property
    def ok(self):
        """True if sensor is initialized."""
        return self._icm is not None


def main():
    def log(msg):
        print(msg)

    log("ICM20948 script starting...")

    # Pico has no board.I2C(); create I2C explicitly (can block if SDA/SCL stuck)
    log("I2C init (GP20=SDA, GP21=SCL)...")
    i2c = busio.I2C(I2C_SCL, I2C_SDA)
    log("I2C ok.")

    try:
        from adafruit_icm20x import ICM20948
        log("Opening ICM-20948 at 0x%02X..." % ICM20948_ADDRESS)
        icm = ICM20948(i2c, address=ICM20948_ADDRESS)
        log("IMU ok.")
    except ImportError as e:
        log("Import error: " + str(e))
        log("Need in Pico lib/: adafruit_icm20x.mpy, adafruit_register/, adafruit_bus_device/")
        log("Get bundle: circuitpython.org/libraries  (match your CircuitPython version)")
        return
    except Exception as e:
        log("ICM-20948 init failed: " + str(e))
        return

    log("SparkFun 9DoF IMU (ICM-20948) on Pico - CircuitPython")
    log("Accel (m/s^2)  |  Gyro (rad/s)  |  Mag (uT)")
    log("-" * 60)

    while True:
        try:
            accel = icm.acceleration  # (x, y, z) m/s^2
            gyro = icm.gyro           # (x, y, z) rad/s
            mag = icm.magnetic        # (x, y, z) uT

            print(
                "A: {:7.2f},{:7.2f},{:7.2f}  G: {:6.3f},{:6.3f},{:6.3f}  M: {:7.2f},{:7.2f},{:7.2f}".format(
                    accel[0], accel[1], accel[2],
                    gyro[0], gyro[1], gyro[2],
                    mag[0], mag[1], mag[2]
                )
            )
        except OSError as e:
            log("Read error: " + str(e))
        time.sleep(0.2)


if __name__ == "__main__":
    main()
