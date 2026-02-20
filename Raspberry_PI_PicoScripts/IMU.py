"""
IMU Module - Reads orientation data from IMU sensor
Import this module and create an IMUReader instance.

Supports:
- ICM-20948 (SparkFun 9DoF Qwiic) - DEFAULT
- MPU6050 (accelerometer + gyroscope)
- MPU9250 (accelerometer + gyroscope + magnetometer)
- BNO055 (9-DOF with built-in sensor fusion)
- LSM9DS1 (accelerometer + gyroscope + magnetometer)
"""

import time
import math
import board
import busio

# -----------------------------
# Configuration - CHANGE THESE
# -----------------------------
I2C_SDA = board.GP14          # I2C Data
I2C_SCL = board.GP15          # I2C Clock

# IMU type: "ICM20948", "MPU6050", "MPU9250", "BNO055", "LSM9DS1"
IMU_TYPE = "ICM20948"         # SparkFun 9DoF Qwiic

# I2C address (0x68 or 0x69 for most IMUs)
IMU_ADDRESS = 0x69            # ICM-20948 default is 0x69

# -----------------------------
# IMU Reader Class
# -----------------------------
class IMUReader:
    """
    Reads orientation data from IMU sensor.
    
    Provides:
    - heading (yaw) from magnetometer or gyro integration
    - pitch and roll from accelerometer
    - angular rates from gyroscope
    
    Usage:
        imu = IMUReader()
        imu.update()
        data = imu.get_data()
    """
    
    def __init__(self, sda_pin=I2C_SDA, scl_pin=I2C_SCL, imu_type=IMU_TYPE):
        self.i2c = busio.I2C(scl_pin, sda_pin)
        self.imu_type = imu_type
        self.imu = None
        
        # Current orientation data
        self.heading = 0.0      # Yaw (0-360 degrees)
        self.pitch = 0.0        # Pitch (-90 to 90 degrees)
        self.roll = 0.0         # Roll (-180 to 180 degrees)
        
        # Raw sensor data
        self.accel = (0, 0, 0)  # Acceleration (x, y, z) in m/s^2
        self.gyro = (0, 0, 0)   # Angular velocity (x, y, z) in deg/s
        self.mag = (0, 0, 0)    # Magnetic field (x, y, z) in uT
        
        # For gyro integration
        self._last_time = time.monotonic()
        
        # Initialize the specific IMU
        self._init_imu()
    
    def _init_imu(self):
        """Initialize the IMU based on type."""
        try:
            if self.imu_type == "ICM20948":
                self._init_icm20948()
            elif self.imu_type == "MPU6050":
                self._init_mpu6050()
            elif self.imu_type == "MPU9250":
                self._init_mpu9250()
            elif self.imu_type == "BNO055":
                self._init_bno055()
            elif self.imu_type == "LSM9DS1":
                self._init_lsm9ds1()
            else:
                print("IMU: Unknown type '%s', using ICM20948" % self.imu_type)
                self._init_icm20948()
        except Exception as e:
            print("IMU: Init failed -", e)
            self.imu = None
    
    def _init_icm20948(self):
        """Initialize ICM-20948 (SparkFun 9DoF Qwiic)."""
        try:
            from adafruit_icm20x import ICM20948
            self.imu = ICM20948(self.i2c, address=IMU_ADDRESS)
            self.imu_type = "ICM20948"
            print("IMU: ICM-20948 initialized (9DoF with magnetometer)")
        except ImportError:
            print("IMU: adafruit_icm20x library not found")
            print("     Install: adafruit_icm20x.mpy in /lib folder")
            self.imu = None
        except Exception as e:
            print("IMU: ICM-20948 error -", e)
            self.imu = None
    
    def _init_mpu6050(self):
        """Initialize MPU6050."""
        try:
            import adafruit_mpu6050
            self.imu = adafruit_mpu6050.MPU6050(self.i2c, address=IMU_ADDRESS)
            print("IMU: MPU6050 initialized")
        except ImportError:
            print("IMU: adafruit_mpu6050 library not found")
            self._init_mpu_raw()
    
    def _init_mpu_raw(self):
        """Initialize MPU6050 with raw I2C (no library needed)."""
        # Wake up MPU6050 (write 0 to PWR_MGMT_1 register)
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto(IMU_ADDRESS, bytes([0x6B, 0x00]))
            self.imu = "MPU_RAW"
            print("IMU: MPU6050 initialized (raw I2C)")
        except Exception as e:
            print("IMU: MPU6050 not found at address 0x%02X" % IMU_ADDRESS)
            self.imu = None
        finally:
            self.i2c.unlock()
    
    def _init_mpu9250(self):
        """Initialize MPU9250."""
        # MPU9250 is similar to MPU6050 but has magnetometer
        self._init_mpu6050()
        # TODO: Initialize AK8963 magnetometer
    
    def _init_bno055(self):
        """Initialize BNO055."""
        try:
            import adafruit_bno055
            self.imu = adafruit_bno055.BNO055_I2C(self.i2c)
            print("IMU: BNO055 initialized")
        except ImportError:
            print("IMU: adafruit_bno055 library not found")
            self.imu = None
    
    def _init_lsm9ds1(self):
        """Initialize LSM9DS1."""
        try:
            import adafruit_lsm9ds1
            self.imu = adafruit_lsm9ds1.LSM9DS1_I2C(self.i2c)
            print("IMU: LSM9DS1 initialized")
        except ImportError:
            print("IMU: adafruit_lsm9ds1 library not found")
            self.imu = None
    
    def _read_mpu_raw(self):
        """Read raw data from MPU6050 via I2C."""
        while not self.i2c.try_lock():
            pass
        try:
            # Read 14 bytes starting from ACCEL_XOUT_H (0x3B)
            data = bytearray(14)
            self.i2c.writeto_then_readfrom(IMU_ADDRESS, bytes([0x3B]), data)
            
            # Parse accelerometer (in g, convert to m/s^2)
            ax = self._bytes_to_int(data[0], data[1]) / 16384.0 * 9.81
            ay = self._bytes_to_int(data[2], data[3]) / 16384.0 * 9.81
            az = self._bytes_to_int(data[4], data[5]) / 16384.0 * 9.81
            self.accel = (ax, ay, az)
            
            # Skip temperature (bytes 6-7)
            
            # Parse gyroscope (in deg/s)
            gx = self._bytes_to_int(data[8], data[9]) / 131.0
            gy = self._bytes_to_int(data[10], data[11]) / 131.0
            gz = self._bytes_to_int(data[12], data[13]) / 131.0
            self.gyro = (gx, gy, gz)
            
        except Exception as e:
            pass
        finally:
            self.i2c.unlock()
    
    def _bytes_to_int(self, msb, lsb):
        """Convert two bytes to signed 16-bit integer."""
        value = (msb << 8) | lsb
        if value > 32767:
            value -= 65536
        return value
    
    def update(self):
        """Read IMU and update orientation. Call this frequently."""
        if self.imu is None:
            return
        
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now
        
        try:
            # Read based on IMU type
            if self.imu == "MPU_RAW":
                self._read_mpu_raw()
            elif hasattr(self.imu, 'acceleration'):
                # Adafruit library IMU (ICM-20948, MPU6050, etc.)
                self.accel = self.imu.acceleration
                if hasattr(self.imu, 'gyro'):
                    self.gyro = self.imu.gyro
                if hasattr(self.imu, 'magnetic'):
                    try:
                        self.mag = self.imu.magnetic
                    except Exception:
                        pass  # Some IMUs may not have magnetometer ready
            
            # Calculate pitch and roll from accelerometer
            ax, ay, az = self.accel
            if az != 0 or ay != 0:  # Avoid division by zero
                self.pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 180 / math.pi
                self.roll = math.atan2(ay, az) * 180 / math.pi
            
            # Calculate heading from magnetometer (tilt-compensated)
            if self.mag != (0, 0, 0) and self.mag[0] is not None:
                mx, my, mz = self.mag
                
                # Tilt compensation for more accurate heading
                pitch_rad = math.radians(self.pitch)
                roll_rad = math.radians(self.roll)
                
                # Compensate magnetometer readings for tilt
                mx_comp = mx * math.cos(pitch_rad) + mz * math.sin(pitch_rad)
                my_comp = (mx * math.sin(roll_rad) * math.sin(pitch_rad) + 
                          my * math.cos(roll_rad) - 
                          mz * math.sin(roll_rad) * math.cos(pitch_rad))
                
                # Calculate heading
                self.heading = math.atan2(-my_comp, mx_comp) * 180 / math.pi
                if self.heading < 0:
                    self.heading += 360
            else:
                # Fallback: Integrate gyroscope for heading (will drift)
                if self.gyro[2] is not None:
                    gz = self.gyro[2]
                    self.heading = (self.heading + gz * dt) % 360
                
        except Exception as e:
            pass
    
    def get_data(self):
        """Return current IMU data as dictionary."""
        return {
            "heading": self.heading,
            "pitch": self.pitch,
            "roll": self.roll,
            "accel": self.accel,
            "gyro": self.gyro,
            "mag": self.mag
        }
    
    def get_heading(self):
        """Get heading (yaw) in degrees (0-360)."""
        return self.heading


# -----------------------------
# Standalone Test
# -----------------------------
if __name__ == "__main__":
    print("IMU Test Mode")
    imu = IMUReader()
    
    while True:
        imu.update()
        data = imu.get_data()
        
        print("Heading: %6.1f  Pitch: %6.1f  Roll: %6.1f" % (
            data["heading"], data["pitch"], data["roll"]))
        
        time.sleep(0.1)
