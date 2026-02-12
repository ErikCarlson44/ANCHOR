import time
import math
import random
from dataclasses import dataclass

# -----------------------------
# Optional servo support (pigpio)
# -----------------------------
SERVO_GPIO = 18  # PWM pin
USE_SERVO = True

try:
    import pigpio
except ImportError:
    pigpio = None


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def deg_to_pulsewidth(deg: float) -> int:
    """
    Map 0..180 degrees to ~500..2500 us (typical servo range).
    Adjust if your servo wants a different range.
    """
    deg = clamp(deg, 0.0, 180.0)
    return int(500 + (deg / 180.0) * 2000)


# -----------------------------
# Radar target structure
# -----------------------------
@dataclass
class Target:
    distance: float  # mm
    angle: float     # deg
    speed: float     # cm/s
    x: float         # mm
    y: float         # mm


# -----------------------------
# Fake RD03D (simulates 3 moving targets)
# -----------------------------
class FakeRD03D:
    def __init__(self):
        self.t0 = time.time()
        self.multi = False
        self._targets = [Target(0, 0, 0, 0, 0) for _ in range(3)]

    def set_multi_mode(self, enabled: bool):
        self.multi = enabled

    def update(self) -> bool:
        """
        Return True most of the time, occasionally False to mimic dropouts.
        """
        # ~5% chance of "no data"
        if random.random() < 0.05:
            return False

        t = time.time() - self.t0

        # Simulate 3 targets with different motion
        for i in range(3):
            phase = i * 1.3
            # angle in -60..60 deg
            angle = 60.0 * math.sin(0.7 * t + phase)

            # distance in 800..4000 mm
            distance = 2400.0 + 1600.0 * math.sin(0.4 * t + phase + 0.5)

            # speed in cm/s (roughly derivative-ish)
            speed = 40.0 * math.cos(0.4 * t + phase + 0.5)

            # Convert polar-ish to x/y (simple)
            rad = math.radians(angle)
            x = distance * math.cos(rad)
            y = distance * math.sin(rad)

            self._targets[i] = Target(
                distance=distance,
                angle=angle,
                speed=speed,
                x=x,
                y=y
            )

        return True

    def get_target(self, n: int) -> Target:
        # n is 1..3 in your code
        idx = clamp(n - 1, 0, 2)
        return self._targets[int(idx)]


# -----------------------------
# Try real RD03D, else fallback
# -----------------------------
def get_radar():
    try:
        from rd03d import RD03D
        radar = RD03D()  # uses /dev/ttyAMA0 by default in your setup
        return radar, True
    except Exception as e:
        print(f"[INFO] Using FakeRD03D (real radar not available): {e}")
        return FakeRD03D(), False


def main():
    radar, is_real = get_radar()
    radar.set_multi_mode(True)

    pi = None
    if USE_SERVO and pigpio is not None:
        pi = pigpio.pi()
        if not pi.connected:
            print("[WARN] pigpio daemon not connected; servo disabled.")
            pi = None
        else:
            # Initialize servo at mid position
            pi.set_servo_pulsewidth(SERVO_GPIO, deg_to_pulsewidth(90))

    try:
        while True:
            if radar.update():
                target1 = radar.get_target(1)
                target2 = radar.get_target(2)
                target3 = radar.get_target(3)

                print('1 dist:', round(target1.distance, 1), 'mm Angle:', round(target1.angle, 1),
                      "deg Speed:", round(target1.speed, 1), "cm/s X:", round(target1.x, 1),
                      "mm Y:", round(target1.y, 1), "mm")
                print('2 dist:', round(target2.distance, 1), 'mm Angle:', round(target2.angle, 1),
                      "deg Speed:", round(target2.speed, 1), "cm/s X:", round(target2.x, 1),
                      "mm Y:", round(target2.y, 1), "mm")
                print('3 dist:', round(target3.distance, 1), 'mm Angle:', round(target3.angle, 1),
                      "deg Speed:", round(target3.speed, 1), "cm/s X:", round(target3.x, 1),
                      "mm Y:", round(target3.y, 1), "mm \n")

                # Servo behavior: point servo based on target1 angle
                # Radar angle approx -60..60 => map to servo 30..150 degrees
                if pi is not None:
                    servo_deg = 90 + (target1.angle * (60.0 / 60.0))  # -60..60 -> 30..150
                    servo_deg = clamp(servo_deg, 30, 150)
                    pi.set_servo_pulsewidth(SERVO_GPIO, deg_to_pulsewidth(servo_deg))

            else:
                print('No radar data received.')

                # Optional: move servo to center on "no data"
                if pi is not None:
                    pi.set_servo_pulsewidth(SERVO_GPIO, deg_to_pulsewidth(90))

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        if pi is not None:
            pi.set_servo_pulsewidth(SERVO_GPIO, 0)  # stop pulses
            pi.stop()


if __name__ == "__main__":
    main()
