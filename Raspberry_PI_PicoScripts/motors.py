"""
Motors Module - Controls rudder servo and throttle ESC
Import this module and create a MotorController instance.
"""

import board
import pwmio

# -----------------------------
# Configuration - CHANGE THESE
# -----------------------------
RUDDER_PIN = board.GP12    # Rudder servo PWM
THROTTLE_PIN = board.GP13  # ESC/Throttle PWM

# -----------------------------
# Helper Functions
# -----------------------------
def clamp(x, lo, hi):
    """Clamp value between lo and hi."""
    return lo if x < lo else hi if x > hi else x

# -----------------------------
# Motor Controller Class
# -----------------------------
class MotorController:
    """
    Controls rudder servo and throttle ESC via PWM.
    
    PWM Signals (standard RC):
    - 1000 us = minimum (full left / off)
    - 1500 us = center (neutral)
    - 2000 us = maximum (full right / full throttle)
    
    Usage:
        motors = MotorController()
        motors.set_controls(throttle=0.5, steering=-1)  # 50% throttle, left
        motors.stop()  # Emergency stop
    """
    
    SERVO_FREQ = 50       # 50 Hz = 20ms period
    PERIOD_US = 20000     # 20,000 microseconds
    
    def __init__(self, rudder_pin=RUDDER_PIN, throttle_pin=THROTTLE_PIN):
        # Create PWM outputs
        self.rudder_pwm = pwmio.PWMOut(rudder_pin, frequency=self.SERVO_FREQ, duty_cycle=0)
        self.throttle_pwm = pwmio.PWMOut(throttle_pin, frequency=self.SERVO_FREQ, duty_cycle=0)
        
        # Current values
        self.throttle = 0.0   # 0 to 1
        self.steering = 0     # -1, 0, or 1
        self.armed = False
        
        # Set to neutral
        self._set_neutral()
        
        print("Motors: Initialized (Rudder: %s, Throttle: %s)" % (rudder_pin, throttle_pin))
    
    def _pulse_us_to_duty(self, pulse_us):
        """Convert pulse width (microseconds) to duty cycle (0-65535)."""
        pulse_us = clamp(pulse_us, 1000, 2000)
        return int((pulse_us / self.PERIOD_US) * 65535)
    
    def _set_neutral(self):
        """Set both outputs to neutral (1500 us)."""
        neutral = self._pulse_us_to_duty(1500)
        self.rudder_pwm.duty_cycle = neutral
        self.throttle_pwm.duty_cycle = neutral
    
    def set_controls(self, throttle, steering):
        """
        Set throttle and steering.
        
        Args:
            throttle: 0.0 to 1.0 (0 = stop, 1 = full)
            steering: -1 (left), 0 (center), or 1 (right)
        """
        self.throttle = clamp(throttle, 0.0, 1.0)
        self.steering = clamp(int(steering), -1, 1)
        
        # Rudder: -1 = 1000us (left), 0 = 1500us (center), 1 = 2000us (right)
        rudder_us = 1500 + (self.steering * 500)
        self.rudder_pwm.duty_cycle = self._pulse_us_to_duty(rudder_us)
        
        # Throttle: Only apply if armed
        if self.armed:
            # 0 = 1500us (neutral), 1 = 2000us (full)
            throttle_us = 1500 + (self.throttle * 500)
        else:
            throttle_us = 1500
        self.throttle_pwm.duty_cycle = self._pulse_us_to_duty(throttle_us)
    
    def stop(self):
        """Emergency stop - everything to neutral."""
        self.throttle = 0.0
        self.steering = 0
        self.armed = False
        self._set_neutral()
    
    def arm(self):
        """Arm the throttle (allow motor to spin)."""
        self.armed = True
    
    def disarm(self):
        """Disarm the throttle (motor cannot spin)."""
        self.armed = False
        self.set_controls(0, self.steering)


# -----------------------------
# Standalone Test
# -----------------------------
if __name__ == "__main__":
    import time
    
    print("Motor Test Mode")
    motors = MotorController()
    motors.arm()
    
    print("Testing rudder: Left -> Center -> Right")
    for steering in [-1, 0, 1]:
        motors.set_controls(0, steering)
        print("  Steering:", steering)
        time.sleep(1)
    
    print("Testing throttle: 0% -> 25% -> 50% -> 0%")
    for throttle in [0, 0.25, 0.5, 0]:
        motors.set_controls(throttle, 0)
        print("  Throttle:", throttle)
        time.sleep(1)
    
    motors.stop()
    print("Test complete")
