"""
=============================================================================
MOTOR CONTROL TEST - Raspberry Pi Pico
=============================================================================
Test script for motor/ESC and rudder servo control.

Wiring:
  Rudder Servo Signal  -> GP12
  Throttle ESC Signal  -> GP13
  ESC/Servo VCC        -> 5V (external power for ESC)
  ESC/Servo GND        -> GND (common ground with Pico)

Usage:
  Copy to Pico as code.py
  Use keyboard in Thonny serial console:
    W/S   - Throttle up/down
    A/D   - Rudder left/right
    X     - Center rudder
    Q     - Stop (zero throttle)
    SPACE - Emergency stop
    1-5   - Set throttle to 10%, 25%, 50%, 75%, 100%
    R     - Arm motors
    
=============================================================================
"""

import board
import pwmio
import time
import supervisor
import sys

# =============================================================================
# CONFIGURATION
# =============================================================================
RUDDER_PIN = board.GP12    # Rudder servo PWM
THROTTLE_PIN = board.GP13  # ESC/Throttle PWM

# =============================================================================
# HELPER FUNCTIONS
# =============================================================================
def clamp(x, lo, hi):
    """Clamp value between lo and hi."""
    return lo if x < lo else hi if x > hi else x

# =============================================================================
# MOTOR CONTROLLER CLASS
# =============================================================================
class MotorController:
    """
    Controls rudder servo and throttle ESC via PWM.
    
    PWM Signals (standard RC):
    - 1000 us = minimum (full left / off)
    - 1500 us = center (neutral)
    - 2000 us = maximum (full right / full throttle)
    
    Usage:
        motors = MotorController()
        motors.arm()
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
        
        print("Motors: Initialized (Rudder: GP12, Throttle: GP13)")
    
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
        print("Motors: ARMED")
    
    def disarm(self):
        """Disarm the throttle (motor cannot spin)."""
        self.armed = False
        self.set_controls(0, self.steering)
        print("Motors: DISARMED")


# =============================================================================
# MAIN TEST
# =============================================================================
def main():
    print("=" * 50)
    print("MOTOR CONTROL TEST")
    print("=" * 50)
    print()
    print("Wiring:")
    print("  Rudder Servo  -> GP12")
    print("  Throttle ESC  -> GP13")
    print()
    
    # Initialize
    motors = MotorController()
    
    print()
    print("Controls:")
    print("  W/S   - Throttle up/down (10% steps)")
    print("  A/D   - Rudder left/right")
    print("  X     - Center rudder")
    print("  Q     - Stop (zero throttle)")
    print("  SPACE - Emergency stop (disarm)")
    print("  R     - Arm motors")
    print("  1-5   - Set throttle to 10/25/50/75/100%")
    print()
    print("=" * 50)
    print("Motors DISARMED - Press 'R' to arm")
    print("=" * 50)
    
    throttle_pct = 0.0
    
    while True:
        # Read keyboard input
        if supervisor.runtime.serial_bytes_available:
            char = sys.stdin.read(1).lower()
            
            if char == 'r':
                motors.arm()
            
            elif char == 'w':
                throttle_pct = min(1.0, throttle_pct + 0.1)
                motors.set_controls(throttle_pct, motors.steering)
                print("Throttle: %d%%" % (throttle_pct * 100))
            
            elif char == 's':
                throttle_pct = max(0.0, throttle_pct - 0.1)
                motors.set_controls(throttle_pct, motors.steering)
                print("Throttle: %d%%" % (throttle_pct * 100))
            
            elif char == 'a':
                motors.set_controls(throttle_pct, -1)
                print("Rudder: LEFT")
            
            elif char == 'd':
                motors.set_controls(throttle_pct, 1)
                print("Rudder: RIGHT")
            
            elif char == 'x':
                motors.set_controls(throttle_pct, 0)
                print("Rudder: CENTER")
            
            elif char == 'q':
                throttle_pct = 0.0
                motors.set_controls(0, 0)
                print("STOPPED - Throttle: 0%, Rudder: CENTER")
            
            elif char == ' ':
                throttle_pct = 0.0
                motors.stop()
                print("EMERGENCY STOP - Motors disarmed")
            
            elif char == '1':
                throttle_pct = 0.1
                motors.set_controls(0.1, motors.steering)
                print("Throttle: 10%")
            
            elif char == '2':
                throttle_pct = 0.25
                motors.set_controls(0.25, motors.steering)
                print("Throttle: 25%")
            
            elif char == '3':
                throttle_pct = 0.5
                motors.set_controls(0.5, motors.steering)
                print("Throttle: 50%")
            
            elif char == '4':
                throttle_pct = 0.75
                motors.set_controls(0.75, motors.steering)
                print("Throttle: 75%")
            
            elif char == '5':
                throttle_pct = 1.0
                motors.set_controls(1.0, motors.steering)
                print("Throttle: 100%")
        
        time.sleep(0.05)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped")
