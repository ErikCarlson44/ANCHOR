# from machine import Pin, PWM
# from time import sleep, time 

# Configuration
SERVO_PIN = 1  # GP1 (change to your GPIO pin)
FREQ = 50  # 50 Hz -> 20 ms period
STEP_DEG = 5
STEP_DELAY = 0.05  # seconds

# Pulse width calibration (microseconds). Typical range ~1000..2000
MIN_PULSE_US = 1000
MAX_PULSE_US = 2000
CENTER_PULSE_US = 1500  # Center position (0°)

PW_PERIOD_US = 1000000 // FREQ  # 20000 for 50Hz

def angle_to_pulse_us(angle: int) -> int:
    """Map logical angle [-90..+90] to pulse width in microseconds."""
    if angle < -90:
        angle = -90
    if angle > 90:
        angle = 90
    t = (angle + 90) / 180.0  # 0..1
    return int(t * (MAX_PULSE_US - MIN_PULSE_US) + MIN_PULSE_US)

def pulse_us_to_duty_u16(pulse_us: int) -> int:
    """Convert pulse width in microseconds to PWM duty_u16 value."""
    duty = int(pulse_us / PW_PERIOD_US * 65535)
    if duty < 0:
        duty = 0
    if duty > 65535:
        duty = 65535
    return duty


def main():
    pwm = PWM(Pin(SERVO_PIN))
    pwm.freq(FREQ)

    try:
        # Center position (0°)
        center_angle = 0
        print(f"Center position: {center_angle}°")
        
        # Move to center first
        pulse = angle_to_pulse_us(center_angle)
        duty = pulse_us_to_duty_u16(pulse)
        pwm.duty_u16(duty)
        sleep(0.5)  # Wait for servo to settle
        
        # Define sweep range: ±75 degrees from center
        left_angle = center_angle - 75
        right_angle = center_angle + 75
        
        print(f"Sweeping from {left_angle}° to {right_angle}°")
        
        # Sweep right (center -> +75°)
        angle = center_angle
        while angle <= right_angle:
            pulse = angle_to_pulse_us(angle)
            duty = pulse_us_to_duty_u16(pulse)  # Corrected function call
            pwm.duty_u16(duty)
            print(f"Angle: {angle}°")
            sleep(STEP_DELAY)
            angle += STEP_DEG
        
        sleep(0.5)  # Pause at right extreme
        
        # Sweep left (+75° -> -75°)
        angle = right_angle
        while angle >= left_angle:
            pulse = angle_to_pulse_us(angle)
            duty = pulse_us_to_duty_u16(pulse)
            pwm.duty_u16(duty)
            print(f"Angle: {angle}°")
            sleep(STEP_DELAY)
            angle -= STEP_DEG
        
        sleep(0.5)  # Pause at left extreme
        
        # Return to center
        angle = left_angle
        while angle <= center_angle:
            pulse = angle_to_pulse_us(angle)
            duty = pulse_us_to_duty_u16(pulse)
            pwm.duty_u16(duty)
            print(f"Angle: {angle}°")
            sleep(STEP_DELAY)
            angle += STEP_DEG
        
        print("RC Boat Rotor sweep complete - returned to center")

    except KeyboardInterrupt:
        print("Stopping rotor servo")
    finally:
        pwm.deinit()
        print("Rotor servo stopped and PWM deinitialized")


if __name__ == "__main__":
    main()