from machine import Pin, PWM
from time import sleep, time

# GPIO pin (BCM numbering)
PWM_PIN = 0  # GPIO 18 (change if needed)

# Pulse widths (µs)
LOW_PULSE = 1000   # LEFT
HIGH_PULSE = 2000   # RIGHT
MID_PULSE = 1500    # CENTER

FRAME_US = 19000    # Total frame: 19 ms (~52.6 Hz)
FREQUENCY = 1000000 / FRAME_US  # Convert to Hz

current_pulse = MID_PULSE  # Start centered


def setup():
    """Initialize GPIO and display instructions."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PWM_PIN, GPIO.OUT)
    
    # Create PWM object at the calculated frequency
    global pwm
    pwm = GPIO.PWM(PWM_PIN, FREQUENCY)
    pwm.start(0)  # Start with 0% duty cycle
    
    print("Rudder Control - Use WASD to control: A=Left, D=Right, W/S=Center")
    print("Press Ctrl+C to exit")


def send_pwm(high_us):
    """Send PWM pulse with specified high time."""
    # Calculate duty cycle as percentage
    duty_cycle = (high_us / FRAME_US) * 100
    pwm.ChangeDutyCycle(duty_cycle)
    # Keep the pulse for one frame duration
    time.sleep(FRAME_US / 1000000)


def cleanup():
    """Clean up GPIO on exit."""
    pwm.stop()
    GPIO.cleanup()
    print("\nGPIO cleanup complete")


def main():
    """Main control loop."""
    global current_pulse
    
    setup()
    
    try:
        while True:
            # Check for keyboard input
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                c = sys.stdin.read(1).lower()
                
                if c == 'a':
                    current_pulse = LOW_PULSE
                    print("Rudder: LEFT")
                elif c == 'd':
                    current_pulse = HIGH_PULSE
                    print("Rudder: RIGHT")
                elif c in ['w', 's']:
                    current_pulse = MID_PULSE
                    print("Rudder: CENTER")
                elif c == 'q':
                    break
            
            # Send PWM continuously
            send_pwm(current_pulse)
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        cleanup()


if __name__ == "__main__":
    import select
    main()
