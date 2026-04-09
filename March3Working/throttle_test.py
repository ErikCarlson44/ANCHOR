"""
Interactive ESC throttle test over USB serial (Thonny REPL).

CircuitPython on Raspberry Pi Pico — same stack as code.py (board + pwmio).
If you see "no module named 'machine'", you are not on MicroPython; use this file as-is on the Pico.

W / S / X keys. CR/LF from Thonny are ignored.
"""

import sys
import time

import board
import pwmio

try:
    import usb_cdc
except ImportError:
    usb_cdc = None

# --- Configuration (ESC signal on GP14, same as main firmware) ---
ESC_PIN = board.GP14
PWM_FREQ = 50
THROTTLE_STEP = 5

PULSE_MIN = 1000
PULSE_MAX = 2000
PULSE_NEUTRAL = 1000


def us_to_duty(pulse_us):
    """50 Hz → 20 ms period; duty_cycle is 0..65535."""
    pulse_us = max(500, min(2500, int(pulse_us)))
    return int(pulse_us / 20000 * 65535)


def set_throttle_percent(pwm, percent):
    percent = max(0, min(100, percent))
    pulse = PULSE_MIN + int((PULSE_MAX - PULSE_MIN) * percent / 100)
    pwm.duty_cycle = us_to_duty(pulse)
    return pulse


def arm_esc(pwm):
    print("Arming ESC... hold neutral 2s.")
    pwm.duty_cycle = us_to_duty(PULSE_NEUTRAL)
    time.sleep(2)
    print("Armed. Keys: W +  S -  X stop")


def get_char_nonblocking():
    """Non-blocking byte from USB REPL (Thonny)."""
    if usb_cdc is not None:
        console = usb_cdc.console
        if console is not None and console.in_waiting:
            b = console.read(1)
            if b:
                return chr(b[0]) if isinstance(b[0], int) else b.decode("utf-8", "ignore")[:1]
    if getattr(sys.stdin, "in_waiting", 0):
        c = sys.stdin.read(1)
        return c if c else None
    return None


# Neutral immediately, then arm
pwm = pwmio.PWMOut(ESC_PIN, frequency=PWM_FREQ, duty_cycle=us_to_duty(PULSE_NEUTRAL))
arm_esc(pwm)

throttle = 0
set_throttle_percent(pwm, throttle)

print("\n=== RC Boat ESC (CircuitPython) ===")
print("  W = +{}%   S = -{}%   X = stop".format(THROTTLE_STEP, THROTTLE_STEP))
print("===================================\n")
print("Throttle: {}%".format(throttle))

while True:
    char = get_char_nonblocking()
    if char is not None:
        if char in "\r\n":
            continue
        char = char.lower()

        if char == "w":
            throttle = min(100, throttle + THROTTLE_STEP)
            pulse = set_throttle_percent(pwm, throttle)
            print("Throttle: {}%  ({} us)  [UP]".format(throttle, pulse))

        elif char == "s":
            throttle = max(0, throttle - THROTTLE_STEP)
            pulse = set_throttle_percent(pwm, throttle)
            print("Throttle: {}%  ({} us)  [DOWN]".format(throttle, pulse))

        elif char == "x":
            throttle = 0
            pulse = set_throttle_percent(pwm, throttle)
            print("Throttle: {}%  ({} us)  [STOP]".format(throttle, pulse))

    time.sleep(0.05)
