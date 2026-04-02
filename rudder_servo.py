# rudder_servo.py — small, smooth rudder movements in CircuitPython
# Wiring: servo signal wire → any PWM-capable pin (e.g. board.GP0 on Pico)

import board
import pwmio
import time
from adafruit_motor import servo

# ── Setup ──────────────────────────────────────────────────────────────────
pwm = pwmio.PWMOut(board.GP0, duty_cycle=0, frequency=50)
rudder = servo.Servo(pwm, min_pulse=500, max_pulse=2400)

# Servo center (neutral rudder). Adjust if yours drifts.
CENTER = 90
rudder.angle = CENTER

# ── Tuning knobs ───────────────────────────────────────────────────────────
STEP_DEG    = 1      # degrees moved per tick  (smaller = smoother)
TICK_SEC    = 0.015  # seconds between ticks   (≈15 ms → ~67 Hz update rate)
MAX_DEFLECT = 25     # max degrees from center  (tweak for your linkage)

# ── Smooth move helper ─────────────────────────────────────────────────────
def move_to(target_angle, current_angle):
    """Incrementally sweep from current_angle to target_angle."""
    target_angle = max(CENTER - MAX_DEFLECT,
                       min(CENTER + MAX_DEFLECT, target_angle))
    direction = 1 if target_angle > current_angle else -1

    while abs(target_angle - current_angle) > STEP_DEG:
        current_angle += direction * STEP_DEG
        rudder.angle = current_angle
        time.sleep(TICK_SEC)

    rudder.angle = target_angle   # snap to exact target
    return target_angle

# ── Demo sequence ──────────────────────────────────────────────────────────
current = CENTER

moves = [
    CENTER + 10,   # small starboard
    CENTER,        # return to center
    CENTER - 10,   # small port
    CENTER,        # return to center
    CENTER + 20,   # larger starboard
    CENTER - 20,   # sweep to port
    CENTER,        # back to neutral
]

for target in moves:
    current = move_to(target, current)
    time.sleep(0.4)   # hold at each position
