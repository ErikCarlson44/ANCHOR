"""
esc.py — ESC throttle test for Flycolor Fairy 90A
Raspberry Pi Pico — CircuitPython 8+
Pin: GP2

Throttle scale:
    0.0 = stop   (1500us neutral)
    0.5 = 50%    (1750us)
    1.0 = full   (2000us)
"""

import time
import board
import pwmio

# ── Config ────────────────────────────────────────────────────────────────────
ESC_PIN       = board.GP14
NEUTRAL_US    = 1500    # 0%   throttle
FULL_US       = 2000    # 100% throttle

# ── Init ──────────────────────────────────────────────────────────────────────
esc = pwmio.PWMOut(ESC_PIN, frequency=50, duty_cycle=0)

def set_throttle(level):
    """
    Set throttle using 0.0 to 1.0 scale.
        0.0 = neutral / stop  (1500us)
        0.5 = half throttle   (1750us)
        1.0 = full throttle   (2000us)
    """
    level     = max(0.0, min(1.0, level))
    pulse_us  = NEUTRAL_US + int(level * (FULL_US - NEUTRAL_US))
    esc.duty_cycle = int((pulse_us / 20000) * 65535)
    print("Throttle: {:.0f}%  ({}us)".format(level * 100, pulse_us))

# ── Arming sequence ───────────────────────────────────────────────────────────
print("=" * 40)
print("ESC Throttle Test — GP2")
print("Scale: 0.0=stop  0.5=half  1.0=full")
print("=" * 40)
print("\nArming — sending full throttle...")
print("Plug battery in NOW")
set_throttle(1.0)
time.sleep(5)

print("Dropping to minimum...")
esc.duty_cycle = int((1000 / 20000) * 65535)
time.sleep(3)

print("Neutral — ARMED\n")
set_throttle(0.0)
time.sleep(3)

# ── Throttle cycle ────────────────────────────────────────────────────────────
print("Starting throttle cycle...\n")

while True:
    print("--- STOP ---")
    set_throttle(0.0)
    time.sleep(4)

    print("--- 25% ---")
    set_throttle(0.25)
    time.sleep(4)

    print("--- 50% ---")
    set_throttle(0.5)
    time.sleep(4)

    print("--- 75% ---")
    set_throttle(0.75)
    time.sleep(4)

    print("--- FULL ---")
    set_throttle(1.0)
    time.sleep(4)

    print("--- back to STOP ---")
    set_throttle(0.0)
    time.sleep(4)

    print()
