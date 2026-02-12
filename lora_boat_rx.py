import time
import struct
import binascii

import board
import busio
import digitalio
import pwmio

import rfm9x

# -----------------------------
# User settings
# -----------------------------
FREQ_MHZ = 915.0
TX_POWER = 13

SPI_CS = board.GP17      # CHANGE to your wiring
RESET_PIN = board.GP16   # CHANGE to your wiring

RUDDER_PIN = board.GP18      # your Pico pin
THROTTLE_PIN = board.GP13    # your Pico pin

FAILSAFE_TIMEOUT_S = 0.6
PACKET_MAGIC = b"RC"

# -----------------------------
# PWM helpers (50 Hz servo pulses)
# -----------------------------
SERVO_HZ = 50
PERIOD_US = 1_000_000 // SERVO_HZ  # 20,000 us at 50 Hz

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def percent_to_pulse_us(pct, center=1500, span=500):
    pct = clamp(pct, -100, 100)
    return int(center + (pct / 100.0) * span)

def set_pulse_us(pwm, pulse_us):
    pulse_us = clamp(pulse_us, 500, 2500)
    duty = int((pulse_us / PERIOD_US) * 65535)
    pwm.duty_cycle = duty

def set_outputs(rudder_pwm, throttle_pwm, rudder_pct, throttle_pct, armed):
    rudder_us = percent_to_pulse_us(rudder_pct, center=1500, span=450)
    set_pulse_us(rudder_pwm, rudder_us)

    if armed:
        throttle_us = percent_to_pulse_us(throttle_pct, center=1500, span=500)
    else:
        throttle_us = 1500
    set_pulse_us(throttle_pwm, throttle_us)

def parse_packet(pkt: bytes):
    # 11 bytes: <2sHbbBI
    if pkt is None or len(pkt) < 11:
        return None
    magic, seq, thr, rud, arm, crc = struct.unpack("<2sHbbBI", pkt[:11])
    if magic != PACKET_MAGIC:
        return None

    calc = binascii.crc32(pkt[:7]) & 0xFFFFFFFF
    if calc != crc:
        return None

    thr = int(clamp(thr, -100, 100))
    rud = int(clamp(rud, -100, 100))
    arm = 1 if arm else 0
    return seq, thr, rud, arm

# -----------------------------
# Main
# -----------------------------
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = digitalio.DigitalInOut(SPI_CS)
reset = digitalio.DigitalInOut(RESET_PIN)

rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, FREQ_MHZ)
rfm9x.tx_power = TX_POWER

rudder_pwm = pwmio.PWMOut(RUDDER_PIN, frequency=SERVO_HZ, duty_cycle=0)
throttle_pwm = pwmio.PWMOut(THROTTLE_PIN, frequency=SERVO_HZ, duty_cycle=0)

print("LoRa receiver ready (Pico + CircuitPython).")

last_ok = time.monotonic()
armed = 0
rudder = 0
throttle = 0
set_outputs(rudder_pwm, throttle_pwm, 0, 0, 0)

while True:
    pkt = rfm9x.receive(timeout=0.1)
    parsed = parse_packet(pkt)
    now = time.monotonic()

    if parsed:
        seq, throttle, rudder, armed = parsed
        last_ok = now
        set_outputs(rudder_pwm, throttle_pwm, rudder, throttle, armed)
        print("seq=%d thr=%d rud=%d arm=%d rssi=%s" % (seq, throttle, rudder, armed, str(rfm9x.last_rssi)))

    if (now - last_ok) > FAILSAFE_TIMEOUT_S:
        armed = 0
        throttle = 0
        rudder = 0
        set_outputs(rudder_pwm, throttle_pwm, 0, 0, 0)
