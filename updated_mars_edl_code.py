#!/usr/bin/env python3
"""
ASTE 577 - Mars Human Landing EDL Flight Computer
Built on Combined MPU-6050 Strapdown IMU, BMP280 Barometric Telemetry & HIL LEDs
Target: Raspberry Pi 3B+

Wiring (Raspberry Pi 3B+ Physical Header Pins):
  MPU-6050:
    VCC -> Pin 01 (3.3V) or Pin 04 (5V)
    GND -> Pin 06 (GND)
    SCL -> Pin 05 (GPIO 3 / I2C1 SCL)
    SDA -> Pin 03 (GPIO 2 / I2C1 SDA)

  BMP280:
    VCC -> Pin 17 (3.3V) or Pin 01 (3.3V)
    GND -> Pin 09 (GND)
    SCL -> Pin 05 (GPIO 3 / I2C1 SCL - shared)
    SDA -> Pin 03 (GPIO 2 / I2C1 SDA - shared)

  Hardware-in-the-Loop LEDs:
    Critical Event LED (Red)    -> GPIO 17 (Pin 11) -> 220-330 ohm resistor -> GND (Pin 14)
    10s Countdown LED (Yellow)  -> GPIO 27 (Pin 13) -> 220-330 ohm resistor -> GND (Pin 20)
    (Tip: Pass `--event-led-pin 4` if using a single LED wired to GPIO 4 / Pin 07)
"""

import argparse
import csv
import math
import os
import smtplib
import time
from email.message import EmailMessage

# Ensure Tkinter uses the Raspberry Pi's primary desktop display even when launched over SSH
if "DISPLAY" not in os.environ:
    os.environ["DISPLAY"] = ":0"

# ---------------------------------------------------------------------------
# Sensor & Pin Configuration Constants
# ---------------------------------------------------------------------------
MPU_DEFAULT_ADDRESS = 0x68
BMP_DEFAULT_ADDRESS = 0x76       # Use 0x77 if i2cdetect shows 77
I2C_BUS_NUMBER = 1

LOCAL_SEA_LEVEL_HPA = 1013.25    # Update with local barometric pressure
BMP_SAMPLE_INTERVAL_SEC = 0.25   # 4 Hz non-blocking polling for BMP280
PRINT_INTERVAL_SEC = 0.5         # 2 Hz live status line readout

# GPIO Pins (BCM numbering)
EVENT_LED_PIN_BCM = 17           # Physical Pin 11 (Critical Event ON/OFF LED)
COUNTDOWN_LED_PIN_BCM = 27       # Physical Pin 13 (10s Landing Countdown Pulsing LED)

LED_PATTERNS = {
    "off": (None, 0.0),
    "on": (0.0, None),
    "calibrating": (0.1, 0.1),   # 5 Hz fast blink during calibration
    "countdown": (0.25, 0.25),   # 2 Hz pulse during 10s landing countdown
    "fault": (0.08, 0.08),       # rapid warning strobe during atmospheric fault
}

ACCEL_RANGE_G = 8
GYRO_RANGE_DPS = 2000
DLPF_CODE_98HZ = 0x02

GRAVITY = 9.80665                # m/s^2
KP_TILT = 2.0

STATIONARY_ACCEL_TOL = 0.12 * GRAVITY
STATIONARY_GYRO_TOL = math.radians(4.0)
TILT_ACCEL_TOL = 0.25 * GRAVITY
TILT_GYRO_TOL = math.radians(30.0)
FREEFALL_SPECIFIC_FORCE = 0.35 * GRAVITY

ANG_ACC_WINDOW = 5
MAX_DT = 0.05                    # s; ignore stalled loop step

# ---------------------------------------------------------------------------
# Mission Requirement 1, 2, 6, 8, 9 Parameters
# ---------------------------------------------------------------------------
MARS_GRAVITY = 3.721             # m/s^2
ENTRY_INTERFACE_ALT_M = 125000.0 # 125 km Entry Interface trigger
ENTRY_VELOCITY_MS = 5500.0       # m/s initial entry velocity
ENTRY_FPA_DEG = -14.5            # Entry flight path angle (deg)
PARACHUTE_DEPLOY_ALT_M = 11000.0 # 11 km Supersonic Parachute Deploy trigger
HEATSHIELD_SEP_ALT_M = 7000.0    # 7 km Heat Shield Separation trigger
TERMINAL_DESCENT_ALT_M = 1500.0  # 1.5 km Terminal Descent & 10s Countdown trigger
SIM_TIME_SCALE = 8.0             # Compresses upper-atmosphere entry (~25s to Terminal Descent)

# Fault Detection Thresholds relative to room baseline (Requirement 6 & 7)
TEMP_FAULT_DELTA_C = 2.5         # +2.5 C above startup baseline (warm breath / heat)
PRESS_FAULT_DELTA_HPA = 1.5      # +1.5 hPa above startup baseline (blowing air / fan)

# Terminal Descent Drop & Bounce Capture (Requirement 4 & 8)
LANDING_COUNTDOWN_SEC = 10.0     # 10-second landing countdown
IMPACT_TRIGGER_G = 2.0           # Touchdown impact threshold (> 2.0 G)
BOUNCE_CAPTURE_SEC = 4.0         # Record bounces for 4.0 s after first touchdown

# Landing Notification via Email / SMS Gateway (Requirement 9)
# Tip: For SMS text message, set RECIPIENT_EMAIL to your carrier gateway:
#   Verizon: 1234567890@vtext.com | T-Mobile: 1234567890@tmomail.net | AT&T: 1234567890@txt.att.net
ENABLE_LANDING_MESSAGE = False
SMTP_SERVER = "smtp.gmail.com"
SMTP_PORT = 587
SENDER_EMAIL = "your_team_email@gmail.com"
SENDER_APP_PASSWORD = "your_16_char_app_password"
RECIPIENT_EMAIL = "recipient_or_phone_gateway@example.com"


# ---------------------------------------------------------------------------
# Vector & Quaternion Math Utilities
# ---------------------------------------------------------------------------
def vadd(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vscale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def vdot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vcross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def vnorm(a):
    return math.sqrt(vdot(a, a))


def vnormalize(a):
    n = vnorm(a)
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    return vscale(a, 1.0 / n)


def vmean(samples):
    n = float(len(samples))
    return (
        sum(s[0] for s in samples) / n,
        sum(s[1] for s in samples) / n,
        sum(s[2] for s in samples) / n,
    )


def quat_normalize(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n < 1e-12:
        return (1.0
