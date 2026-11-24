"""
ASTE 577 - Mars Human Landing EDL Flight Computer
Hardware Configuration:
  - Raspberry Pi 3 B+ / Raspberry Pi 4 Model B
  - HiLetgo BMP280-3.3 Atmospheric Pressure & Temperature Sensor (I2C: 0x76 or 0x77)
  - GY-521 MPU-6050 6-Axis Accelerometer & Gyroscope (I2C: 0x68)
  - Hardware-in-the-Loop LEDs:
      * Red Critical Event LED   -> GPIO 17 (Physical Pin 11)
      * Yellow 10s Countdown LED -> GPIO 27 (Physical Pin 13)

Wiring Matches Attached Diagram (Physical Pins on 40-Pin Header):
  - GY-521 VCC -> Pin 1 (3.3V)   | BMP280 VCC -> Pin 17 (3.3V)
  - GY-521 GND -> Pin 6 (GND)    | BMP280 GND -> Pin 9 (GND)
  - GY-521 SDA -> Pin 3 (GPIO 2) | BMP280 SDA -> Pin 3 (GPIO 2 - shared)
  - GY-521 SCL -> Pin 5 (GPIO 3) | BMP280 SCL -> Pin 5 (GPIO 3 - shared)
  - Red Event LED (+)    -> Pin 11 (GPIO 17) via resistor -> Cathode (-) to Pin 14 (GND)
  - Yellow Countdown (+) -> Pin 13 (GPIO 27) via resistor -> Cathode (-) to Pin 20 (GND)

Setup Instructions:
  1. Enable I2C: `sudo raspi-config` -> Interface Options -> I2C -> Enable -> Reboot
  2. Install libraries:
       sudo apt update && sudo apt install -y python3-lgpio
       pip3 install adafruit-circuitpython-bmp280 adafruit-circuitpython-mpu6050 gpiozero
  3. Run script:
       python3 mars_edl_flight_computer.py
"""

import time
import csv
import math
import smtplib
from email.message import EmailMessage
import board
import adafruit_bmp280
import adafruit_mpu6050

try:
    from gpiozero import LED
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False

# ---------------------------------------------------------------------------
# 1. Mission & Sensor Configuration
# ---------------------------------------------------------------------------
BMP280_I2C_ADDRESS = 0x77        # Change to 0x77 if i2cdetect shows 77
MPU6050_I2C_ADDRESS = 0x68       # Default address when AD0 pin is unconnected
LOCAL_SEA_LEVEL_HPA = 1013.25    # Update with local barometric pressure
SAMPLE_INTERVAL_SEC = 0.1        # 10 Hz loop rate for real-time EDL & terminal descent capture
LOG_FILENAME = "mars_edl_flight_log.csv"

# Dynamic Fault Trigger Deltas (triggers relative to room baseline at startup)
TEMP_RISE_FAULT_DELTA_C = 2.5    # Blowing warm air or touching BMP280 (+2.5 C) triggers fault
PRESS_RISE_FAULT_DELTA_HPA = 1.5 # Blowing directly on BMP280 (+1.5 hPa) triggers fault

# GPIO Pins Matching Attached Wiring Diagram
EVENT_LED_PIN = 17               # Physical Pin 11 (Red LED)
COUNTDOWN_LED_PIN = 27           # Physical Pin 13 (Yellow LED)

# Automated Landing Notification Settings (Optional - fill in to enable email alert at landing)
ENABLE_EMAIL_ALERT = False
SMTP_SERVER = "smtp.gmail.com"
SMTP_PORT = 587
SENDER_EMAIL = "your_team_email@gmail.com"
SENDER_APP_PASSWORD = "your_app_password"
RECIPIENT_EMAIL = "recipient_email@example.com"

# ---------------------------------------------------------------------------
# 2. Mars EDL Simulation Parameters (Update with your team's exact report values)
# ---------------------------------------------------------------------------
MARS_GRAVITY = 3.721             # m/s^2
ENTRY_INTERFACE_ALT_M = 125000.0 # 125 km Mars atmospheric entry interface
ENTRY_VELOCITY_MS = 5500.0       # m/s initial entry velocity
ENTRY_FPA_DEG = -14.5            # Initial flight path angle (degrees)
PARACHUTE_DEPLOY_ALT_M = 11000.0 # 11 km supersonic parachute deploy trigger
HEATSHIELD_SEP_ALT_M = 7000.0    # 7 km heat shield separation trigger
TERMINAL_DESCENT_ALT_M = 1500.0  # 1.5 km powered terminal descent trigger
SIM_TIME_SCALE = 8.0             # Speeds up upper-atmosphere entry for live presentation

# Terminal Descent Drop & Bounce Capture Settings
DROP_IMPACT_THRESHOLD_G = 2.0    # Impact spike (> 2.0 G) confirms surface touchdown
POST_IMPACT_CAPTURE_SEC = 4.0    # Continues logging bounces for 4 seconds after first impact

# ---------------------------------------------------------------------------
# 3. Helper Functions: Telecom, LEDs, & Landing Notification
# ---------------------------------------------------------------------------
def send_telecom(sim_t, message):
    print(f"[TELECOM | T+{sim_t:6.1f}s] >>> {message}")

def send_landing_notification(summary_text):
    print(f"\n[LANDING ALERT DISPATCHED]: {summary_text}")
    if not ENABLE_EMAIL_ALERT:
        return
    try:
        msg = EmailMessage()
        msg.set_content(summary_text)
        msg["Subject"] = "ASTE 577 Mars EDL Flight Computer: Touchdown Confirmed"
        msg["From"] = SENDER_EMAIL
        msg["To"] = RECIPIENT_EMAIL

        with smtplib.SMTP(SMTP_SERVER, SMTP_PORT) as server:
            server.starttls()
            server.login(SENDER_EMAIL, SENDER_APP_PASSWORD)
            server.send_message(msg)
        print("[TELECOM] Landing email notification sent successfully.")
    except Exception as e:
        print(f"[TELECOM WARNING] Email dispatch failed: {e}")

# ---------------------------------------------------------------------------
# 4. Main Flight Computer Loop
# ---------------------------------------------------------------------------
def main():
    # Initialize I2C Sensors (GPIO 2 = SDA, GPIO 3 = SCL)
    i2c = board.I2C()
    bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=BMP280_I2C_ADDRESS)
    bmp280.sea_level_pressure = LOCAL_SEA_LEVEL_HPA

    mpu = adafruit_mpu6050.MPU6050(i2c, address=MPU6050_I2C_ADDRESS)
    mpu.accelerometer_range = adafruit_mpu6050.Range.RANGE_8_G

    # Safe GPIO LED initialization (Pin 11 = Red Event LED, Pin 13 = Yellow Countdown LED)
    event_led = None
    countdown_led = None
    if HAS_GPIO:
        try:
            event_led = LED(EVENT_LED_PIN)
            countdown_led = LED(COUNTDOWN_LED_PIN)
        except Exception as e:
            print(f"[GPIO WARNING] LEDs unavailable ({e}); continuing in console-only mode.")

    # Capture room baseline from BMP280 for reliable fault detection
    baseline_temp = bmp280.temperature
    baseline_press = bmp280.pressure
    overtemp_limit = baseline_temp + TEMP_RISE_FAULT_DELTA_C
    overpress_limit = baseline_press + PRESS_RISE_FAULT_DELTA_HPA

    # Mission State Flags
    phase = "PRE_ENTRY"
    events_triggered = {
        "ENTRY": False,
        "PEAK_HEATING": False,
        "PEAK_DECEL": False,
        "CHUTE_DEPLOY": False,
        "HEATSHIELD_SEP": False,
        "TERMINAL_DESCENT": False,
        "LANDING": False
    }
    fault_active = False
    peak_heat_rate = 0.0
    peak_decel_g = 0.0
    max_landing_impact_g = 0.0
    landing_timestamp = None

    # Simulated Trajectory State
    sim_alt = ENTRY_INTERFACE_ALT_M
    sim_vel = ENTRY_VELOCITY_MS
    gamma_rad = math.radians(ENTRY_FPA_DEG)
    real_start_time = time.time()
    last_loop_time = real_start_time

    print("=== ASTE 577 MARS HUMAN LANDING FLIGHT COMPUTER ONLINE ===")
    print(f"BMP280 Baseline: {baseline_temp:.2f} C, {baseline_press:.2f} hPa")
    print(f"Fault Thresholds: > {overtemp_limit:.2f} C OR > {overpress_limit:.2f} hPa")
    print(f"Logging real-time telemetry & sensor data to '{LOG_FILENAME}'...\n")

    with open(LOG_FILENAME, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([
            "Real_Time_s", "Sim_Time_s", "Phase", "Sim_Alt_m", "Sim_Vel_ms",
            "BMP_Temp_C", "BMP_Press_hPa", "MPU_Accel_X", "MPU_Accel_Y", "MPU_Accel_Z",
            "MPU_Total_G", "MPU_Gyro_X", "MPU_Gyro_Y", "MPU_Gyro_Z", "Fault_Status"
        ])

        try:
            while True:
                now = time.time()
                dt_real = now - last_loop_time
                last_loop_time = now
                real_t = now - real_start_time
                sim_t = real_t * SIM_TIME_SCALE
                dt_sim = dt_real * SIM_TIME_SCALE

                # --- A. Read Physical Hardware Sensors (BMP280 + GY-521 MPU-6050) ---
                temp_c = bmp280.temperature
                pressure_hpa = bmp280.pressure
                ax, ay, az = mpu.acceleration
                gx, gy, gz = mpu.gyro
                measured_g = math.sqrt(ax**2 + ay**2 + az**2) / 9.80665

                # --- B. Fault Detection & Response Logic (BMP280 Trigger) ---
                if (temp_c > overtemp_limit or pressure_hpa > overpress_limit) and not fault_active:
                    fault_active = True
                    send_telecom(
                        sim_t,
                        f"FAULT DETECTED! Temp={temp_c:.1f}C, Press={pressure_hpa:.1f}hPa "
                        f"-> Engaging Thermal/Attitude Compensation Mode!"
                    )
                    if event_led:
                        event_led.blink(on_time=0.1, off_time=0.1, background=True)
                elif temp_c <= overtemp_limit and pressure_hpa <= overpress_limit and fault_active:
                    fault_active = False
                    send_telecom(sim_t, "FAULT CLEARED: Atmospheric parameters nominal.")
                    if event_led and not events_triggered["LANDING"]:
                        event_led.on()

                # --- C. Real-Time EDL Physics & Phase State Machine ---
                if sim_alt > TERMINAL_DESCENT_ALT_M:
                    # Exponential Mars atmosphere density model
                    rho = 0.020 * math.exp(-sim_alt / 11100.0)
                    cd_area_mass = 0.008 if not events_triggered["CHUTE_DEPLOY"] else 0.085
                    drag_decel = 0.5 * rho * (sim_vel ** 2) * cd_area_mass
                    sim_decel_g = drag_decel / 9.80665

                    # Sutton-Graves convective heating proxy (W/cm^2)
                    heat_rate = 1.83e-4 * math.sqrt(rho / 2.5) * (sim_vel ** 3) / 10000.0

                    # Integrate velocity, flight path angle, and altitude
                    dv = (-drag_decel - MARS_GRAVITY * math.sin(gamma_rad)) * dt_sim
                    dgamma = (-(MARS_GRAVITY * math.cos(gamma_rad)) / max(sim_vel, 50.0)) * dt_sim
                    sim_vel = max(80.0, sim_vel + dv)
                    gamma_rad = max(math.radians(-85.0), gamma_rad + dgamma)
                    sim_alt = max(TERMINAL_DESCENT_ALT_M, sim_alt + (sim_vel * math.sin(gamma_rad)) * dt_sim)

                    # Milestone 1: Entry Interface
                    if not events_triggered["ENTRY"]:
                        events_triggered["ENTRY"] = True
                        phase = "HYPERSONIC_ENTRY"
                        send_telecom(sim_t, f"CRITICAL EVENT: Atmospheric Entry Interface at {sim_alt/1000:.1f} km | V={sim_vel:.0f} m/s")
                        if event_led and not fault_active:
                            event_led.on()

                    # Milestone 2: Peak Heating
                    if heat_rate > peak_heat_rate:
                        peak_heat_rate = heat_rate
                    elif peak_heat_rate > 10.0 and not events_triggered["PEAK_HEATING"]:
                        events_triggered["PEAK_HEATING"] = True
                        send_telecom(sim_t, f"CRITICAL EVENT: Peak Heating Passed ({peak_heat_rate:.1f} W/cm^2) at Alt {sim_alt/1000:.1f} km")
                        if event_led and not fault_active:
                            event_led.blink(on_time=0.15, off_time=0.15, n=3, background=True)

                    # Milestone 3: Peak Deceleration
                    if sim_decel_g > peak_decel_g:
                        peak_decel_g = sim_decel_g
                    elif peak_decel_g > 1.5 and not events_triggered["PEAK_DECEL"]:
                        events_triggered["PEAK_DECEL"] = True
                        send_telecom(sim_t, f"CRITICAL EVENT: Peak Deceleration Passed ({peak_decel_g:.2f} G) at Alt {sim_alt/1000:.1f} km")
                        if event_led and not fault_active:
                            event_led.off()

                    # Milestone 4: Parachute Deploy
                    if sim_alt <= PARACHUTE_DEPLOY_ALT_M and not events_triggered["CHUTE_DEPLOY"]:
                        events_triggered["CHUTE_DEPLOY"] = True
                        phase = "PARACHUTE_DESCENT"
                        send_telecom(sim_t, f"CRITICAL EVENT: Supersonic Parachute Deployed at {sim_alt/1000:.2f} km")
                        if event_led and not fault_active:
                            event_led.on()

                    # Milestone 5: Heat Shield Separation
                    if sim_alt <= HEATSHIELD_SEP_ALT_M and not events_triggered["HEATSHIELD_SEP"]:
                        events_triggered["HEATSHIELD_SEP"] = True
                        send_telecom(sim_t, f"CRITICAL EVENT: Heat Shield Separation at {sim_alt/1000:.2f} km")
                        if event_led and not fault_active:
                            event_led.off()

                else:
                    # --- D. Terminal Descent & Physical Drop/Bounce Capture (MPU-6050) ---
                    if not events_triggered["TERMINAL_DESCENT"]:
                        events_triggered["TERMINAL_DESCENT"] = True
                        phase = "TERMINAL_DESCENT"
                        send_telecom(sim_t, "CRITICAL EVENT: Terminal Descent Started! Yellow 10s Countdown LED Pulsing -> Drop Lander Package!")
                        if countdown_led:
                            countdown_led.blink(on_time=0.25, off_time=0.25, background=True)

                    if not events_triggered["LANDING"]:
                        # Hold final approach hover until physical accelerometer detects impact
                        sim_vel = max(2.5, sim_vel - 10.0 * dt_real)
                        sim_alt = max(15.0, sim_alt - 120.0 * dt_real)

                        if measured_g > DROP_IMPACT_THRESHOLD_G:
                            events_triggered["LANDING"] = True
                            phase = "LANDED_BOUNCE"
                            sim_alt = 0.0
                            sim_vel = 0.0
                            landing_timestamp = now
                            max_landing_impact_g = measured_g
                            if countdown_led:
                                countdown_led.off()
                            if event_led:
                                event_led.on()
                            send_telecom(sim_t, f"CRITICAL EVENT: TOUCHDOWN DETECTED! Initial Impact = {measured_g:.2f} G (Recording bounces...)")
                    else:
                        # Continue logging post-impact bounces for POST_IMPACT_CAPTURE_SEC
                        if measured_g > max_landing_impact_g:
                            max_landing_impact_g = measured_g
                        if now - landing_timestamp >= POST_IMPACT_CAPTURE_SEC:
                            phase = "LANDED_REST"
                            send_landing_notification(
                                f"Mars Lander Touchdown Confirmed! Peak Impact: {max_landing_impact_g:.2f} G | "
                                f"Surface Temp: {temp_c:.1f} C | Pressure: {pressure_hpa:.1f} hPa"
                            )
                            break

                # --- E. Log Telemetry Row to CSV ---
                writer.writerow([
                    round(real_t, 2), round(sim_t, 2), phase, round(sim_alt, 1), round(sim_vel, 1),
                    round(temp_c, 2), round(pressure_hpa, 2), round(ax, 2), round(ay, 2), round(az, 2),
                    round(measured_g, 2), round(gx, 2), round(gy, 2), round(gz, 2),
                    "FAULT" if fault_active else "NOMINAL"
                ])
                file.flush()

                # Print concise live status line
                print(
                    f"[{phase:17s}] Alt:{sim_alt:8.1f}m | V:{sim_vel:6.1f}m/s | "
                    f"BMP280: {temp_c:4.1f}C / {pressure_hpa:6.1f}hPa | "
                    f"MPU6050: {measured_g:4.2f}G"
                )

                time.sleep(SAMPLE_INTERVAL_SEC)

        except KeyboardInterrupt:
            print("\nFlight computer manually stopped.")
        finally:
            if event_led:
                event_led.off()
            if countdown_led:
                countdown_led.off()
            print(f"Complete mission log saved to '{LOG_FILENAME}'.")


if __name__ == "__main__":
    main()
