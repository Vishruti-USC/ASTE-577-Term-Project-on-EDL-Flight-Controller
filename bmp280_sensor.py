#!/usr/bin/env python3
"""
ASTE 577 - BMP280 Atmospheric Pressure & Temperature Sensor Script
Hardware Configuration:
  - Raspberry Pi 3 B+
  - HiLetgo BMP280-3.3 Atmospheric Pressure & Temperature Sensor (I2C: 0x76 or 0x77)

Wiring (Raspberry Pi 3 B+ Physical Pins):
  - BMP280 VCC -> Pin 1 (3.3V)
  - BMP280 GND -> Pin 6 (GND)
  - BMP280 SDA -> Pin 3 (GPIO 2)
  - BMP280 SCL -> Pin 5 (GPIO 3)

Setup Instructions:
  1. Enable I2C: Run `sudo raspi-config` -> Interface Options -> I2C -> Enable -> Reboot
  2. Verify sensor: Run `sudo i2cdetect -y 1` (confirm 0x76 or 0x77 appears)
  3. Install library:
       pip3 install adafruit-circuitpython-bmp280
     (Add `--break-system-packages` if prompted on newer Raspberry Pi OS versions)
  4. Run script:
       python3 bmp280_sensor.py
"""

import time
import csv
import board
import adafruit_bmp280

# ---------------------------------------------------------------------------
# Sensor Configuration
# ---------------------------------------------------------------------------
BMP280_I2C_ADDRESS = 0x76        # Change to 0x77 if i2cdetect shows 77
LOCAL_SEA_LEVEL_HPA = 1013.25    # Update with local barometric pressure for accurate altitude
SAMPLE_INTERVAL_SEC = 0.5        # 0.5s = 2 Hz sampling rate
LOG_FILENAME = "bmp280_telemetry.csv"

# ---------------------------------------------------------------------------
# Hardware Initialization
# ---------------------------------------------------------------------------
def init_bmp280():
    # Initialize I2C bus on Pi 3 B+ (Pin 3 = SDA, Pin 5 = SCL)
    i2c = board.I2C()

    # Initialize HiLetgo BMP280-3.3
    bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=BMP280_I2C_ADDRESS)
    bmp280.sea_level_pressure = LOCAL_SEA_LEVEL_HPA

    return bmp280


# ---------------------------------------------------------------------------
# Main Telemetry & Logging Loop
# ---------------------------------------------------------------------------
def main():
    bmp280 = init_bmp280()
    print(f"BMP280 initialized successfully. Logging data to '{LOG_FILENAME}'...")
    print("Press Ctrl+C to stop and save data.\n")

    with open(LOG_FILENAME, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Time_s", "Pressure_hPa", "Temp_C", "Est_Alt_m"])

        start_time = time.time()

        try:
            while True:
                t = time.time() - start_time

                # Read BMP280 atmospheric data
                temp_c = bmp280.temperature
                pressure_hpa = bmp280.pressure
                altitude_m = bmp280.altitude

                # Write packet to CSV
                writer.writerow([
                    round(t, 3),
                    round(pressure_hpa, 2),
                    round(temp_c, 2),
                    round(altitude_m, 2)
                ])
                file.flush()

                # Print live telemetry to terminal
                print(
                    f"[{t:6.2f}s] Pressure: {pressure_hpa:7.2f} hPa | "
                    f"Temp: {temp_c:5.2f} C | Est. Alt: {altitude_m:6.2f} m"
                )

                time.sleep(SAMPLE_INTERVAL_SEC)

        except KeyboardInterrupt:
            print(f"\nBMP280 logging stopped. Data saved to '{LOG_FILENAME}'.")


if __name__ == "__main__":
    main()
