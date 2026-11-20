#!/usr/bin/env python3
"""
MPU-6050 strapdown estimator for a Raspberry Pi 3B+ drop test.

The chip measures specific force (accelerometer) and angular rate
(gyroscope). This script converts those into:

  linear acceleration, velocity, and displacement in the world frame
  angular velocity and angular acceleration in the sensor frame
  angular displacement as roll, pitch, and yaw

World frame: X and Y horizontal, Z up. The origin is the position at the
end of calibration. Yaw is zero at calibration, and Z is aligned with
gravity rather than with the board, so a tilted mount still reports
vertical motion on Z.

Sign convention (Z up):
  at rest, specific force is about (0, 0, +9.81) m/s^2 and linear
  acceleration is about (0, 0, 0)
  in free fall, specific force falls to about (0, 0, 0) and linear
  acceleration is about (0, 0, -9.81) m/s^2
  released from rest, vertical velocity is about -g*t and vertical
  displacement is about -0.5*g*t^2

Angular velocity is not the time derivative of roll/pitch/yaw. Attitude
is integrated with a quaternion. Roll and pitch are corrected from the
accelerometer only while the specific-force magnitude is near 1 g.
During free fall the accelerometer no longer indicates "up", so attitude
coasts on the gyroscope. The MPU-6050 has no magnetometer, so yaw drifts.

Double integration of acceleration is usable for a short drop (about a
second or two) after a still calibration. It walks off over longer times.
Hold the package still, release it from rest, and do not throw it.

Hardware: GY-521 style MPU-6050 on I2C bus 1, address 0x68 (AD0 to GND).
  VCC -> pin 1 (3.3 V), GND -> pin 6, SDA -> pin 3, SCL -> pin 5
  sudo raspi-config -> Interface Options -> I2C
  sudo apt install python3-smbus i2c-tools
  pip3 install mpu6050-raspberrypi
  sudo i2cdetect -y 1    # expect address 68

Library: https://github.com/m-rtijn/mpu6050
"""

import argparse
import csv
import math
import time

# Chip full-scale settings. +/-8 g covers release and tumble with usable
# resolution. A hard landing still clips at 8 g. Gyro at +/-2000 deg/s
# covers a tumbling package.
ACCEL_RANGE_G = 8
GYRO_RANGE_DPS = 2000
# Digital low-pass. 98 Hz keeps the drop edge while cutting high-frequency noise.
DLPF_CODE_98HZ = 0x02

GRAVITY = 9.80665  # m/s^2, same constant the mpu6050 library uses

# Attitude correction gain, applied only while the accelerometer still
# indicates gravity. About a 0.5 s time constant.
KP_TILT = 2.0

# Still: specific force near 1 g and almost no rotation. Velocity is held
# at zero so bias does not integrate before release.
STATIONARY_ACCEL_TOL = 0.12 * GRAVITY
STATIONARY_GYRO_TOL = math.radians(4.0)

# Tilt correction is allowed during slow motion, and refused in free fall
# or during a hard push, when specific force is not gravity.
TILT_ACCEL_TOL = 0.25 * GRAVITY
TILT_GYRO_TOL = math.radians(30.0)

FREEFALL_SPECIFIC_FORCE = 0.30 * GRAVITY
ANG_ACC_WINDOW = 5
MAX_DT = 0.05  # s; ignore a stalled loop instead of taking a wild step


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
        return (1.0, 0.0, 0.0, 0.0)
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def quat_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_rotate(q, v):
    """Rotate a body-frame vector into the world frame."""
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def euler_to_quat(roll, pitch, yaw):
    """Intrinsic ZYX quaternion. q rotates body vectors into the world."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return quat_normalize((
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ))


def quat_to_euler(q):
    """Return roll, pitch, yaw in radians."""
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (roll, pitch, yaw)


def quat_from_accel(specific_force):
    """Level the world frame to gravity. Yaw is left at zero."""
    ax, ay, az = specific_force
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    return euler_to_quat(roll, pitch, 0.0)


def quat_integrate(q, omega, dt):
    """Exact step for a body-frame rate assumed constant over dt."""
    ox, oy, oz = omega
    mag = math.sqrt(ox * ox + oy * oy + oz * oz)
    half = 0.5 * mag * dt
    if mag < 1e-12:
        return quat_normalize(q)
    s = math.sin(half) / mag
    dq = (math.cos(half), ox * s, oy * s, oz * s)
    return quat_normalize(quat_mul(q, dq))


def _lin_acc(q, specific_force, gravity):
    """Kinematic acceleration in the world frame: a = R f + (0, 0, -g)."""
    f_world = quat_rotate(q, specific_force)
    return (f_world[0], f_world[1], f_world[2] - gravity)


class InertialEstimator:
    """Integrate bias-corrected specific force and angular rate."""

    def __init__(self, gravity=GRAVITY):
        self.gravity = gravity
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.lin_vel = (0.0, 0.0, 0.0)
        self.lin_pos = (0.0, 0.0, 0.0)
        self.t_prev = None
        self.omega_prev = (0.0, 0.0, 0.0)
        self._alpha_hist = []

    def align(self, specific_force):
        self.q = quat_from_accel(specific_force)
        self.lin_vel = (0.0, 0.0, 0.0)
        self.lin_pos = (0.0, 0.0, 0.0)
        self.t_prev = None
        self._alpha_hist = []

    def update(self, specific_force, omega, timestamp):
        """
        specific_force: body-frame accelerometer, m/s^2, bias removed
        omega: body-frame gyroscope, rad/s, bias removed
        timestamp: seconds, monotonic
        """
        if self.t_prev is None:
            self.t_prev = timestamp
            self.omega_prev = omega
            state = self._state(specific_force, omega, (0.0, 0.0, 0.0), False, True)
            return state

        dt = timestamp - self.t_prev
        if dt <= 0.0 or dt > MAX_DT:
            self.t_prev = timestamp
            self.omega_prev = omega
            return self._state(specific_force, omega, (0.0, 0.0, 0.0), False, False)

        f_mag = vnorm(specific_force)
        w_mag = vnorm(omega)
        freefall = f_mag < FREEFALL_SPECIFIC_FORCE
        stationary = (
            abs(f_mag - self.gravity) < STATIONARY_ACCEL_TOL
            and w_mag < STATIONARY_GYRO_TOL
        )
        tilt_ok = (
            abs(f_mag - self.gravity) < TILT_ACCEL_TOL
            and w_mag < TILT_GYRO_TOL
        )

        omega_used = omega
        if tilt_ok:
            # Mahony tilt correction. Specific force points up at rest.
            # error = measured up × estimated up, added to the body rate.
            up_body = quat_rotate(quat_conj(self.q), (0.0, 0.0, 1.0))
            error = vcross(vnormalize(specific_force), up_body)
            omega_used = vadd(omega, vscale(error, KP_TILT))

        self.q = quat_integrate(self.q, omega_used, dt)

        # a = f + g, with g = (0, 0, -gravity) in the world frame.
        lin_acc = _lin_acc(self.q, specific_force, self.gravity)

        if stationary:
            self.lin_vel = (0.0, 0.0, 0.0)
        else:
            v_new = vadd(self.lin_vel, vscale(lin_acc, dt))
            self.lin_pos = vadd(
                self.lin_pos,
                vscale(vadd(self.lin_vel, v_new), 0.5 * dt),
            )
            self.lin_vel = v_new

        ang_acc = vscale(vsub(omega, self.omega_prev), 1.0 / dt)
        self._alpha_hist.append(ang_acc)
        if len(self._alpha_hist) > ANG_ACC_WINDOW:
            self._alpha_hist.pop(0)
        ang_acc_avg = vmean(self._alpha_hist)

        self.t_prev = timestamp
        self.omega_prev = omega
        return self._state(specific_force, omega, ang_acc_avg, freefall, stationary)

    def _state(self, specific_force, omega, ang_acc, freefall, stationary):
        return {
            "lin_acc": _lin_acc(self.q, specific_force, self.gravity),
            "lin_vel": self.lin_vel,
            "lin_pos": self.lin_pos,
            "ang_acc": ang_acc,
            "ang_vel": omega,
            "ang_disp": quat_to_euler(self.q),
            "specific_force": specific_force,
            "freefall": freefall,
            "stationary": stationary,
        }


CSV_HEADER = [
    "t_s",
    "ax_mps2", "ay_mps2", "az_mps2",
    "vx_mps", "vy_mps", "vz_mps",
    "px_m", "py_m", "pz_m",
    "alphax_radps2", "alphay_radps2", "alphaz_radps2",
    "wx_radps", "wy_radps", "wz_radps",
    "roll_rad", "pitch_rad", "yaw_rad",
    "fx_mps2", "fy_mps2", "fz_mps2",
    "freefall", "stationary",
]


def state_row(t, state):
    a, v, p = state["lin_acc"], state["lin_vel"], state["lin_pos"]
    al, w, ang = state["ang_acc"], state["ang_vel"], state["ang_disp"]
    f = state["specific_force"]
    return [
        t,
        a[0], a[1], a[2],
        v[0], v[1], v[2],
        p[0], p[1], p[2],
        al[0], al[1], al[2],
        w[0], w[1], w[2],
        ang[0], ang[1], ang[2],
        f[0], f[1], f[2],
        int(state["freefall"]), int(state["stationary"]),
    ]


def format_state(t, state):
    a, v, p = state["lin_acc"], state["lin_vel"], state["lin_pos"]
    al, w, ang = state["ang_acc"], state["ang_vel"], state["ang_disp"]
    if state["freefall"]:
        phase = "FREE FALL"
    elif state["stationary"]:
        phase = "STILL"
    else:
        phase = "MOVING"
    return (
        "t={:7.3f} s  {}\n"
        "  lin acc   m/s^2    {:+9.3f} {:+9.3f} {:+9.3f}\n"
        "  lin vel   m/s      {:+9.3f} {:+9.3f} {:+9.3f}\n"
        "  lin disp  m        {:+9.3f} {:+9.3f} {:+9.3f}\n"
        "  ang acc   rad/s^2  {:+9.3f} {:+9.3f} {:+9.3f}\n"
        "  ang vel   rad/s    {:+9.3f} {:+9.3f} {:+9.3f}\n"
        "  ang disp  deg      {:+9.2f} {:+9.2f} {:+9.2f}"
    ).format(
        t, phase,
        a[0], a[1], a[2],
        v[0], v[1], v[2],
        p[0], p[1], p[2],
        al[0], al[1], al[2],
        w[0], w[1], w[2],
        math.degrees(ang[0]), math.degrees(ang[1]), math.degrees(ang[2]),
    )


def read_sensor(sensor):
    """Return specific force (m/s^2) and angular rate (rad/s)."""
    accel = sensor.get_accel_data(g=False)
    gyro = sensor.get_gyro_data()
    specific_force = (accel["x"], accel["y"], accel["z"])
    omega = (
        math.radians(gyro["x"]),
        math.radians(gyro["y"]),
        math.radians(gyro["z"]),
    )
    return specific_force, omega


def configure_sensor(sensor):
    accel_ranges = {
        2: sensor.ACCEL_RANGE_2G,
        4: sensor.ACCEL_RANGE_4G,
        8: sensor.ACCEL_RANGE_8G,
        16: sensor.ACCEL_RANGE_16G,
    }
    gyro_ranges = {
        250: sensor.GYRO_RANGE_250DEG,
        500: sensor.GYRO_RANGE_500DEG,
        1000: sensor.GYRO_RANGE_1000DEG,
        2000: sensor.GYRO_RANGE_2000DEG,
    }
    sensor.set_accel_range(accel_ranges[ACCEL_RANGE_G])
    sensor.set_gyro_range(gyro_ranges[GYRO_RANGE_DPS])
    if hasattr(sensor, "set_filter_range"):
        sensor.set_filter_range(DLPF_CODE_98HZ)


def calibrate(sensor, seconds):
    """Still-package gyro bias and accelerometer offset, gravity kept."""
    print("Hold the package still for {:.1f} s...".format(seconds))
    time.sleep(0.3)
    forces = []
    rates = []
    t_end = time.perf_counter() + seconds
    while time.perf_counter() < t_end:
        f, w = read_sensor(sensor)
        forces.append(f)
        rates.append(w)
    if len(forces) < 10:
        raise RuntimeError("Calibration collected too few samples. Check I2C wiring.")
    mean_f = vmean(forces)
    mean_w = vmean(rates)
    f_mag = vnorm(mean_f)
    if f_mag < 0.5 * GRAVITY:
        raise RuntimeError(
            "Accelerometer magnitude is {:.2f} m/s^2 during calibration, "
            "not near g. The chip is not sitting still, or it is not an MPU-6050.".format(f_mag)
        )
    # Remove the small offset but keep a vector of length g along the measured up direction.
    accel_bias = vsub(mean_f, vscale(vnormalize(mean_f), GRAVITY))
    print("Samples: {}".format(len(forces)))
    print("Gyro bias (rad/s):  {:+.5f} {:+.5f} {:+.5f}".format(*mean_w))
    print("Accel bias (m/s^2): {:+.4f} {:+.4f} {:+.4f}".format(*accel_bias))
    return accel_bias, mean_w, vsub(mean_f, accel_bias)


def open_sensor(address, bus):
    try:
        from mpu6050 import mpu6050
    except ImportError as exc:
        raise SystemExit(
            "Could not import mpu6050. On the Pi run:\n"
            "  sudo apt install python3-smbus i2c-tools\n"
            "  pip3 install mpu6050-raspberrypi"
        ) from exc
    sensor = mpu6050(address, bus=bus)
    configure_sensor(sensor)
    return sensor


def run_logger(sensor, outfile, calibrate_s, duration):
    accel_bias, gyro_bias, f_align = calibrate(sensor, calibrate_s)
    est = InertialEstimator()
    est.align(f_align)

    print("Logging to {}".format(outfile))
    print("Linear columns are world X Y Z with Z up.")
    print("Angular vel/acc are sensor X Y Z. Angular disp is roll pitch yaw.")
    print("Release from rest. Ctrl+C stops logging.\n")

    t0 = time.perf_counter()
    next_print = t0
    with open(outfile, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(CSV_HEADER)
        try:
            while True:
                now = time.perf_counter()
                if duration > 0.0 and (now - t0) >= duration:
                    break
                f_raw, w_raw = read_sensor(sensor)
                f = vsub(f_raw, accel_bias)
                w = vsub(w_raw, gyro_bias)
                state = est.update(f, w, now)
                writer.writerow(state_row(now - t0, state))
                handle.flush()
                if now >= next_print:
                    print(format_state(now - t0, state))
                    print("")
                    next_print = now + 0.25
        except KeyboardInterrupt:
            print("\nStopped.")
    print("Saved {}".format(outfile))


def _check(name, ok, detail):
    if not ok:
        raise AssertionError("{}: {}".format(name, detail))
    print("  pass  {}  ({})".format(name, detail))


def run_self_test():
    """Numeric checks that do not need the sensor."""
    print("Self-test")
    g = GRAVITY
    dt = 0.01

    # Euler angles survive a round trip, and the accelerometer tilt
    # formula recovers roll and pitch from a pure gravity measurement.
    for roll, pitch, yaw in (
        (0.0, 0.0, 0.0),
        (math.radians(20.0), math.radians(-15.0), math.radians(40.0)),
        (math.radians(-35.0), math.radians(25.0), math.radians(-10.0)),
        (math.radians(170.0), math.radians(5.0), math.radians(90.0)),
    ):
        q = euler_to_quat(roll, pitch, yaw)
        got = quat_to_euler(q)
        err = max(abs(math.atan2(math.sin(got[i] - src), math.cos(got[i] - src)))
                  for i, src in enumerate((roll, pitch, yaw)))
        _check("euler roundtrip", err < 1e-9, "err={:.3e} rad".format(err))

        f_body = quat_rotate(quat_conj(q), (0.0, 0.0, g))
        q_acc = quat_from_accel(f_body)
        got_rp = quat_to_euler(q_acc)
        rp_err = max(
            abs(math.atan2(math.sin(got_rp[0] - roll), math.cos(got_rp[0] - roll))),
            abs(math.atan2(math.sin(got_rp[1] - pitch), math.cos(got_rp[1] - pitch))),
        )
        _check("accel leveling", rp_err < 1e-8, "roll/pitch err={:.3e} rad".format(rp_err))
        a_world = _lin_acc(q, f_body, g)
        _check(
            "still linear acc",
            vnorm(a_world) < 1e-8,
            "accel={:.3e} m/s^2".format(vnorm(a_world)),
        )

    # Tilted, still package: correction pulls attitude onto gravity and
    # linear acceleration stays near zero.
    true_q = euler_to_quat(math.radians(12.0), math.radians(-8.0), 0.0)
    f_still = quat_rotate(quat_conj(true_q), (0.0, 0.0, g))
    est = InertialEstimator()
    t = 0.0
    state = None
    for _ in range(300):
        t += dt
        state = est.update(f_still, (0.0, 0.0, 0.0), t)
    roll, pitch, _yaw = state["ang_disp"]
    _check("tilt converges", abs(roll - math.radians(12.0)) < math.radians(0.2)
           and abs(pitch - math.radians(-8.0)) < math.radians(0.2),
           "roll={:.2f} deg pitch={:.2f} deg".format(math.degrees(roll), math.degrees(pitch)))
    _check("still velocity held", vnorm(state["lin_vel"]) < 1e-9
           and vnorm(state["lin_pos"]) < 1e-9, "ZUPT")

    # Free fall after a still alignment: a_z = -g, v_z = -g*t, p_z = -0.5*g*t^2.
    est = InertialEstimator()
    est.align((0.0, 0.0, g))
    t = 0.0
    fall_t = 1.0
    steps = int(fall_t / dt)
    state = est.update((0.0, 0.0, g), (0.0, 0.0, 0.0), t)
    for _ in range(steps):
        t += dt
        state = est.update((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), t)
    _check("freefall acc", abs(state["lin_acc"][2] + g) < 1e-6, "az={:.5f}".format(state["lin_acc"][2]))
    _check("freefall vel", abs(state["lin_vel"][2] + g * fall_t) < 1e-6,
           "vz={:.5f}".format(state["lin_vel"][2]))
    _check("freefall pos", abs(state["lin_pos"][2] + 0.5 * g * fall_t * fall_t) < 1e-6,
           "pz={:.5f}".format(state["lin_pos"][2]))
    _check("freefall flag", state["freefall"], "specific force is 0")

    # Constant body yaw rate of +90 deg/s for 1 s, while remaining level.
    est = InertialEstimator()
    est.align((0.0, 0.0, g))
    t = 0.0
    yaw_rate = math.radians(90.0)
    state = est.update((0.0, 0.0, g), (0.0, 0.0, 0.0), t)
    for _ in range(steps):
        t += dt
        state = est.update((0.0, 0.0, g), (0.0, 0.0, yaw_rate), t)
    _yaw = state["ang_disp"][2]
    yaw_err = abs(math.atan2(math.sin(_yaw - math.radians(90.0)), math.cos(_yaw - math.radians(90.0))))
    _check("yaw integration", yaw_err < math.radians(0.05),
           "yaw={:.3f} deg".format(math.degrees(_yaw)))
    _check("yaw rate", abs(state["ang_vel"][2] - yaw_rate) < 1e-12, "wz")

    # Angular acceleration is the derivative of angular velocity.
    est = InertialEstimator()
    est.align((0.0, 0.0, g))
    t = 0.0
    est.update((0.0, 0.0, g), (0.0, 0.0, 0.0), t)
    alpha = 2.0  # rad/s^2 about X
    last = None
    for i in range(1, 40):
        t += dt
        w = alpha * i * dt
        last = est.update((0.0, 0.0, g), (w, 0.0, 0.0), t)
    _check("angular acc", abs(last["ang_acc"][0] - alpha) < 1e-6,
           "alphax={:.5f}".format(last["ang_acc"][0]))
    print("Self-test passed.")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Log MPU-6050 linear and angular displacement, velocity, and acceleration."
    )
    parser.add_argument("--address", type=lambda s: int(s, 0), default=0x68,
                        help="I2C address (default 0x68)")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus (1 on Pi 3B+)")
    parser.add_argument("--calibrate", type=float, default=2.0,
                        help="still-calibration time in seconds")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="log this many seconds after calibration; 0 logs until Ctrl+C")
    parser.add_argument("--outfile", default="drop_log.csv", help="CSV output path")
    parser.add_argument("--self-test", action="store_true",
                        help="check the kinematics without the sensor, then exit")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.self_test:
        run_self_test()
        return
    sensor = open_sensor(args.address, args.bus)
    run_logger(sensor, args.outfile, args.calibrate, args.duration)


if __name__ == "__main__":
    main()
