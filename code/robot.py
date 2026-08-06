"""
Hardware layer for the one-sensor line follower.

Wraps the LEGO Education Double Motor + Color Sensor (pip install legoeducation)
behind the two functions a controller actually wants:

    read()          -> (e_mm, theta_rad, saturated)
    drive(v, omega) -> apply forward speed [mm/s] and yaw rate [rad/s]

Everything above this file is pure control theory and runs unchanged in
code/sim.py.
"""

import json
import math
import os
import time

import legoeducation as le

CAL_PATH = os.path.join(os.path.dirname(__file__), "calibration.json")

DEFAULTS = {
    "reflect_black": 12.0,   # sensor reading fully over the tape
    "reflect_white": 85.0,   # sensor reading fully over the background
    "e_max_mm": 6.0,         # half-width of the linear region (see calibrate.py)
    "track_width_mm": 96.0,  # distance between wheel contact patches
    "wheel_max_mms": 400.0,  # wheel speed at motor_speed = 100%
    "dt": 0.05,              # control period == notification delay
    "follow_left_edge": True,
}


def load_calibration():
    cal = dict(DEFAULTS)
    if os.path.exists(CAL_PATH):
        with open(CAL_PATH) as f:
            cal.update(json.load(f))
    return cal


class LineFollower:
    """Differential-drive robot with a single downward-facing light sensor."""

    def __init__(self, cal=None):
        self.cal = cal or load_calibration()
        self.dt = self.cal["dt"]
        self.motors = le.DoubleMotor()
        self.sensor = le.ColorSensor()
        self._theta = 0.0          # heading-error estimate [rad]
        self._e_prev = None
        self._v_prev = 0.0

    # -- connection ---------------------------------------------------------
    def connect(self):
        delay_ms = int(self.dt * 1000)
        self.motors.connect(device_notification_delay=delay_ms)
        self.sensor.connect(device_notification_delay=delay_ms)
        # Let the controller, not the firmware, shape the ramps.
        self.motors.movement_set_acceleration(100, 100)
        self.motors.movement_set_end_state(le.MOTOR_END_STATE_BRAKE)
        self.motors.imu_reset_yaw_axis(0)
        return self

    def close(self):
        try:
            self.motors.movement_stop()
        finally:
            self.motors.disconnect()
            self.sensor.disconnect()

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        self.close()

    # -- sensing ------------------------------------------------------------
    def normalized(self):
        """Map raw reflection to s in [0, 1]: 0 = fully on tape, 1 = fully off."""
        raw = float(self.sensor.sensor.reflection)
        black, white = self.cal["reflect_black"], self.cal["reflect_white"]
        return min(1.0, max(0.0, (raw - black) / (white - black)))

    def read(self):
        """Return (e_mm, theta_rad, saturated).

        e is the lateral offset of the sensor from the tape edge, positive
        meaning "drifted off the line toward the background". Only one sensor,
        so e is only meaningful inside +-e_max; outside that the reading
        saturates and carries sign but no magnitude.
        """
        e_max = self.cal["e_max_mm"]
        s = self.normalized()
        e = 2.0 * e_max * (s - 0.5)
        if not self.cal["follow_left_edge"]:
            e = -e
        saturated = s <= 0.02 or s >= 0.98
        e = max(-e_max, min(e_max, e))

        # Complementary estimate of theta: predict with the yaw rate we asked
        # for, correct with the observed drift rate e_dot = v * theta.
        # Freeze the correction while the sensor is saturated -- a pinned
        # reading has zero slope and would lie about theta being zero.
        if self._e_prev is not None and not saturated and self._v_prev > 1.0:
            theta_meas = (e - self._e_prev) / (self.dt * self._v_prev)
            self._theta = 0.7 * self._theta + 0.3 * theta_meas
        self._e_prev = e
        return e, self._theta, saturated

    def gyro_yaw_rate(self):
        """Raw IMU yaw rate from the Double Motor, for the observer lab.

        Units are firmware-defined; run code/calibrate.py, spin the robot a
        known number of turns, and divide to get your own scale factor.
        """
        return float(self.motors.imu_device.gyroscopeZ)

    # -- actuation ----------------------------------------------------------
    def drive(self, v_mms, omega_rads):
        """Differential-drive inverse kinematics, then percent, then BLE."""
        half = self.cal["track_width_mm"] / 2.0
        v_left = v_mms - omega_rads * half
        v_right = v_mms + omega_rads * half
        scale = 100.0 / self.cal["wheel_max_mms"]
        left = int(round(max(-100.0, min(100.0, v_left * scale))))
        right = int(round(max(-100.0, min(100.0, v_right * scale))))
        self.motors.movement_move_tank(left, right, blocking=False)
        # Remember the speed we actually commanded; the theta estimator uses it.
        self._theta += self.dt * omega_rads
        self._v_prev = v_mms
        return left, right

    def stop(self):
        self.motors.movement_stop()

    # -- loop timing --------------------------------------------------------
    def loop(self, duration_s):
        """Yield once per control period, holding the period as well as BLE allows."""
        t0 = time.monotonic()
        k = 0
        while True:
            now = time.monotonic()
            if now - t0 > duration_s:
                return
            yield k, now - t0
            k += 1
            slack = t0 + k * self.dt - time.monotonic()
            if slack > 0:
                time.sleep(slack)


def curvature_estimate(omega, v, floor=20.0):
    """kappa ~= omega / v: how tight a turn the controller is currently holding.

    This is the cheap stand-in for track preview when you have no map --
    see docs/02-mpc.md, "Where does the preview come from?".
    """
    return omega / max(v, floor)


def deg(rad):
    return rad * 180.0 / math.pi
