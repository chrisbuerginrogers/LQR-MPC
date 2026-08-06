"""
Lesson 1 on hardware: bang-bang line following.

    python3 code/run_bangbang.py [seconds] [speed_mms] [omega_rads] [--no-plot]

The simplest control law there is: look at the *sign* of the error and turn
one way or the other at a fixed rate. It works, and it oscillates -- the
sensor reports a number between reflect_black and reflect_white and this
throws away everything but one bit of it.

Every run is logged to code/runs/ and, unless --no-plot is given, a live plot
window opens showing e(t) and v(t) against your last couple of bang-bang runs
and the most recent run of every other controller (see code/live_plot.py).
"""

import sys

from robot import LineFollower, load_calibration
from telemetry import RunLogger, spawn_live_plot

NAME = "bangbang"


def main():
    no_plot = "--no-plot" in sys.argv
    if no_plot:
        sys.argv.remove("--no-plot")

    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    v = float(sys.argv[2]) if len(sys.argv) > 2 else 100.0
    omega_mag = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0

    cal = load_calibration()
    print(f"Bang-bang: v = {v:.0f} mm/s   omega = +-{omega_mag:.2f} rad/s   "
          f"dt = {cal['dt'] * 1000:.0f} ms")
    print("Control law: omega = -omega_mag * sign(e)  (one bit of the error, "
          "not its size)\n")

    logger = RunLogger(NAME, meta={"v": v, "omega_mag": omega_mag,
                                    "dt": cal["dt"]})
    spawn_live_plot(logger.path, NAME, cal["e_max_mm"], disable=no_plot)
    print(f"Logging to {logger.path}\n")

    with LineFollower(cal) as bot:
        blind = 0
        peak = 0.0
        crossings = 0
        sign_prev = None
        for k, t in bot.loop(duration):
            e, theta, saturated = bot.read()
            sign = 1.0 if e > 0 else -1.0
            omega = -omega_mag * sign
            left, right = bot.drive(v, omega)
            logger.log(t, e, theta, v, omega, saturated, left, right)
            if sign_prev is not None and sign != sign_prev:
                crossings += 1
            sign_prev = sign
            blind += saturated
            peak = max(peak, abs(e))
            if k % 10 == 0:
                print(f"  t={t:5.2f}s  e={e:+6.2f}mm  omega={omega:+6.2f}rad/s  "
                      f"motors={left:+4d}/{right:+4d}"
                      f"{'  BLIND' if saturated else ''}")
        bot.stop()
    logger.close()

    total = int(duration / cal["dt"])
    wobble_hz = crossings / (2.0 * duration) if duration > 0 else 0.0
    print(f"\npeak |e| = {peak:.2f} mm, blind on "
          f"{100.0 * blind / max(total, 1):.1f}% of samples, "
          f"wobble ~ {wobble_hz:.2f} Hz ({crossings} sign changes in {duration:.1f}s)")


if __name__ == "__main__":
    main()
