"""
Lesson 2 on hardware: proportional (and PD) line following.

    python3 code/run_pd.py [seconds] [speed_mms] [kp] [kd] [--no-plot]

Proportional control on the lateral error, plus an optional derivative term
on the heading estimate. Push kp up and it oscillates; push it down and it
corners badly. Add kd and it gets much better -- and now there are two knobs
with no principle for setting them. That tuning wall is the point of this
lesson; Session 3 (LQR) is what replaces the guessing.

kp and kd are in the same units as k_e and k_theta in run_lqr.py
(rad/s per mm, and 1/s), so a (kp, kd) pair you like can be compared directly
against the gains LQR comes up with later.

Every run is logged to code/runs/ and, unless --no-plot is given, a live plot
window opens showing e(t) and v(t) against your last couple of P/PD runs and
the most recent run of every other controller (see code/live_plot.py).
"""

import sys

from robot import LineFollower, load_calibration
from telemetry import RunLogger, spawn_live_plot

NAME = "pd"


def main():
    no_plot = "--no-plot" in sys.argv
    if no_plot:
        sys.argv.remove("--no-plot")

    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    v = float(sys.argv[2]) if len(sys.argv) > 2 else 150.0
    kp = float(sys.argv[3]) if len(sys.argv) > 3 else 0.15
    kd = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0

    cal = load_calibration()
    label = "PD" if kd else "P"
    print(f"{label} control: v = {v:.0f} mm/s   kp = {kp:.3f}   kd = {kd:.3f}   "
          f"dt = {cal['dt'] * 1000:.0f} ms")
    print("Control law: omega = -kp*e - kd*theta\n")

    logger = RunLogger(NAME, meta={"v": v, "kp": kp, "kd": kd, "dt": cal["dt"]})
    spawn_live_plot(logger.path, NAME, cal["e_max_mm"], disable=no_plot)
    print(f"Logging to {logger.path}\n")

    with LineFollower(cal) as bot:
        blind = 0
        peak = 0.0
        for k, t in bot.loop(duration):
            e, theta, saturated = bot.read()
            omega = -kp * e - kd * theta
            left, right = bot.drive(v, omega)
            logger.log(t, e, theta, v, omega, saturated, left, right)
            blind += saturated
            peak = max(peak, abs(e))
            if k % 10 == 0:
                print(f"  t={t:5.2f}s  e={e:+6.2f}mm  theta={theta:+6.3f}rad  "
                      f"omega={omega:+6.2f}rad/s  motors={left:+4d}/{right:+4d}"
                      f"{'  BLIND' if saturated else ''}")
        bot.stop()
    logger.close()

    total = int(duration / cal["dt"])
    print(f"\npeak |e| = {peak:.2f} mm, blind on "
          f"{100.0 * blind / max(total, 1):.1f}% of samples")
    print("Record this (kp, kd, behavior) triple, then try another -- there is "
          "no formula yet for which pair is best.")


if __name__ == "__main__":
    main()
