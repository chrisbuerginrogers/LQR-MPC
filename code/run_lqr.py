"""
Lesson 3 on hardware: fixed-gain LQR line following.

    python3 code/run_lqr.py [seconds] [speed_mms] [--no-plot]

The gains are computed once, before the robot moves, from the model and the
cost weights. Nothing is tuned by hand.

Every run is logged to code/runs/ and, unless --no-plot is given, a live plot
window opens showing e(t) and v(t) against your last couple of LQR runs and
the most recent run of every other controller (see code/live_plot.py).
"""

import sys

from robot import LineFollower, load_calibration
from sim import Q_E, Q_TH, R_W, lqr_gains_analytic, lqr_gains_discrete
from telemetry import RunLogger, spawn_live_plot

NAME = "lqr"


def main():
    no_plot = "--no-plot" in sys.argv
    if no_plot:
        sys.argv.remove("--no-plot")

    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    v = float(sys.argv[2]) if len(sys.argv) > 2 else 150.0

    cal = load_calibration()
    k_e, k_th = lqr_gains_discrete(v, cal["dt"], Q_E, Q_TH, R_W)
    k_e_c, k_th_c = lqr_gains_analytic(v, Q_E, Q_TH, R_W)

    print(f"Q = diag({Q_E}, {Q_TH})   R = {R_W}   v = {v:.0f} mm/s   "
          f"dt = {cal['dt'] * 1000:.0f} ms")
    print(f"  continuous-time gains : k_e = {k_e_c:.4f}  k_theta = {k_th_c:.4f}")
    print(f"  discrete-time gains   : k_e = {k_e:.4f}  k_theta = {k_th:.4f}  <- used")
    print(f"  predicted offset on an R = 200 mm curve: "
          f"{-(1 / 200) * v / k_e:+.1f} mm "
          f"(window is +-{cal['e_max_mm']:.1f} mm)")

    logger = RunLogger(NAME, meta={"v": v, "k_e": k_e, "k_th": k_th,
                                    "dt": cal["dt"]})
    spawn_live_plot(logger.path, NAME, cal["e_max_mm"], disable=no_plot)
    print(f"Logging to {logger.path}\n")

    with LineFollower(cal) as bot:
        blind = 0
        peak = 0.0
        for k, t in bot.loop(duration):
            e, theta, saturated = bot.read()
            omega = -k_e * e - k_th * theta
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


if __name__ == "__main__":
    main()
