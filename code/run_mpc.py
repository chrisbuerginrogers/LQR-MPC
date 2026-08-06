"""
Lesson 5 on hardware: MPC that slows down for sharp turns.

    python3 code/run_mpc.py [seconds] [--no-plot]

Same steering law as run_lqr.py. The difference is that the forward speed is
no longer a constant you picked -- it is re-decided every control period by
solving a small constrained optimization over a short preview of the track.

With no track map on board, the preview here is the curvature the controller
is currently having to hold, assumed to persist for the length of the horizon.
That is a crude predictor, and it is exactly the thing to improve in the
capstone (see docs/02-mpc.md, "Where does the preview come from?").

Every run is logged to code/runs/ and, unless --no-plot is given, a live plot
window opens showing e(t) and v(t) against your last couple of MPC runs and
the most recent run of every other controller (see code/live_plot.py).
"""

import sys

from robot import LineFollower, curvature_estimate, load_calibration
from sim import V_REF, lqr_gains_discrete, mpc_choose_speed
from telemetry import RunLogger, spawn_live_plot

CANDIDATES = (40.0, 60.0, 80.0, 100.0, 120.0, 150.0, 180.0, 210.0)
HORIZON = 15
NAME = "mpc"


class ConstantCurvature:
    """Preview model: 'whatever I am turning now, I will keep turning'."""

    def __init__(self, kappa):
        self._kappa = kappa

    def kappa(self, _s):
        return self._kappa


def main():
    no_plot = "--no-plot" in sys.argv
    if no_plot:
        sys.argv.remove("--no-plot")

    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    cal = load_calibration()
    dt, e_max = cal["dt"], cal["e_max_mm"]

    print(f"MPC: {len(CANDIDATES)} candidate speeds, horizon {HORIZON} steps "
          f"= {HORIZON * dt:.2f} s")
    print(f"hard-ish constraint: |e| <= {e_max:.1f} mm over the whole horizon\n")

    logger = RunLogger(NAME, meta={"candidates": CANDIDATES, "horizon": HORIZON,
                                    "dt": dt})
    spawn_live_plot(logger.path, NAME, e_max, disable=no_plot)
    print(f"Logging to {logger.path}\n")

    with LineFollower(cal) as bot:
        v = V_REF
        kappa_hat = 0.0
        peak = 0.0
        blind = 0
        for k, t in bot.loop(duration):
            e, theta, saturated = bot.read()

            preview = ConstantCurvature(kappa_hat)
            v, table = mpc_choose_speed(e, theta, 0.0, preview, CANDIDATES,
                                        HORIZON, dt, e_max)

            k_e, k_th = lqr_gains_discrete(v, dt)
            omega = -k_e * e - k_th * theta
            left, right = bot.drive(v, omega)
            logger.log(t, e, theta, v, omega, saturated, left, right)

            # Low-pass the curvature we are actually holding; that becomes
            # the next period's preview.
            kappa_hat = 0.8 * kappa_hat + 0.2 * curvature_estimate(omega, v)
            blind += saturated
            peak = max(peak, abs(e))

            if k % 10 == 0:
                radius = 1e9 if abs(kappa_hat) < 1e-6 else 1.0 / abs(kappa_hat)
                rejected = sum(1 for _, pk, _ in table if pk > e_max)
                print(f"  t={t:5.2f}s  e={e:+6.2f}mm  R~{min(radius, 9999):6.0f}mm  "
                      f"v={v:5.0f}mm/s  omega={omega:+6.2f}rad/s  "
                      f"{rejected}/{len(CANDIDATES)} speeds infeasible"
                      f"{'  BLIND' if saturated else ''}")
        bot.stop()
    logger.close()

    total = int(duration / dt)
    print(f"\npeak |e| = {peak:.2f} mm, blind on "
          f"{100.0 * blind / max(total, 1):.1f}% of samples")


if __name__ == "__main__":
    main()
