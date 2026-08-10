"""
Real-hardware control loops for docs/interactive.html's "Run on real robot"
buttons, invoked by code/calibrate_server.py.

Each function mirrors the matching CLI script (run_bangbang.py, run_pd.py,
run_lqr.py, run_mpc.py) exactly, but returns the captured (t, e, theta, v,
omega, saturated) log as plain dicts instead of printing it, so the web
server can hand it back as JSON. The CLI scripts remain the canonical,
independently-tested entry points; this module exists only to avoid
re-typing their control laws.

MPC has no track map here -- a real robot has no odometry position estimate
wired up in robot.py, so like run_mpc.py, the preview is the reactive
"whatever curvature I'm holding now" estimate, not a true look-ahead. That's
an honest limitation, not a bug: see docs/02-mpc.md, "Where does the preview
come from?".
"""

from robot import LineFollower, curvature_estimate
from sim import coerce, lqr_gains_discrete, step


def run_bangbang(cal, duration, v=100.0, omega_mag=2.0):
    log = []
    with LineFollower(cal) as bot:
        for k, t in bot.loop(duration):
            e, theta, saturated = bot.read()
            omega = -omega_mag * (1.0 if e > 0 else -1.0)
            bot.drive(v, omega)
            log.append({"t": t, "e": e, "theta": theta, "v": v, "omega": omega,
                         "saturated": bool(saturated)})
        bot.stop()
    return log


def run_pd(cal, duration, v=150.0, kp=0.15, kd=0.0):
    log = []
    with LineFollower(cal) as bot:
        for k, t in bot.loop(duration):
            e, theta, saturated = bot.read()
            omega = -kp * e - kd * theta
            bot.drive(v, omega)
            log.append({"t": t, "e": e, "theta": theta, "v": v, "omega": omega,
                         "saturated": bool(saturated)})
        bot.stop()
    return log


def run_lqr(cal, duration, v=150.0, qe=1.0, qth=0.0, r=25.0):
    dt = cal["dt"]
    k_e, k_th = lqr_gains_discrete(v, dt, qe, qth, r)
    log = []
    with LineFollower(cal) as bot:
        for k, t in bot.loop(duration):
            e, theta, saturated = bot.read()
            omega = -k_e * e - k_th * theta
            bot.drive(v, omega)
            log.append({"t": t, "e": e, "theta": theta, "v": v, "omega": omega,
                         "saturated": bool(saturated)})
        bot.stop()
    return log


def _mpc_choose_speed(e, theta, kappa_hat, candidates, horizon, dt, e_max,
                       v_ref, qe, qth, r, qv, pen=1e4):
    """Same enumeration as sim.py's mpc_choose_speed, but with qe/qth/r
    explicitly threaded into the steering gains inside the rollout (sim.py's
    version always uses its module-level Q_E/Q_TH/R_W for that), so a real
    run matches the interactive page's simulated MPC tab exactly."""
    best_cost, best_v = float("inf"), candidates[0]
    for v in candidates:
        k_e, k_th = lqr_gains_discrete(v, dt, qe, qth, r)
        ep, thp, cost = e, theta, 0.0
        for _ in range(horizon):
            omega = -k_e * coerce(ep, e_max) - k_th * thp
            cost += (qe * ep ** 2 + qth * thp ** 2) * dt
            cost += pen * max(0.0, abs(ep) - e_max) ** 2 * dt
            ep, thp = step(ep, thp, v, omega, kappa_hat, dt)
        cost += qv * (v - v_ref) ** 2 * horizon * dt
        if cost < best_cost:
            best_cost, best_v = cost, v
    return best_v


def run_mpc(cal, duration, v_ref=150.0, qe=1.0, qth=0.0, r=25.0, horizon=15,
            qv=0.01, candidates=(40., 60., 80., 100., 120., 150., 180., 210.)):
    dt, e_max = cal["dt"], cal["e_max_mm"]
    candidates = tuple(c for c in candidates if c <= cal["wheel_max_mms"] * 0.95) or (v_ref,)
    log = []
    with LineFollower(cal) as bot:
        kappa_hat = 0.0
        for k, t in bot.loop(duration):
            e, theta, saturated = bot.read()
            v = _mpc_choose_speed(e, theta, kappa_hat, candidates, horizon, dt,
                                   e_max, v_ref, qe, qth, r, qv)
            k_e, k_th = lqr_gains_discrete(v, dt, qe, qth, r)
            omega = -k_e * e - k_th * theta
            bot.drive(v, omega)
            kappa_hat = 0.8 * kappa_hat + 0.2 * curvature_estimate(omega, v)
            log.append({"t": t, "e": e, "theta": theta, "v": v, "omega": omega,
                         "saturated": bool(saturated)})
        bot.stop()
    return log


RUNNERS = {"bangbang": run_bangbang, "pd": run_pd, "lqr": run_lqr, "mpc": run_mpc}
