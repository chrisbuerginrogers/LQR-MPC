"""
Live view of a line-follower run, with earlier runs faded in behind it.

Launched automatically by run_bangbang.py / run_pd.py / run_lqr.py /
run_mpc.py -- you shouldn't normally need to run this by hand. If you do:

    python3 code/live_plot.py code/runs/lqr_20260101_120000.csv --name lqr

It runs in its own process on purpose: a slow matplotlib redraw here can
never cost the control loop a 50 ms period, because it isn't in that
process at all. It reads the CSV the run script is writing to, the same way
`tail -f` would, and redraws every ~150 ms.
"""

import argparse
import os

import matplotlib.pyplot as plt

from telemetry import CONTROLLERS, load_run, previous_runs, latest_run_per_name

HISTORY_COLOR = "0.75"
OTHER_COLORS = {"bangbang": "tab:orange", "pd": "tab:green",
                "lqr": "tab:blue", "mpc": "tab:red"}
POLL_S = 0.15
IDLE_POLLS_BEFORE_SLOW = 20  # ~3 s of no new rows -> assume the run is over


def tail_rows(path):
    """Yield newly-appended CSV rows (as dicts) as they're written, else None."""
    f = open(path, newline="")
    header = None
    buf = ""
    while True:
        chunk = f.read()
        if not chunk:
            yield None
            continue
        buf += chunk
        lines = buf.split("\n")
        buf = lines.pop()  # keep the trailing partial line for next time
        for line in lines:
            if not line or line.startswith("#"):
                continue
            if header is None:
                header = line.split(",")
                continue
            yield dict(zip(header, line.split(",")))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("live_csv")
    ap.add_argument("--name", required=True, choices=CONTROLLERS)
    ap.add_argument("--e-max", type=float, default=6.0)
    ap.add_argument("--history", type=int, default=2)
    args = ap.parse_args()

    fig, (ax_e, ax_v) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    fig.canvas.manager.set_window_title(f"{args.name} -- live")

    for p in previous_runs(args.name, limit=args.history, exclude_path=args.live_csv):
        d = load_run(p)
        ax_e.plot(d["t"], d["e"], color=HISTORY_COLOR, lw=1, zorder=1)
        ax_v.plot(d["t"], d["v"], color=HISTORY_COLOR, lw=1, zorder=1)
    if args.history:
        ax_v.plot([], [], color=HISTORY_COLOR, lw=1,
                   label=f"{args.name} (earlier runs)")

    others = latest_run_per_name([n for n in CONTROLLERS if n != args.name],
                                  exclude_path=args.live_csv)
    for name, p in others.items():
        d = load_run(p)
        c = OTHER_COLORS.get(name, "gray")
        ax_e.plot(d["t"], d["e"], color=c, lw=1.2, alpha=0.6, zorder=2)
        ax_v.plot(d["t"], d["v"], color=c, lw=1.2, alpha=0.6, zorder=2,
                  label=name)

    (line_e,) = ax_e.plot([], [], color="black", lw=2, zorder=3)
    (line_v,) = ax_v.plot([], [], color="black", lw=2, zorder=3,
                          label=f"{args.name} (live)")

    ax_e.axhline(args.e_max, color="red", ls="--", lw=1)
    ax_e.axhline(-args.e_max, color="red", ls="--", lw=1)
    ax_e.set_ylabel("e [mm]")
    ax_v.set_ylabel("v [mm/s]")
    ax_v.set_xlabel("t [s]")
    ax_v.legend(loc="upper right", fontsize=8)
    fig.tight_layout()

    plt.ion()
    plt.show()

    t, e, v = [], [], []
    idle_polls = 0
    reader = tail_rows(args.live_csv)
    while plt.fignum_exists(fig.number):
        row = next(reader)
        if row is None:
            idle_polls += 1
            plt.pause(POLL_S if idle_polls < IDLE_POLLS_BEFORE_SLOW else 1.0)
            continue
        idle_polls = 0
        t.append(float(row["t"]))
        e.append(float(row["e"]))
        v.append(float(row["v"]))
        line_e.set_data(t, e)
        line_v.set_data(t, v)
        for ax in (ax_e, ax_v):
            ax.relim()
            ax.autoscale_view()
        plt.pause(0.001)


if __name__ == "__main__":
    main()
