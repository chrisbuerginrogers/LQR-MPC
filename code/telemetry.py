"""
Shared run logging for the line-follower demos.

Each run_*.py script writes one row per control period to a timestamped CSV
under code/runs/, then launches code/live_plot.py in a separate process to
render it live. Logging is a single buffered line write -- cheap enough not
to disturb the 50 ms control loop -- and all plotting/rendering work happens
outside this process entirely, so a slow redraw can never cost the robot a
control period.
"""

import csv
import glob
import os
import subprocess
import sys
import time

RUNS_DIR = os.path.join(os.path.dirname(__file__), "runs")
COLUMNS = ["t", "e", "theta", "v", "omega", "saturated", "left", "right"]
CONTROLLERS = ["bangbang", "pd", "lqr", "mpc"]


class RunLogger:
    """Appends one CSV row per control period, flushed immediately so a
    separate live-plotting process can tail the file in near real time."""

    def __init__(self, name, meta=None):
        os.makedirs(RUNS_DIR, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.name = name
        self.path = os.path.join(RUNS_DIR, f"{name}_{stamp}.csv")
        self._file = open(self.path, "w", newline="")
        for key, value in (meta or {}).items():
            self._file.write(f"# {key}={value}\n")
        self._writer = csv.writer(self._file, lineterminator="\n")
        self._writer.writerow(COLUMNS)
        self._file.flush()

    def log(self, t, e, theta, v, omega, saturated, left, right):
        self._writer.writerow([f"{t:.4f}", f"{e:.4f}", f"{theta:.5f}",
                                f"{v:.2f}", f"{omega:.4f}", int(saturated),
                                left, right])
        self._file.flush()

    def close(self):
        self._file.close()


def previous_runs(name, limit=2, exclude_path=None):
    """Earlier CSVs for this controller, newest first, current run excluded."""
    paths = sorted(glob.glob(os.path.join(RUNS_DIR, f"{name}_*.csv")),
                    key=os.path.getmtime, reverse=True)
    paths = [p for p in paths if p != exclude_path]
    return paths[:limit]


def latest_run_per_name(names, exclude_path=None):
    """Most recent CSV for each controller name in `names`, if one exists."""
    out = {}
    for n in names:
        found = previous_runs(n, limit=1, exclude_path=exclude_path)
        if found:
            out[n] = found[0]
    return out


def load_run(path):
    """Read a completed run CSV back into a dict of float lists."""
    data = {c: [] for c in COLUMNS}
    with open(path, newline="") as f:
        rows = (r for r in f if not r.startswith("#"))
        reader = csv.DictReader(rows)
        for row in reader:
            for c in COLUMNS:
                data[c].append(float(row[c]))
    return data


def spawn_live_plot(csv_path, name, e_max, disable=False):
    """Launch live_plot.py against this run's CSV in its own process.

    Runs fully decoupled from the caller: rendering can be slow or a display
    can be unavailable without ever touching the control loop's timing.
    """
    if disable:
        return None
    script = os.path.join(os.path.dirname(__file__), "live_plot.py")
    try:
        return subprocess.Popen([sys.executable, script, csv_path,
                                  "--name", name, "--e-max", str(e_max)])
    except OSError as exc:
        print(f"(live plot not started: {exc})")
        return None
