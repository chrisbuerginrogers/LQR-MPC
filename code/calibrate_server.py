"""
Local server behind docs/interactive.html's hardware buttons.

Serves this repo as a website -- every README.md/docs/*.md renders as styled
HTML instead of plain text, and docs/interactive.html gets a JSON API that
lets it talk to the real robot: redo calibration, and run bang-bang/PD/LQR/
MPC on real hardware for a few seconds and plot what actually happened.

    python3 -m venv .venv && .venv/bin/pip install -r code/requirements.txt
    .venv/bin/python3 code/calibrate_server.py
    open http://localhost:8000/

Only one browser tab should drive the robot at a time -- the connection
state (which sensor/motor object is open) lives in this process, not per
request.
"""

import json
import os
import sys
import threading
import time
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler

import markdown as markdown_lib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import legoeducation as le
from calibrate import sample_reflection
from hardware_runs import RUNNERS
from robot import CAL_PATH, DEFAULTS

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

MD_EXTENSIONS = ["tables", "fenced_code", "sane_lists", "toc", "pymdownx.arithmatex"]
MD_EXTENSION_CONFIGS = {"pymdownx.arithmatex": {"generic": True}}

MATHJAX_SCRIPT = """
<script>
  window.MathJax = {
    tex: { inlineMath: [["\\\\(", "\\\\)"]], displayMath: [["\\\\[", "\\\\]"]] }
  };
</script>
<script src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>
"""

NAV_LINKS = [
    ("/README.md", "Home"),
    ("/docs/00-sim.md", "00 · sim.py"),
    ("/docs/01-lqr.md", "01 · LQR"),
    ("/docs/02-mpc.md", "02 · MPC"),
    ("/docs/03-lqr-vs-mpc.md", "03 · LQR vs MPC"),
    ("/docs/interactive.html", "Interactive"),
]

PAGE_CSS = """
<style>
  :root {
    --bg: #f5f6f8; --panel-bg: #ffffff; --fg: #1b1e24; --muted: #5b6270;
    --grid: #e2e4e9; --border: #d8dbe1; --accent: #2563eb; --ok: #157a3d; --bad: #b3261e;
  }
  @media (prefers-color-scheme: dark) {
    :root {
      --bg: #14161a; --panel-bg: #1c1f26; --fg: #e7e9ee; --muted: #9aa2b1;
      --grid: #333844; --border: #333844; --accent: #6ea8fe; --ok: #4fd17a; --bad: #ff6b6b;
    }
  }
  * { box-sizing: border-box; }
  html, body { margin: 0; background: var(--bg); color: var(--fg);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Helvetica, Arial, sans-serif;
    font-size: 15px; line-height: 1.6; }
  .site-nav { display: flex; flex-wrap: wrap; gap: 4px; align-items: center;
    padding: 10px 20px; border-bottom: 1px solid var(--border); background: var(--panel-bg); }
  .site-nav a { color: var(--muted); text-decoration: none; font-size: 13px;
    padding: 6px 10px; border-radius: 6px; }
  .site-nav a:hover { background: var(--grid); color: var(--fg); }
  .site-nav a.active { background: var(--accent); color: #fff; }
  .site-nav a.robot-btn { margin-left: auto; background: var(--ok); color: #fff; font-weight: 600; }
  main.doc { max-width: 78ch; margin: 0 auto; padding: 28px 20px 60px 20px; }
  main.doc h1, main.doc h2, main.doc h3 { line-height: 1.25; }
  main.doc h1 { font-size: 26px; border-bottom: 1px solid var(--border); padding-bottom: 10px; }
  main.doc h2 { font-size: 20px; margin-top: 2.2em; border-bottom: 1px solid var(--border); padding-bottom: 6px; }
  main.doc h3 { font-size: 16px; margin-top: 1.8em; }
  main.doc a { color: var(--accent); }
  main.doc code { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size: 0.9em;
    background: var(--grid); padding: 0.15em 0.4em; border-radius: 4px; }
  main.doc pre { background: var(--panel-bg); border: 1px solid var(--border); border-radius: 8px;
    padding: 12px 14px; overflow-x: auto; }
  main.doc pre code { background: none; padding: 0; }
  main.doc blockquote { border-left: 3px solid var(--accent); margin: 1.2em 0; padding: 4px 16px;
    background: var(--panel-bg); border-radius: 0 6px 6px 0; color: var(--muted); }
  main.doc table { border-collapse: collapse; width: 100%; margin: 1.2em 0; font-size: 14px;
    display: block; overflow-x: auto; }
  main.doc th, main.doc td { border: 1px solid var(--border); padding: 6px 10px; text-align: left; }
  main.doc th { background: var(--panel-bg); }
  main.doc hr { border: none; border-top: 1px solid var(--border); margin: 2em 0; }
  main.doc img { max-width: 100%; border-radius: 8px; }
</style>
"""


def nav_link(href, label, active):
    cls = ' class="active"' if href == active else ""
    return f'<a href="{href}"{cls}>{label}</a>'


def render_markdown_page(md_text, title, active_path):
    body_html = markdown_lib.markdown(
        md_text, extensions=MD_EXTENSIONS, extension_configs=MD_EXTENSION_CONFIGS)
    nav_html = "\n  ".join(nav_link(href, label, active_path) for href, label in NAV_LINKS)
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title}</title>
{PAGE_CSS}
{MATHJAX_SCRIPT}
</head>
<body>
<div class="site-nav">
  {nav_html}
  <a href="/docs/interactive.html#robot" class="robot-btn">\U0001f50c Connect to robot</a>
</div>
<main class="doc">
{body_html}
</main>
</body>
</html>"""


# --------------------------------------------------------------------------
# Calibration + real-run session (single, process-global -- one robot)
# --------------------------------------------------------------------------
class CalibrationSession:
    def __init__(self):
        self.lock = threading.Lock()
        self.cal = dict(DEFAULTS)
        if os.path.exists(CAL_PATH):
            with open(CAL_PATH) as f:
                self.cal.update(json.load(f))
        self.sensor = None
        self.motors = None

    def connect_sensor(self):
        with self.lock:
            if self.sensor is not None:
                return
            sensor = le.ColorSensor()
            sensor.connect(device_notification_delay=50)
            if not sensor.connected:
                raise RuntimeError(
                    "No Color Sensor found. Check it's powered on and not "
                    "already connected to another app, then try again.")
            self.sensor = sensor

    def live_reflection(self):
        if self.sensor is None:
            raise RuntimeError("sensor not connected")
        return float(self.sensor.sensor.reflection)

    def sample_black(self, seconds=3.0):
        if self.sensor is None:
            raise RuntimeError("sensor not connected")
        val, lo, hi = sample_reflection(self.sensor, seconds=seconds)
        self.cal["reflect_black"] = val
        return {"value": val, "lo": lo, "hi": hi}

    def sample_white(self, seconds=3.0):
        if self.sensor is None:
            raise RuntimeError("sensor not connected")
        val, lo, hi = sample_reflection(self.sensor, seconds=seconds)
        self.cal["reflect_white"] = val
        return {"value": val, "lo": lo, "hi": hi,
                 "contrast": self.cal["reflect_white"] - self.cal["reflect_black"]}

    def set_window(self, span_mm):
        self.cal["e_max_mm"] = span_mm / 2.0
        return self.cal["e_max_mm"]

    def disconnect_sensor(self):
        with self.lock:
            if self.sensor is not None:
                self.sensor.disconnect()
                self.sensor = None

    def connect_motors(self):
        with self.lock:
            if self.motors is not None:
                return
            motors = le.DoubleMotor()
            motors.connect(device_notification_delay=100)
            if not motors.connected:
                raise RuntimeError(
                    "No Double Motor found. Check it's powered on and not "
                    "already connected to another app, then try again.")
            motors.movement_set_acceleration(100, 100)
            self.motors = motors

    def drive_test(self, seconds=2.0, speed=100):
        if self.motors is None:
            raise RuntimeError("motors not connected")
        self.motors.movement_move_tank(speed, speed, blocking=False)
        time.sleep(seconds)
        self.motors.movement_stop()

    def set_speed(self, distance_mm, seconds=2.0):
        self.cal["wheel_max_mms"] = distance_mm / seconds
        return self.cal["wheel_max_mms"]

    def set_track_width(self, width_mm):
        self.cal["track_width_mm"] = width_mm
        return width_mm

    def disconnect_motors(self):
        with self.lock:
            if self.motors is not None:
                self.motors.disconnect()
                self.motors = None

    def save(self):
        with open(CAL_PATH, "w") as f:
            json.dump(self.cal, f, indent=2)
        return self.cal

    def run_controller(self, name, duration, params):
        if name not in RUNNERS:
            raise ValueError(f"unknown controller {name!r}")
        if self.sensor is not None or self.motors is not None:
            raise RuntimeError("disconnect the calibration sensor/motors before running a controller")
        with self.lock:
            return RUNNERS[name](self.cal, duration, **params)


session = CalibrationSession()


# --------------------------------------------------------------------------
# HTTP handler
# --------------------------------------------------------------------------
class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=REPO_ROOT, **kwargs)

    def log_message(self, fmt, *args):
        pass

    def _send_json(self, status, payload):
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_html(self, status, html):
        body = html.encode()
        self.send_response(status)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self):
        length = int(self.headers.get("Content-Length", 0))
        if length == 0:
            return {}
        return json.loads(self.rfile.read(length) or b"{}")

    def do_GET(self):
        if self.path == "/":
            self.send_response(302)
            self.send_header("Location", "/README.md")
            self.end_headers()
            return
        if self.path.startswith("/api/"):
            return self._api(self.path, {})
        clean_path = self.path.split("?", 1)[0].split("#", 1)[0]
        if clean_path.endswith(".md"):
            return self._serve_markdown(clean_path)
        return super().do_GET()

    def do_POST(self):
        if self.path.startswith("/api/"):
            try:
                body = self._read_json()
            except Exception as exc:
                return self._send_json(400, {"error": f"bad request body: {exc}"})
            return self._api(self.path, body)
        return self._send_json(404, {"error": "not found"})

    def _serve_markdown(self, clean_path):
        fs_path = os.path.join(REPO_ROOT, clean_path.lstrip("/"))
        if not os.path.isfile(fs_path):
            return self._send_html(404, f"<h1>404</h1><p>{clean_path} not found</p>")
        with open(fs_path, encoding="utf-8") as f:
            md_text = f.read()
        title = os.path.basename(fs_path)
        html = render_markdown_page(md_text, title, clean_path)
        return self._send_html(200, html)

    def _api(self, path, body):
        try:
            if path == "/api/cal/status":
                return self._send_json(200, {
                    "cal": session.cal,
                    "sensor_connected": session.sensor is not None,
                    "motors_connected": session.motors is not None,
                })
            if path == "/api/cal/connect-sensor":
                session.connect_sensor()
                return self._send_json(200, {"ok": True})
            if path == "/api/cal/live":
                return self._send_json(200, {"reflection": session.live_reflection()})
            if path == "/api/cal/sample-black":
                return self._send_json(200, session.sample_black(float(body.get("seconds", 3.0))))
            if path == "/api/cal/sample-white":
                return self._send_json(200, session.sample_white(float(body.get("seconds", 3.0))))
            if path == "/api/cal/set-window":
                return self._send_json(200, {"e_max_mm": session.set_window(float(body["span_mm"]))})
            if path == "/api/cal/disconnect-sensor":
                session.disconnect_sensor()
                return self._send_json(200, {"ok": True})
            if path == "/api/cal/connect-motors":
                session.connect_motors()
                return self._send_json(200, {"ok": True})
            if path == "/api/cal/drive-test":
                session.drive_test(float(body.get("seconds", 2.0)), float(body.get("speed", 100)))
                return self._send_json(200, {"ok": True})
            if path == "/api/cal/set-speed":
                return self._send_json(200, {"wheel_max_mms":
                    session.set_speed(float(body["distance_mm"]), float(body.get("seconds", 2.0)))})
            if path == "/api/cal/set-track-width":
                return self._send_json(200, {"track_width_mm":
                    session.set_track_width(float(body["width_mm"]))})
            if path == "/api/cal/disconnect-motors":
                session.disconnect_motors()
                return self._send_json(200, {"ok": True})
            if path == "/api/cal/save":
                return self._send_json(200, {"cal": session.save()})
            if path.startswith("/api/run/"):
                name = path[len("/api/run/"):]
                duration = float(body.pop("duration", 8.0))
                log = session.run_controller(name, duration, body)
                return self._send_json(200, {"log": log})
            return self._send_json(404, {"error": f"unknown endpoint {path}"})
        except Exception as exc:
            return self._send_json(500, {"error": str(exc)})


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    server = ThreadingHTTPServer(("localhost", port), Handler)
    print(f"Serving {REPO_ROOT}")
    print(f"  http://localhost:{port}/            (README, rendered)")
    print(f"  http://localhost:{port}/docs/interactive.html")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        session.disconnect_sensor()
        session.disconnect_motors()


if __name__ == "__main__":
    main()
