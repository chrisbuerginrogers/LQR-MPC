"""
Lesson 0: measure your own robot.

Every number in the LQR and MPC designs comes from something you can measure
in ten minutes. Run this first; it writes code/calibration.json, which
robot.py loads automatically.

    python3 code/calibrate.py
"""

import json
import statistics
import threading
import time

import legoeducation as le

from robot import CAL_PATH, DEFAULTS


def sample_reflection(sensor, seconds=3.0, dt=0.05):
    vals = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        vals.append(float(sensor.sensor.reflection))
        time.sleep(dt)
    return statistics.median(vals), min(vals), max(vals)


def show_live_reflection(sensor, stop_event, dt=0.05):
    while not stop_event.is_set():
        val = float(sensor.sensor.reflection)
        print(f"\r     live reflection = {val:6.1f}   ", end="", flush=True)
        time.sleep(dt)
    print()


def main():
    cal = dict(DEFAULTS)

    sensor = le.ColorSensor()
    print("Connecting to Color Sensor...")
    sensor.connect(device_notification_delay=50)

    input("\n1/4  Park the sensor fully over the BLACK tape, then press Enter. ")
    black, lo, hi = sample_reflection(sensor)
    print(f"     black reflection = {black:.1f}  (spread {lo:.0f}..{hi:.0f})")
    cal["reflect_black"] = black

    input("\n2/4  Park the sensor fully over the WHITE background, press Enter. ")
    white, lo, hi = sample_reflection(sensor)
    print(f"     white reflection = {white:.1f}  (spread {lo:.0f}..{hi:.0f})")
    cal["reflect_white"] = white

    if white - black < 20:
        print("     WARNING: less than 20 counts of contrast. Fix the lighting or")
        print("     the sensor ride height before going any further.")

    print("\n3/4  Sensor window. Line the sensor up on the tape edge so the")
    print("     reading sits halfway between black and white. Slide the robot")
    print("     sideways until the reading just stops changing in each")
    print("     direction, and measure that total travel with a ruler.")
    print("     Watch the live reading below while you slide it:")
    stop_event = threading.Event()
    live_thread = threading.Thread(
        target=show_live_reflection, args=(sensor, stop_event), daemon=True
    )
    live_thread.start()
    input("     Press Enter once you've found the transition span. ")
    stop_event.set()
    live_thread.join()
    span = input("     Total sideways travel across the transition, in mm: ").strip()
    if span:
        cal["e_max_mm"] = float(span) / 2.0
    print(f"     e_max = {cal['e_max_mm']:.1f} mm "
          f"(the controller is blind outside +-e_max)")

    sensor.disconnect()

    print("\n4/4  Wheel speed. The robot will drive straight at 100% for 2 s.")
    print("     Mark the start, and measure how far it traveled.")
    input("     Clear a straight run of at least 1 m, then press Enter. ")
    motors = le.DoubleMotor()
    motors.connect(device_notification_delay=100)
    motors.movement_set_acceleration(100, 100)
    motors.movement_move_tank(100, 100, blocking=False)
    time.sleep(2.0)
    motors.movement_stop()
    dist = input("     Distance traveled in mm: ").strip()
    if dist:
        cal["wheel_max_mms"] = float(dist) / 2.0
    width = input("     Track width (between wheel contact patches) in mm "
                  f"[{cal['track_width_mm']:.0f}]: ").strip()
    if width:
        cal["track_width_mm"] = float(width)
    motors.disconnect()

    with open(CAL_PATH, "w") as f:
        json.dump(cal, f, indent=2)

    print(f"\nWrote {CAL_PATH}:")
    for key, value in cal.items():
        print(f"  {key:18s} {value}")

    v_max_window = cal["e_max_mm"] / cal["dt"]
    print(f"\nSanity check: at the {cal['dt'] * 1000:.0f} ms control period, a speed")
    print(f"of {v_max_window:.0f} mm/s moves the robot one whole sensor window between")
    print("samples. Stay well under that or no controller can help you.")


if __name__ == "__main__":
    main()
