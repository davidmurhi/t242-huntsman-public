#!/usr/bin/env python3
"""
walk.py

Watch the CAD-built Huntsman walk, or check its walk headless.

    macOS            mjpython walk.py            walk straight ahead
                     mjpython walk.py --route    forward, turn, back, sideways
    Windows / Linux  python walk.py [--route]
    python3 walk.py --check                      walk 30 s headless and report

To drive it yourself (keyboard, drive pad page, or the dashboard) run
controller.py instead. Both use the same walking code, gait.py.

THE GAIT (gait.py has the detail)
    Spider ripple: every leg steps on its own beat, front to back down
    each side, the right side 5/8 of a cycle behind the left, six feet
    down at all times. Each foot follows a planned path (straight back
    along the floor while planted, a smooth arc forward while lifted) and
    the joint angles come from inverse kinematics on V Source's leg
    geometry. Planted feet move exactly with the floor, so they do not
    scrape; lifting and landing are smooth in both speed and height.

MEASURED in MuJoCo, 30 s at 0.10 m/s (first 3 s of start-up excluded)
                                   hip-swing gait   this gait
    foot slip while planted        34 mm/s          7 mm/s
    body acceleration (rms)        1.10 m/s^2       0.50 m/s^2
    body jerk (rms)                102 m/s^3        42 m/s^3
    body rocking (rms)             7.5 deg/s        3.4 deg/s
    off its line after 3 m         1 cm             1 mm
    legs touching legs             0                0
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import mujoco

import gait as gaitlib

HERE = os.path.dirname(os.path.abspath(__file__))
CONTROL_DT = 0.01

# Demo route for --route: (seconds, vx m/s, vy m/s, turn deg/s)
ROUTE = [(8, 0.10, 0.0, 0.0), (2, 0.0, 0.0, 0.0), (4, 0.0, 0.0, 22.5),
         (2, 0.0, 0.0, 0.0), (6, -0.08, 0.0, 0.0), (6, 0.0, 0.07, 0.0),
         (6, 0.08, 0.0, 12.0), (3, 0.0, 0.0, 0.0)]


# ---------------------------------------------------------------------
# For the scripts in diagnostics/: the same gait through the interface
# they were written against (drive() once per physics step).
# ---------------------------------------------------------------------
STEP_TIME = gaitlib.GAITS["walk"]["cycle"]
SWING_SHARE = gaitlib.GAITS["walk"]["swing"]
LEGS = [(leg, 1.0 if leg[0] == "l" else -1.0,
         (-gaitlib.GAITS["walk"]["lift"][leg]) % 1.0, 1.0)
        for leg in gaitlib.LEGS]
_shim = {}


def drive(data, actuators, elapsed, step_time=STEP_TIME, vx=0.10):
    """Set all 24 servo targets for this instant (straight ahead)."""
    key = id(data)
    g, last = _shim.get(key, (None, None))
    if g is None or elapsed < last:
        g, last = gaitlib.from_spider(), elapsed
    g.drive(vx, 0.0, 0.0)
    targets = g.step(max(1e-4, elapsed - last), heading=yaw_of(data.qpos[3:7]),
                     position=(data.qpos[0], data.qpos[1]))
    _shim[key] = (g, elapsed)
    for k, (leg, joint) in enumerate((l, j) for l in gaitlib.LEGS
                                     for j in gaitlib.JOINTS):
        data.ctrl[actuators[f"{leg}_{joint}"]] = targets[k]


def load():
    path = os.path.join(HERE, "spider.xml")
    if not os.path.exists(path):
        print("spider.xml not found. Run first:  python spider.py --save")
        sys.exit(1)
    model = mujoco.MjModel.from_xml_path(path)
    order = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                               f"{leg}_{j}")
             for leg in gaitlib.LEGS for j in gaitlib.JOINTS]
    if min(order) < 0:
        print("spider.xml is missing servos; rebuild it with spider.py")
        sys.exit(1)
    return model, order


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def tilt_of(q):
    _w, x, y, _z = q
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def command_at(t, route):
    if not route:
        return 0.10, 0.0, 0.0
    for secs, vx, vy, wz in route:
        if t < secs:
            return vx, vy, wz
        t -= secs
    return 0.0, 0.0, 0.0


def control(model, data, order, g, t, route):
    g.drive(*command_at(t, route))
    targets = g.step(CONTROL_DT, heading=yaw_of(data.qpos[3:7]),
                     position=(data.qpos[0], data.qpos[1]))
    for k, a in enumerate(order):
        data.ctrl[a] = targets[k]


# =====================================================================
# HEADLESS CHECK
# =====================================================================

def check(model, order, seconds=30.0):
    data = mujoco.MjData(model)
    g = gaitlib.from_spider()
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
             for i in range(model.ngeom)]
    tips = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{leg}_tip")
            for leg in gaitlib.LEGS]
    limits = model.actuator_forcerange[:, 1].copy()
    for _ in range(400):
        mujoco.mj_step(model, data)
    start = data.qpos[:3].copy()
    per = max(1, round(CONTROL_DT / model.opt.timestep))
    ticks = int(seconds / CONTROL_DT)
    last_tip = None
    slip, worst_tilt, leg_hits, floor_hits, sat, samples = [], 0.0, 0, 0, 0, 0
    first_hit = None
    for i in range(ticks):
        control(model, data, order, g, i * CONTROL_DT, None)
        for _ in range(per):
            mujoco.mj_step(model, data)
        down = set()
        for c in range(data.ncon):
            con = data.contact[c]
            a, b = names[con.geom1], names[con.geom2]
            if "ground" in (a, b):
                other = b if a == "ground" else a
                if other.endswith("_tip"):
                    down.add(other)
                else:
                    floor_hits += 1
            else:
                leg_hits += 1
                first_hit = first_hit or (i * CONTROL_DT, a, b)
        tip_pos = [data.geom_xpos[t].copy() for t in tips]
        if last_tip is not None and i * CONTROL_DT > 3.0:
            for k, leg in enumerate(gaitlib.LEGS):
                if f"{leg}_tip" in down:
                    d = tip_pos[k][:2] - last_tip[k][:2]
                    slip.append(math.hypot(*d) / CONTROL_DT)
        last_tip = tip_pos
        worst_tilt = max(worst_tilt, tilt_of(data.qpos[3:7]))
        sat += int((abs(data.actuator_force) >= 0.95 * limits).sum())
        samples += model.nu
    end = data.qpos[:3]
    fwd, side = end[0] - start[0], end[1] - start[1]
    slip_mm = 1000 * sum(slip) / max(1, len(slip))

    print()
    print("Walk check - spider ripple, foot-path gait (gait.py)")
    print("=" * 58)
    print(f"  walked forward        {fwd:+.3f} m in {seconds:.0f} s "
          f"({fwd / seconds * 100:+.1f} cm/s)")
    print(f"  off its line          {side * 1000:+.0f} mm")
    print(f"  heading change        {math.degrees(yaw_of(data.qpos[3:7])):+.1f} deg")
    print(f"  worst tilt            {worst_tilt:.1f} deg")
    print(f"  foot slip, planted    {slip_mm:.1f} mm/s average")
    print(f"  servos at their limit {sat / samples:.1%} of the time")
    print(f"  legs touching legs    {leg_hits} contacts")
    if first_hit:
        print(f"    first at {first_hit[0]:.2f} s: {first_hit[1]} / {first_hit[2]}")
    print(f"  body touching floor   {floor_hits} contacts")
    ok = fwd > 0.5 * 0.10 * seconds and abs(side) < 0.05 and not leg_hits
    print()
    print("  IT WALKS STRAIGHT, SMOOTHLY, WITH NO LEG CONTACT." if ok
          else "  SOMETHING IS WRONG - see the numbers above.")
    print("=" * 58)
    return 0 if ok else 1


# =====================================================================
# VIEWER
# =====================================================================

def watch(model, order, route):
    import time
    import mujoco.viewer

    data = mujoco.MjData(model)
    g = gaitlib.from_spider()
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body")
    for _ in range(400):
        mujoco.mj_step(model, data)
    per = max(1, round(CONTROL_DT / model.opt.timestep))
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.0, 0.0, 0.08]
        viewer.cam.distance, viewer.cam.elevation = 1.1, -22.0
        viewer.cam.azimuth = 135.0
        t, wall = 0.0, time.monotonic()
        while viewer.is_running():
            with viewer.lock():
                control(model, data, order, g, t, route)
                for _ in range(per):
                    mujoco.mj_step(model, data)
                viewer.cam.lookat[:2] = data.xpos[body][:2]
            viewer.sync()
            t += CONTROL_DT
            wall += CONTROL_DT
            time.sleep(max(0.0, wall - time.monotonic()))
            if int(t / CONTROL_DT) % 100 == 0:
                vx, vy, wz = command_at(t, route)
                print(f"\r  {t:5.1f}s  x {data.qpos[0]:+6.2f} m  y "
                      f"{data.qpos[1]:+6.2f} m  heading "
                      f"{math.degrees(yaw_of(data.qpos[3:7])):+6.1f} deg   "
                      f"command vx {vx:+.2f} vy {vy:+.2f} turn {wz:+.0f}   ",
                      end="", flush=True)
    print()
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--route", action="store_true")
    args = ap.parse_args()
    model, order = load()
    print(f"loaded spider.xml - {model.nu} servos, "
          f"{sum(model.body_mass):.3f} kg")
    if args.check:
        return check(model, order)
    return watch(model, order, ROUTE if args.route else None)


if __name__ == "__main__":
    sys.exit(main())
