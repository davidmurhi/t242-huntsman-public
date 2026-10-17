#!/usr/bin/env python3
"""
walk.py

Makes the CAD-calibrated Huntsman walk in a straight line.

Run spider.py first so spider.xml exists. This file only drives it.

THE GAIT: PAIRED WAVE
    Four beats per cycle, two legs in the air on each beat, six planted:

        beat 1   l2 + l3     left middle pair
        beat 2   r1 + r4     right front and rear
        beat 3   r2 + r3     right middle pair
        beat 4   l1 + l4     left front and rear

    Why these pairs, and not the row-by-row gait used on the placeholder
    model:

    1. The middle legs' hips are 62 mm apart in V Source's CAD, leaving
       6 mm between their femur brackets. If l2 and l3 ever swing in
       opposite directions they touch almost immediately (spider.py
       --check measures this from the CAD geometry). Here the two middle legs on a side always lift and
       swing together, so the gap between them never changes.

    2. The front legs are fanned about 30 degrees forward and the rear
       legs about 23 degrees back, so their hips push sideways as well as
       backwards. When l1 and l4 step together those sideways pushes
       largely cancel, and r1 with r4 cancels the remainder on the
       mirrored beat. The middle legs point straight out and push
       straight back. Every beat is balanced side to side, so the robot
       should not crab.

    3. Two feet up, six down, the same support the placeholder model
       needed to stop rocking. Each beat lifts one diagonal-ish pair, so
       the centre of mass stays inside the six planted feet.

HOW A FOOT LIFTS
    The knee raises the femur and the ankle turns back by the same amount
    (ANKLE_FOLLOW = 1.0), so the tibia keeps its angle and the foot rises
    almost straight up, moving slightly inward. On the placeholder model,
    raising the knee threw the foot outward towards the neighbouring leg;
    keeping the tibia parallel removes that.

RUN
    macOS            mjpython walk.py
    Windows / Linux  python walk.py
    python3 walk.py --check    walk headless for 20 s and report

CONTROLS
    SPACE pause   S stand   R reset   T above   F follow
    C contact markers   X snapshot   UP quicker   DOWN slower   Q quit
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import mujoco

# =====================================================================
# THE WALK - numbers to tune. Angles in degrees, times in seconds.
# =====================================================================

STEP_REACH = 8.0        # hip swing either side of centre
STEP_LIFT = 15.0        # knee lift at the top of the step (~25 mm of foot)
ANKLE_FOLLOW = 1.0      # ankle turns back by this share of the knee lift
STEP_TIME = 1.6         # seconds for a full cycle (all 8 legs step once)
SWING_SHARE = 0.22      # share of the cycle each leg spends in the air
EASE_IN = 1.5           # seconds to grow from standing to full stride

SPEED_STEP = 0.1
FASTEST, SLOWEST = 0.6, 4.0

try:
    from spider import LEG_FAN
except ImportError:
    LEG_FAN = (30.0, 0.0, 0.0, -22.7)

# Which beat (0-3) each leg steps on.
BEATS = {"l2": 0, "l3": 0, "r1": 1, "r4": 1,
         "r2": 2, "r3": 2, "l1": 3, "l4": 3}


def swing_scale(pair_index):
    """Fanned legs turn only cos(fan) of their swing into forward travel,
    so they swing 1/cos(fan) as far to keep pace with the middle legs."""
    return 1.0 / math.cos(math.radians(LEG_FAN[pair_index]))


def _build_legs():
    legs = []
    for leg, beat in BEATS.items():
        side = 1.0 if leg[0] == "l" else -1.0
        pair = int(leg[1])
        # A leg lifts when (cycle + offset) wraps past zero, so the offset
        # is the mirror of the moment it should step.
        offset = (1.0 - beat / 4.0) % 1.0
        legs.append((leg, side, offset, swing_scale(pair - 1)))
    return legs


LEGS = _build_legs()


def ease(x):
    return x * x * (3.0 - 2.0 * x)


def leg_targets(phase, reach):
    """(sweep, lift) for one leg. sweep is positive with the foot forward."""
    if phase < SWING_SHARE:
        through = phase / SWING_SHARE
        return -reach + 2.0 * reach * ease(through), \
            STEP_LIFT * math.sin(math.pi * through)
    through = (phase - SWING_SHARE) / (1.0 - SWING_SHARE)
    return reach - 2.0 * reach * through, 0.0


def drive(data, actuators, elapsed, step_time):
    """Set all 24 servo targets for this instant."""
    growth = min(1.0, elapsed / EASE_IN) if EASE_IN > 0 else 1.0
    growth = growth * growth * (3.0 - 2.0 * growth)
    cycle = (elapsed / step_time) % 1.0
    for leg, side, offset, scale in LEGS:
        sweep, lift = leg_targets((cycle + offset) % 1.0,
                                  STEP_REACH * scale * growth)
        # Hip turns anticlockwise for positive, which is backwards for a
        # left leg and forwards for a right one.
        data.ctrl[actuators[f"{leg}_hip"]] = -side * sweep
        data.ctrl[actuators[f"{leg}_knee"]] = lift * growth
        data.ctrl[actuators[f"{leg}_ankle"]] = -lift * ANKLE_FOLLOW * growth


def stand(data):
    data.ctrl[:] = 0.0


def load():
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "spider.xml")
    if not os.path.exists(path):
        print("spider.xml not found. Run first:  python3 spider.py --save")
        sys.exit(1)
    model = mujoco.MjModel.from_xml_path(path)
    actuators = {}
    for leg, *_ in LEGS:
        for joint in ("hip", "knee", "ankle"):
            name = f"{leg}_{joint}"
            idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if idx == -1:
                print(f"motor {name} is missing from the model")
                sys.exit(1)
            actuators[name] = idx
    return model, actuators


def tilt_of(quat):
    _w, x, y, _z = quat
    upright = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, upright))))


def _is_ground(name):
    return name == "ground"


# =====================================================================
# HEADLESS CHECK
# =====================================================================

def check(model, actuators, seconds=20.0):
    data = mujoco.MjData(model)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body")
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
             for i in range(model.ngeom)]
    limits = model.actuator_forcerange[:, 1].copy()

    for _ in range(800):
        mujoco.mj_step(model, data)
    settled = data.xpos[body].copy()

    first, touching_frames, body_floor = None, 0, 0
    saturated, samples, worst_tilt, lowest = 0, 0, 0.0, 9.0
    start = data.xpos[body].copy()
    steps = int(seconds / model.opt.timestep)
    for i in range(steps):
        drive(data, actuators, i * model.opt.timestep, STEP_TIME)
        mujoco.mj_step(model, data)

        hit = False
        for c in range(data.ncon):
            one = names[data.contact[c].geom1]
            two = names[data.contact[c].geom2]
            if _is_ground(one) or _is_ground(two):
                other = two if _is_ground(one) else one
                if not other.endswith("_tip"):
                    body_floor += 1
                continue
            hit = True
            if first is None:
                first = (i * model.opt.timestep, one, two,
                         tilt_of(data.xquat[body]))
        touching_frames += hit
        saturated += int((abs(data.actuator_force) >= 0.95 * limits).sum())
        samples += model.nu
        worst_tilt = max(worst_tilt, tilt_of(data.xquat[body]))
        lowest = min(lowest, data.xpos[body][2])

    end = data.xpos[body].copy()
    fwd, side = end[0] - start[0], end[1] - start[1]

    print()
    print("Walk check - paired wave gait on the CAD model")
    print("=" * 56)
    print(f"  stood at             {settled[2]:.4f} m")
    print(f"  walked forward       {fwd:+.3f} m in {seconds:.0f} s "
          f"({fwd / seconds * 100:+.1f} cm/s)")
    print(f"  drifted sideways     {side:+.3f} m "
          f"({abs(side) / max(abs(fwd), 1e-6):.0%} of forward)")
    print(f"  height  end {end[2]:.4f} m, lowest {lowest:.4f} m")
    print(f"  worst tilt           {worst_tilt:.1f} deg")
    print(f"  servos at their limit {saturated / samples:.1%} of the time")
    print(f"  legs touching legs   {touching_frames} of {steps} frames")
    if first:
        print(f"    first at {first[0]:.2f} s: {first[1]} / {first[2]}, "
              f"tilt {first[3]:.1f} deg")
    print(f"  body touching floor  {body_floor} frames")
    print()
    if end[2] < 0.04 or worst_tilt > 40:
        verdict = "IT FELL OVER."
    elif fwd < 0.02:
        verdict = "IT IS NOT MAKING PROGRESS."
    elif abs(side) > abs(fwd) * 0.25:
        verdict = "IT WALKS, BUT VEERS."
    elif touching_frames:
        verdict = "IT WALKS STRAIGHT, BUT LEGS TOUCH."
    else:
        verdict = "IT WALKS STRAIGHT WITH NO LEG CONTACT."
    print(f"  {verdict}")
    print("=" * 56)
    return 0


# =====================================================================
# VIEWER
# =====================================================================

def watch(model, actuators):
    import time
    import mujoco.viewer

    data = mujoco.MjData(model)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body")
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
             for i in range(model.ngeom)]
    state = {"paused": False, "standing": False, "above": False,
             "follow": True, "step_time": STEP_TIME, "live": True,
             "t0": time.time(), "x0": None, "ticks": 0,
             "contacts": False, "dump": False}

    def touching():
        out = []
        for c in range(data.ncon):
            one = names[data.contact[c].geom1]
            two = names[data.contact[c].geom2]
            if not (_is_ground(one) or _is_ground(two)):
                out.append((one, two, -data.contact[c].dist * 1000))
        return out

    def restart():
        mujoco.mj_resetData(model, data)
        for _ in range(400):
            mujoco.mj_step(model, data)
        state["t0"], state["x0"] = time.time(), None
        print("\n  reset")

    def on_key(code):
        if code == 32:
            state["paused"] = not state["paused"]
        elif code == 82:
            restart()
        elif code == 83:
            state["standing"] = not state["standing"]
        elif code == 84:
            state["above"] = not state["above"]
        elif code == 70:
            state["follow"] = not state["follow"]
        elif code == 265:
            state["step_time"] = max(FASTEST, state["step_time"] - SPEED_STEP)
        elif code == 264:
            state["step_time"] = min(SLOWEST, state["step_time"] + SPEED_STEP)
        elif code == 67:
            state["contacts"] = not state["contacts"]
        elif code == 88:
            state["dump"] = True
        elif code in (81, 256):
            state["live"] = False

    print("\n  SPACE pause   S stand   R reset   T above   F follow")
    print("  C contact markers   X snapshot   UP/DOWN speed   Q quit\n")

    with mujoco.viewer.launch_passive(model, data,
                                      key_callback=on_key) as viewer:
        viewer.cam.lookat[:] = [0.0, 0.0, 0.08]
        viewer.cam.distance, viewer.cam.elevation = 1.1, -20.0
        viewer.cam.azimuth = 135.0
        was_above = False
        restart()
        while viewer.is_running() and state["live"]:
            if state["paused"]:
                viewer.sync()
                time.sleep(0.02)
                continue
            elapsed = time.time() - state["t0"]
            pos = data.xpos[body].copy()
            if state["x0"] is None:
                state["x0"] = pos[0]
            if state["standing"]:
                stand(data)
            else:
                drive(data, actuators, elapsed, state["step_time"])
            mujoco.mj_step(model, data)

            if state["follow"]:
                viewer.cam.lookat[:2] = pos[:2]
            if state["above"] != was_above:
                if state["above"]:
                    viewer.cam.elevation, viewer.cam.azimuth = -89.0, 0.0
                else:
                    viewer.cam.elevation, viewer.cam.azimuth = -20.0, 135.0
                was_above = state["above"]
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = \
                state["contacts"]
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = \
                state["contacts"]
            if state["dump"]:
                print("\n  touching:", touching() or "nothing but the floor")
                state["dump"] = False

            state["ticks"] += 1
            if state["ticks"] % 60 == 0:
                t = touching()
                note = f"  TOUCHING {t[0][0]}/{t[0][1]}" if t else "  clear"
                print(f"\r  {elapsed:5.1f}s  fwd {pos[0] - state['x0']:+6.3f}m"
                      f"  drift {pos[1]:+6.3f}m  h {pos[2]:5.3f}m{note:<40}",
                      end="", flush=True)
            viewer.sync()
            time.sleep(1.0 / 240.0)
    print("\n")
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    model, actuators = load()
    print(f"loaded spider.xml - {model.nu} servos, "
          f"{sum(model.body_mass):.3f} kg")
    print("gait: paired wave, two legs up per beat")
    for leg, _s, off, sc in sorted(LEGS, key=lambda r: (1 - r[2]) % 1):
        print(f"    {leg}  steps at {(1.0 - off) % 1.0:.0%} of the cycle, "
              f"swings {STEP_REACH * sc:.1f} deg")
    if args.check:
        return check(model, actuators)
    return watch(model, actuators)


if __name__ == "__main__":
    sys.exit(main())
