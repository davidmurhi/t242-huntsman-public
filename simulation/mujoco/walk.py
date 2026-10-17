#!/usr/bin/env python3
"""
walk.py

Makes the CAD-built Huntsman walk in a straight line, front first (the
battery rides at the back).

Run spider.py first so spider.xml exists. This file only drives it.

THE GAIT: A SPIDER'S RIPPLE
    Every leg steps on its own, one at a time, eight distinct moments per
    cycle. Down each side a wave runs from the front leg to the back, a
    quarter cycle apart; the right side runs five-eighths of a cycle
    behind the left, so no two legs ever lift at the same instant:

        left    l1 0.000   l2 0.250   l3 0.500   l4 0.750
        right   r1 0.625   r2 0.875   r3 0.125   r4 0.375

    About two feet are in the air at once and six stay planted.

    Why front to back, when many animals ripple back to front? Measured
    in MuJoCo on V Source's geometry: back-to-front makes each front leg
    swing forward just as the middle leg behind it reaches the back of
    its stroke, and their femur brackets meet. Front-to-back means a leg
    steps just after its neighbour ahead has landed forward, so
    neighbours always move the same way: zero leg contact, measured.

    The stance splay in spider.py (STANCE_YAW) gives every leg room of
    its own. In the CAD the two middle legs on a side point straight out
    with 6 mm between their brackets; standing, they splay apart like
    the built robot in Ronan's photos.

    Heading hold steers back to the starting line by lengthening one
    side's stride. The ripple is nearly balanced on its own (2.5 cm off
    line over 3 m without it); with it, under 1 cm.

    Measured over 60 s: 10 cm/s, within 1 cm of its line, body tilt
    under 1.5 deg, no leg contact, servos at their limit ~3.5% of the
    time.

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

STEP_REACH = 12.0       # hip swing either side of centre
STEP_LIFT = 15.0        # knee lift at the top of the step (~25 mm of foot)
ANKLE_FOLLOW = 1.0      # ankle turns back by this share of the knee lift
STEP_TIME = 0.9         # seconds for a full cycle (all 8 legs step once)
SWING_SHARE = 0.22      # share of the cycle each leg spends in the air
EASE_IN = 1.5           # seconds to grow from standing to full stride

# Heading hold. The eight legs are one CAD assembly repeated, not mirror
# images, so the servo horns sit on opposite sides left and right and the
# robot slowly crabs and turns if driven blind. Like the real robot will
# with its IMU, the gait watches the body's heading and how far it has
# drifted off its starting line, and lengthens one side's stride to steer
# back. Set HEADING_HOLD = False to see the uncorrected walk.
HEADING_HOLD = True
STEER_GAIN = 4.0        # stride change per radian of heading error
LINE_GAIN = 3.0         # heading aimed back at the line per metre off it
MAX_STEER = 0.35        # never change a side's stride by more than this

SPEED_STEP = 0.1
FASTEST, SLOWEST = 0.6, 4.0

try:
    from spider import LEG_FAN
except ImportError:
    LEG_FAN = (30.7, 15.0, -15.0, -38.0)

# When each leg lifts, as a fraction of the cycle. A spider's ripple: on
# each side a wave runs from the back leg to the front, a quarter cycle
# apart, and the right side runs SIDE_LAG behind the left. Every leg has
# its own moment; no two legs on the same side ever move together.
WAVE = {1: 0.00, 2: 0.25, 3: 0.50, 4: 0.75}     # pair -> lift time, left
SIDE_LAG = 0.625


def lift_time(leg):
    t = WAVE[int(leg[1])]
    return t if leg[0] == "l" else (t + SIDE_LAG) % 1.0


def swing_scale(pair_index):
    """Fanned legs turn only cos(fan) of their swing into forward travel,
    so they swing 1/cos(fan) as far to keep pace with the middle legs."""
    return 1.0 / math.cos(math.radians(LEG_FAN[pair_index]))


def _build_legs():
    legs = []
    for leg in ("l1", "r1", "l2", "r2", "l3", "r3", "l4", "r4"):
        side = 1.0 if leg[0] == "l" else -1.0
        pair = int(leg[1])
        # A leg lifts when (cycle + offset) wraps past zero, so the offset
        # is the mirror of the moment it should step.
        offset = (1.0 - lift_time(leg)) % 1.0
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


_line = {}


def steering(data, elapsed):
    """How much to lengthen the right side's stride (and shorten the
    left's) to hold a straight line. Positive turns the robot left."""
    if not HEADING_HOLD or not hasattr(data, "qpos"):
        return 0.0
    w, x, y, z = (float(v) for v in data.qpos[3:7])
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    key = id(data)
    # Start a new line on the first call, and whenever the clock goes
    # backwards (the viewer's reset key restarts it).
    if key not in _line or elapsed < _line[key][3]:
        _line[key] = (float(data.qpos[0]), float(data.qpos[1]), yaw, elapsed)
    x0, y0, yaw0, _t = _line[key]
    _line[key] = (x0, y0, yaw0, elapsed)
    # Distance off the starting line, measured square to it.
    off = (-(data.qpos[0] - x0) * math.sin(yaw0)
           + (data.qpos[1] - y0) * math.cos(yaw0))
    aim = yaw0 - math.atan(LINE_GAIN * off)
    err = math.atan2(math.sin(aim - yaw), math.cos(aim - yaw))
    return max(-MAX_STEER, min(MAX_STEER, STEER_GAIN * err))


def drive(data, actuators, elapsed, step_time):
    """Set all 24 servo targets for this instant."""
    growth = min(1.0, elapsed / EASE_IN) if EASE_IN > 0 else 1.0
    growth = growth * growth * (3.0 - 2.0 * growth)
    cycle = (elapsed / step_time) % 1.0
    steer = steering(data, elapsed)
    for leg, side, offset, scale in LEGS:
        # Left legs are side +1: a left turn needs the right side to push
        # further and the left side less.
        trim = 1.0 - side * steer
        sweep, lift = leg_targets((cycle + offset) % 1.0,
                                  STEP_REACH * scale * trim * growth)
        # Hip turns anticlockwise for positive, which is backwards for a
        # left leg and forwards for a right one.
        # Servo targets go to MuJoCo in radians.
        data.ctrl[actuators[f"{leg}_hip"]] = math.radians(-side * sweep)
        data.ctrl[actuators[f"{leg}_knee"]] = math.radians(lift * growth)
        data.ctrl[actuators[f"{leg}_ankle"]] = math.radians(
            -lift * ANKLE_FOLLOW * growth)


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
    print("Walk check - spider ripple on the CAD model")
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
    print("gait: spider ripple, front to back, every leg on its own beat")
    for leg, _s, off, sc in sorted(LEGS, key=lambda r: (1 - r[2]) % 1):
        print(f"    {leg}  steps at {(1.0 - off) % 1.0:.0%} of the cycle, "
              f"swings {STEP_REACH * sc:.1f} deg")
    if args.check:
        return check(model, actuators)
    return watch(model, actuators)


if __name__ == "__main__":
    sys.exit(main())
