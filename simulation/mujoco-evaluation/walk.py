#!/usr/bin/env python3
"""
walk.py

Makes the Huntsman spider walk in a straight line.

Run spider.py first so spider.xml exists. This file only drives it.

THE GAIT
    Row by row. The two back legs step together, then the next pair
    forward, then the next, then the front pair, and round again.

    Down each side the legs step in turn from the back forward:

        left    l4  ->  l3  ->  l2  ->  l1
        right   r4  ->  r3  ->  r2  ->  r1

    with the right side running half a cycle behind the left. That is the
    ripple you see travelling up a walking spider. Two feet are in the air
    at any moment and six stay planted.

WHY EVERY LEG SWINGS THE SAME AMOUNT
    The hip turns about a vertical axis, so it swings the foot sideways to
    whichever way the leg points. The middle legs point out to the side,
    so their hips drive the robot almost straight forward. The front and
    back legs are fanned 56 degrees, so only about half their effort
    pushes the robot along.

    The tempting fix is to make those legs swing further to compensate.
    Do not. Swinging them further closes the gap to their neighbours -
    measured, from 53 mm down to 27 mm, which is where the bunching and
    clashing comes from.

    Every leg swings the same amount instead. The outer legs push less
    and hold more, which is what they are for.

    That matters. An earlier version lifted four legs at once, which is a
    perfectly good gait on paper, but on this robot it left too little
    support and the body rocked hard enough to swing the legs into each
    other. Measuring showed the poses themselves were fine - the machine
    was simply losing its balance. Fewer feet in the air at once fixes
    that.

    Within a step each leg does two things at once: the hip sweeps it
    forward or back, and the knee lifts it clear of the ground and sets
    it down again. The lift follows a smooth arc so the foot rises,
    travels, and lands rather than dragging.

RUN
    macOS            mjpython walk.py
    Windows / Linux  python walk.py

    python walk.py --check    walk it headless and report, no viewer

CONTROLS
    SPACE   pause
    R       reset to the start
    S       stand still, no gait
    UP      quicker steps
    DOWN    slower steps
    T       look from above
    F       stop the camera following
    Q       quit
    Mouse   drag to rotate, scroll to zoom
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import mujoco

# =====================================================================
# THE WALK - these are the numbers to play with
# All angles in degrees, times in seconds.
# =====================================================================

# How far the hip swings each way from centre. Kept small on purpose:
# a bigger step closes the gap between neighbouring legs, and the whole
# reason they clash is that the gap runs out.
STEP_REACH = 10.0

# The smallest gap allowed between two neighbouring legs at full swing.
# The legs are about 12 mm thick, but the motors always lag a little
# behind the commanded angle, so the real gap is smaller than the
# commanded one. This leaves generous room for that.
MIN_LEG_GAP = 0.050
STEP_LIFT = 10.0        # how high the knee picks the foot up
# How hard the ankle turns back against the knee.
#
# This matters more than it looks. The tibia already angles down and
# outward, so raising the knee does not lift the foot straight up. It
# swings the foot outward as well, and by more than it lifts: measured,
# a 12 degree knee lift raises the foot 18 mm but throws it 25 mm
# sideways, straight towards the neighbouring leg.
#
# Turning the ankle the other way pulls the foot back in. The ratio that
# leaves a purely vertical lift is the tibia's drop divided by the foot's
# length, which is about 2.6 for this robot. It is worked out from the
# body rather than typed in, so changing the leg dimensions keeps it
# right.
# Derived value is 2.6, which does make the foot lift vertically. But
# tried on the robot it made the overlapping worse, not better, because
# the collisions are between the femur and tibia - segments the ankle
# does not move. Left at the lower value that walked better in practice.
ANKLE_FOLLOW = 0.5

STEP_TIME = 1.8         # seconds for one full cycle (all eight legs
                        # step once)
SWING_SHARE = 0.22      # fraction of the cycle a leg spends in the air.
                        # With eight legs an eighth of a cycle apart, this
                        # keeps at most two off the ground at once.

SPEED_STEP = 0.1        # seconds added or removed per key press
FASTEST, SLOWEST = 0.35, 3.0

# The fan angles come from the body, so the two files cannot drift apart.
try:
    from spider import LEG_FAN
except ImportError:
    LEG_FAN = (56.0, 19.0, -19.0, -56.0)


def largest_safe_step():
    """The biggest hip swing that still keeps the legs apart.

    Two neighbouring legs sit LEG_FAN apart. Swinging each of them by R
    closes that by 2R. The gap left over has to be wide enough, at the
    distance the legs reach, to clear MIN_LEG_GAP.
    """
    try:
        from spider import leg_reach
        reach = leg_reach()
    except ImportError:
        reach = 0.18
    needed = math.degrees(MIN_LEG_GAP / reach)
    worst = None
    for a, b in zip(LEG_FAN, LEG_FAN[1:]):
        allowed = ((a - b) - needed) / 2.0
        worst = allowed if worst is None else min(worst, allowed)
    return max(0.0, worst)


def swing_scale(pair_index):
    """How much of the base swing this pair uses.

    A leg fanned away from sideways only turns cos(fan) of its hip
    rotation into forward travel, so the obvious move is to swing it
    further to compensate - 1 / cos(fan).

    That was tried and it is wrong. The front and rear legs are fanned
    56 degrees, so compensating makes them swing 17.9 degrees against the
    middle legs' 10.6, and the extra sweep eats the gap between
    neighbours. Measured, it drops the tightest clearance from 53 mm to
    27 mm, and the legs bunch and clash.

    So every leg swings the same amount instead. The outer legs
    contribute less push, which is fine - they are mostly there to hold
    the robot up, and real spiders use their outer legs the same way.
    """
    return 1.0


# Row by row. Both legs of a pair step together, then the next pair, and
# so on from the back of the body to the front. Two legs in the air at a
# time, six on the ground. This is the plainest walk there is - get it
# working first, make it look like a spider afterwards.
def _build_legs():
    legs = []
    for position, pair in enumerate((4, 3, 2, 1)):      # back row first
        # A leg lifts when (cycle + offset) wraps past zero, so the offset
        # is the mirror of the moment we want it to step. Getting this
        # backwards runs the sequence in the wrong order.
        lifts_at = position / 4.0
        offset = (1.0 - lifts_at) % 1.0
        for letter, side in (("l", 1.0), ("r", -1.0)):
            legs.append((f"{letter}{pair}", side, offset,
                         swing_scale(pair - 1)))
    return legs


LEGS = _build_legs()

# Never let the step grow past what the geometry allows.
SAFE_STEP = largest_safe_step()
if STEP_REACH > SAFE_STEP:
    print(f"  step reduced from {STEP_REACH:.1f} to {SAFE_STEP:.1f} deg "
          f"so the legs keep {MIN_LEG_GAP * 1000:.0f} mm apart")
    STEP_REACH = SAFE_STEP

# =====================================================================


def ease(x):
    """Smooth start and smooth stop, so the leg does not snap."""
    return x * x * (3.0 - 2.0 * x)


def arc(x):
    """Foot rises, peaks, and comes back down across one swing."""
    return math.sin(math.pi * x)


def leg_targets(phase, reach):
    """Where one leg's three joints should be, at this point in its cycle.

    Returns (sweep, lift) before the left/right sign is applied.
    sweep is positive when the foot is forward of centre.
    """
    if phase < SWING_SHARE:
        # In the air, travelling forward.
        through = phase / SWING_SHARE
        sweep = -reach + 2.0 * reach * ease(through)
        lift = STEP_LIFT * arc(through)
    else:
        # On the ground, pushing the body forward. Even speed, because
        # the foot is not slipping - the body is moving over it.
        through = (phase - SWING_SHARE) / (1.0 - SWING_SHARE)
        sweep = reach - 2.0 * reach * through
        lift = 0.0
    return sweep, lift


EASE_IN = 1.5           # seconds spent growing from standing to full
                        # stride. Without this the very first command is a
                        # step change of nearly 20 degrees, which saturates
                        # every motor at once and flings the legs.


def drive(data, actuators, elapsed, step_time):
    """Set all 24 motor targets for this instant."""
    # Grow the stride smoothly from nothing so the motors are never asked
    # for a jump they cannot make.
    growth = min(1.0, elapsed / EASE_IN) if EASE_IN > 0 else 1.0
    growth = growth * growth * (3.0 - 2.0 * growth)
    cycle = (elapsed / step_time) % 1.0
    for leg, side, offset, scale in LEGS:
        sweep, lift = leg_targets((cycle + offset) % 1.0,
                                  STEP_REACH * scale * growth)
        # Forward is negative hip for a left leg, positive for a right one.
        data.ctrl[actuators[f"{leg}_hip"]] = -side * sweep
        # Positive knee raises a left leg, lowers a right one.
        data.ctrl[actuators[f"{leg}_knee"]] = side * lift * growth
        data.ctrl[actuators[f"{leg}_ankle"]] = (
            -side * lift * ANKLE_FOLLOW * growth)


def stand(data):
    data.ctrl[:] = 0.0


def load():
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "spider.xml")
    if not os.path.exists(path):
        print("spider.xml not found. Run this first:")
        print("    python3 spider.py --save")
        sys.exit(1)
    try:
        model = mujoco.MjModel.from_xml_path(path)
    except ValueError as exc:
        print(f"MuJoCo rejected spider.xml:\n  {exc}")
        sys.exit(1)

    actuators = {}
    for leg, _side, _off, _scale in LEGS:
        for joint in ("hip", "knee", "ankle"):
            name = f"{leg}_{joint}"
            idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if idx == -1:
                print(f"motor {name} is missing from the model")
                sys.exit(1)
            actuators[name] = idx
    return model, actuators


def tilt_of(quat):
    """How far the body has tipped from level, in degrees."""
    _w, x, y, _z = quat
    upright = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, upright))))


# =====================================================================
# HEADLESS CHECK
# =====================================================================

def check(model, actuators, seconds=10.0):
    data = mujoco.MjData(model)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body")

    geom_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
                  for i in range(model.ngeom)]
    first_contact = None
    contact_frames = 0

    # Let it settle on its feet before asking it to walk.
    for _ in range(600):
        mujoco.mj_step(model, data)
    settled = data.xpos[body].copy()

    start = None
    steps = int(seconds / model.opt.timestep)
    for i in range(steps):
        drive(data, actuators, i * model.opt.timestep, STEP_TIME)
        mujoco.mj_step(model, data)
        if start is None:
            start = data.xpos[body].copy()

        # Watch for legs touching each other, and remember the first time.
        touched = False
        for c in range(data.ncon):
            one = geom_names[data.contact[c].geom1]
            two = geom_names[data.contact[c].geom2]
            if one == "ground" or two == "ground":
                continue
            touched = True
            if first_contact is None:
                first_contact = {
                    "time": i * model.opt.timestep,
                    "one": one, "two": two,
                    "height": float(data.xpos[body][2]),
                    "tilt": tilt_of(data.xquat[body]),
                    "overlap": -float(data.contact[c].dist) * 1000.0,
                }
        if touched:
            contact_frames += 1

    end = data.xpos[body].copy()
    forward = end[0] - start[0]
    sideways = end[1] - start[1]
    tilt = tilt_of(data.xquat[body])

    print()
    print("Walk check")
    print("=" * 52)
    print(f"  stood at            {settled[2]:.4f} m")
    print(f"  walked forward      {forward:+.3f} m in {seconds:.0f} s")
    print(f"  wandered sideways   {sideways:+.3f} m")
    print(f"  speed               {forward / seconds:+.3f} m/s")
    print(f"  body height at end  {end[2]:.4f} m")
    print(f"  tipped              {tilt:.1f} deg from level")
    print()
    print(f"  legs touched each other on {contact_frames} of {steps} frames")
    if first_contact is None:
        print("  NO LEG EVER TOUCHED ANOTHER LEG.")
    else:
        f = first_contact
        print(f"  FIRST self contact at {f['time']:.3f} s: "
              f"{f['one']} against {f['two']}")
        print(f"    body was {f['height']:.4f} m high, "
              f"tilted {f['tilt']:.1f} deg, overlap {f['overlap']:.1f} mm")
        if f["tilt"] < 5.0:
            print("    the body was still level, so the legs were driven")
            print("    together rather than shaken together")
        else:
            print("    the body was already tipping, so balance came first")
    print()

    if end[2] < 0.04 or tilt > 40:
        print("  IT FELL OVER.")
        print("  try smaller STEP_REACH and STEP_LIFT, or a longer STEP_TIME")
    elif abs(forward) < 0.02:
        print("  IT IS NOT GOING ANYWHERE. The feet are slipping or not")
        print("  gripping. try a larger STEP_REACH, or more foot friction")
        print("  in spider.py")
    elif forward < 0:
        print("  IT IS WALKING BACKWARDS. A direction sign is flipped in")
        print("  drive() - swap the sign on the hip target")
    elif abs(sideways) > abs(forward) * 0.4:
        print("  IT IS VEERING OFF COURSE. The two sides are not matching -")
        print("  check the left and right signs in drive()")
    else:
        print("  IT IS WALKING FORWARD IN A STRAIGHT LINE.")
    print("=" * 52)
    return 0


# =====================================================================
# VIEWER
# =====================================================================

def watch(model, actuators):
    import time
    import mujoco.viewer

    data = mujoco.MjData(model)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body")

    state = {"paused": False, "standing": False, "above": False,
             "follow": True, "step_time": STEP_TIME, "live": True,
             "t0": time.time(), "x0": None, "ticks": 0,
             "show_contacts": False, "dump": False}

    geom_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
                  for i in range(model.ngeom)]

    def leg_contacts():
        """Which pairs of parts touch right now, ignoring the floor."""
        found = []
        for c in range(data.ncon):
            con = data.contact[c]
            one, two = geom_names[con.geom1], geom_names[con.geom2]
            if one == "ground" or two == "ground":
                continue
            found.append((one, two, -float(con.dist) * 1000.0, con.pos.copy()))
        return found

    def dump_now(elapsed):
        """Print everything about this instant."""
        touching = leg_contacts()
        pos = data.xpos[body]
        cycle = (elapsed / state["step_time"]) % 1.0
        airborne = [leg for leg, _s, off, _sc in LEGS
                    if ((cycle + off) % 1.0) < SWING_SHARE]
        print()
        print("  " + "-" * 64)
        print(f"  SNAPSHOT at {elapsed:.2f} s")
        print(f"    body ({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:.3f}), "
              f"tilted {tilt_of(data.xquat[body]):.1f} deg")
        print(f"    {cycle:.0%} through the cycle, in the air: "
              f"{' '.join(airborne) if airborne else 'none'}")
        if not touching:
            print("    nothing touching except the floor")
        else:
            print(f"    {len(touching)} contact(s):")
            for one, two, depth, where in touching:
                print(f"      {one} against {two}, {depth:.2f} mm deep, at "
                      f"({where[0]:+.3f}, {where[1]:+.3f}, {where[2]:+.3f})")
        print("  " + "-" * 64)
        print()

    def restart():
        mujoco.mj_resetData(model, data)
        for _ in range(400):
            mujoco.mj_step(model, data)
        state["t0"] = time.time()
        state["x0"] = None
        print("\n  reset")

    def on_key(code):
        if code == 32:
            state["paused"] = not state["paused"]
            print("\n  " + ("paused" if state["paused"] else "walking"))
        elif code == 82:
            restart()
        elif code == 83:
            state["standing"] = not state["standing"]
            print("\n  " + ("standing still" if state["standing"] else "walking"))
        elif code == 84:
            state["above"] = not state["above"]
        elif code == 70:
            state["follow"] = not state["follow"]
        elif code == 265:
            state["step_time"] = max(FASTEST, state["step_time"] - SPEED_STEP)
            print(f"\n  step time {state['step_time']:.2f}s")
        elif code == 264:
            state["step_time"] = min(SLOWEST, state["step_time"] + SPEED_STEP)
            print(f"\n  step time {state['step_time']:.2f}s")
        elif code == 67:                    # C
            state["show_contacts"] = not state["show_contacts"]
            print(f"\n  contact markers "
                  f"{'on' if state['show_contacts'] else 'off'}")
        elif code == 88:                    # X
            state["dump"] = True
        elif code in (81, 256):
            state["live"] = False

    print()
    print("  SPACE pause   S stand   R reset   T above   F follow")
    print("  C contact markers on/off     X snapshot of what is touching")
    print("  UP quicker    DOWN slower         Q quit")
    print("  drag to rotate, scroll to zoom")
    print()

    with mujoco.viewer.launch_passive(
            model, data, key_callback=on_key) as viewer:

        # Contact dots and force arrows are off by default. Press C to
        # turn them on when you want to see what is touching.
        viewer.cam.lookat[:] = [0.0, 0.0, 0.06]
        viewer.cam.distance = 1.1
        viewer.cam.elevation = -18.0
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
                viewer.cam.lookat[0] = pos[0]
                viewer.cam.lookat[1] = pos[1]
                viewer.cam.lookat[2] = 0.0 if state["above"] else pos[2]

            if state["above"] != was_above:
                if state["above"]:
                    viewer.cam.elevation, viewer.cam.azimuth = -89.0, 0.0
                    viewer.cam.distance = 1.6
                else:
                    viewer.cam.elevation, viewer.cam.azimuth = -18.0, 135.0
                    viewer.cam.distance = 1.1
                was_above = state["above"]

            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = (
                state["show_contacts"])
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = (
                state["show_contacts"])

            if state["dump"]:
                dump_now(elapsed)
                state["dump"] = False

            state["ticks"] += 1
            if state["ticks"] % 60 == 0:
                gone = pos[0] - state["x0"]
                touching = leg_contacts()
                if touching:
                    worst = max(touching, key=lambda c: c[2])
                    note = (f"  TOUCHING {len(touching)}: "
                            f"{worst[0]}/{worst[1]} {worst[2]:.1f}mm")
                else:
                    note = "  clear"
                print(f"\r  {elapsed:5.1f}s  fwd {gone:+6.3f}m  "
                      f"drift {pos[1]:+6.3f}m  h {pos[2]:5.3f}m"
                      f"{note:<46}", end="", flush=True)

            viewer.sync()
            time.sleep(1.0 / 240.0)

    print("\n")
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true",
                        help="walk it headless and report")
    args = parser.parse_args()

    model, actuators = load()
    print(f"loaded spider.xml - {model.nu} motors, "
          f"{sum(model.body_mass):.3f} kg")
    print("gait: row by row, back pair first, two legs up at a time")
    for leg, _s, off, sc in LEGS:
        print(f"    {leg}  steps at {(1.0 - off) % 1.0:.0%} of the cycle, "
              f"swings {STEP_REACH * sc:.1f} deg")

    if args.check:
        return check(model, actuators)
    return watch(model, actuators)


if __name__ == "__main__":
    sys.exit(main())
