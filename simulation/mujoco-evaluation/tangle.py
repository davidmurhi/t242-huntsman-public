#!/usr/bin/env python3
"""
tangle.py

Finds out what is actually hitting what.

Runs the walk and watches MuJoCo's own contact detection, so this is
ground truth rather than a guess about the geometry. It reports every
pair of parts that touch each other, how often, and how hard.

Run spider.py and have walk.py beside it, then:

    python3 tangle.py

No viewer, no graphics. Takes about ten seconds.
"""

from __future__ import annotations

import math
import os
import sys
from collections import defaultdict

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "spider.xml")

LEGS = ["l1", "r1", "l2", "r2", "l3", "r3", "l4", "r4"]
SECONDS = 10.0


def part_of(name):
    """Which leg a geom belongs to, or 'body' or 'ground'."""
    if name is None:
        return "?"
    if name in ("ground",):
        return "ground"
    if name.startswith("body") or name.startswith("front_marker"):
        return "body"
    for leg in LEGS:
        if name.startswith(leg + "_"):
            return leg
    return name


def main():
    if not os.path.exists(XML):
        print("spider.xml not found. run: python3 spider.py --save")
        return 1

    try:
        import walk
    except ImportError:
        print("walk.py must be in the same folder")
        return 1

    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)

    actuators = {}
    for leg, _s, _o in walk.LEGS:
        for joint in ("hip", "knee", "ankle"):
            name = f"{leg}_{joint}"
            actuators[name] = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)

    geom_name = {}
    for i in range(model.ngeom):
        geom_name[i] = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)

    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body")

    # Settle first.
    for _ in range(600):
        mujoco.mj_step(model, data)

    print()
    print("Contact report")
    print("=" * 60)
    print(f"  settled height {data.xpos[body_id][2]:.4f} m")
    print(f"  walking for {SECONDS:.0f} s ...")

    hits = defaultdict(lambda: {"count": 0, "peak": 0.0})
    worst_tilt = 0.0
    min_height = 99.0
    steps = int(SECONDS / model.opt.timestep)
    start_x = None

    force = __import__("numpy").zeros(6)

    for i in range(steps):
        walk.drive(data, actuators, i * model.opt.timestep, walk.STEP_TIME)
        mujoco.mj_step(model, data)

        if start_x is None:
            start_x = data.xpos[body_id][0]

        for c in range(data.ncon):
            con = data.contact[c]
            a = part_of(geom_name.get(con.geom1))
            b = part_of(geom_name.get(con.geom2))
            if a == "ground" or b == "ground":
                continue          # feet on the floor is what we want
            pair = tuple(sorted((a, b)))
            mujoco.mj_contactForce(model, data, c, force)
            magnitude = float(abs(force[0]))
            hits[pair]["count"] += 1
            hits[pair]["peak"] = max(hits[pair]["peak"], magnitude)

        w, x, y, _z = data.xquat[body_id]
        tilt = math.degrees(math.acos(
            max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))
        worst_tilt = max(worst_tilt, tilt)
        min_height = min(min_height, data.xpos[body_id][2])

    end = data.xpos[body_id].copy()
    print()
    print(f"  travelled  {end[0] - start_x:+.3f} m forward, "
          f"{end[1]:+.3f} m sideways")
    print(f"  height     ended {end[2]:.4f} m, dipped to {min_height:.4f} m")
    print(f"  tilt       worst {worst_tilt:.1f} deg from level")
    print()

    if not hits:
        print("  NOTHING is touching anything except the floor.")
        print("  The legs are not tangling physically. If it still looks")
        print("  wrong, it is how the legs move, not whether they collide.")
    else:
        print(f"  {len(hits)} pair(s) of parts are touching each other:")
        print()
        print(f"  {'parts':<20} {'frames':>8} {'peak force':>12}")
        print("  " + "-" * 44)
        ranked = sorted(hits.items(),
                        key=lambda kv: kv[1]["count"], reverse=True)
        for (a, b), info in ranked:
            share = 100.0 * info["count"] / steps
            print(f"  {a + ' - ' + b:<20} {info['count']:>8} "
                  f"{info['peak']:>10.2f} N   ({share:.0f}% of the time)")
        print()
        leg_on_leg = [p for p in hits if p[0] in LEGS and p[1] in LEGS]
        leg_on_body = [p for p in hits if "body" in p]
        if leg_on_leg:
            print(f"  LEGS HITTING LEGS: {len(leg_on_leg)} pair(s)")
            print("  -> spread LEG_FAN further apart, or shorten the legs")
        if leg_on_body:
            print(f"  LEGS HITTING THE BODY: {len(leg_on_body)} pair(s)")
            print("  -> the legs are folding under. reduce STEP_LIFT in "
                  "walk.py,")
            print("     or raise FEMUR_ANGLE_UP in spider.py")

    print()
    print("  Also worth knowing:")
    if worst_tilt > 25:
        print(f"  the body tipped {worst_tilt:.0f} degrees - it is rocking")
        print("  badly. try a longer STEP_TIME or smaller STEP_LIFT.")
    if min_height < 0.06:
        print(f"  the body dropped to {min_height:.3f} m - the legs are")
        print("  buckling under the weight. raise the servo torque in "
              "spider.py.")
    if worst_tilt <= 25 and min_height >= 0.06 and not hits:
        print("  it is walking cleanly. nothing to fix here.")
    print("=" * 60)
    print()
    print("  Paste this whole report back.")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
