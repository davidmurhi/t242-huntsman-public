#!/usr/bin/env python3
"""
tune.py

Finds gait settings that actually work, by trying them.

The problem with the current walk is that lifting the knee does not just
raise the foot. Because the tibia angles down and outward, rotating it up
also swings the whole leg further out sideways. A big lift makes each leg
reach roughly 50 mm further than it does at rest, which is what closes the
gap to its neighbour. On top of that a hard gait makes the body rock, and
a rocking body swings the legs into each other.

So instead of guessing, this runs the walk headlessly with many different
combinations of step size, lift height and step speed, and reports which
ones walk straight without the legs touching.

    python3 tune.py            try the standard grid  (about a minute)
    python3 tune.py --quick    fewer combinations     (about 20 seconds)

Needs spider.xml and walk.py beside it.
"""

from __future__ import annotations

import argparse
import itertools
import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "spider.xml")

LEGS = ["l1", "r1", "l2", "r2", "l3", "r3", "l4", "r4"]
TEST_SECONDS = 6.0

FULL_GRID = {
    "reach": [8.0, 11.0, 14.0],
    "lift": [10.0, 14.0, 18.0, 26.0],
    "step_time": [0.9, 1.3, 1.8],
}
QUICK_GRID = {
    "reach": [10.0, 14.0],
    "lift": [12.0, 20.0],
    "step_time": [1.0, 1.6],
}


def ease(x):
    return x * x * (3.0 - 2.0 * x)


def drive(data, act, elapsed, reach, lift_height, step_time, swing_share,
          ankle_follow, legs):
    cycle = (elapsed / step_time) % 1.0
    for leg, side, offset in legs:
        phase = (cycle + offset) % 1.0
        if phase < swing_share:
            through = phase / swing_share
            sweep = -reach + 2.0 * reach * ease(through)
            lift = lift_height * math.sin(math.pi * through)
        else:
            through = (phase - swing_share) / (1.0 - swing_share)
            sweep = reach - 2.0 * reach * through
            lift = 0.0
        data.ctrl[act[f"{leg}_hip"]] = -side * sweep
        data.ctrl[act[f"{leg}_knee"]] = side * lift
        data.ctrl[act[f"{leg}_ankle"]] = -side * lift * ankle_follow


def part_of(name):
    if name is None or name == "ground":
        return "ground"
    if name.startswith("body") or name.startswith("front_marker"):
        return "body"
    for leg in LEGS:
        if name.startswith(leg + "_"):
            return leg
    return name


def trial(model, act, geom_names, body_id, legs, reach, lift, step_time,
          swing_share, ankle_follow):
    """Run one setting. Returns a dict of what happened."""
    data = mujoco.MjData(model)
    for _ in range(500):
        mujoco.mj_step(model, data)

    start = data.xpos[body_id].copy()
    steps = int(TEST_SECONDS / model.opt.timestep)
    touches = 0
    worst_tilt = 0.0
    lowest = 99.0

    for i in range(steps):
        drive(data, act, i * model.opt.timestep, reach, lift, step_time,
              swing_share, ankle_follow, legs)
        mujoco.mj_step(model, data)

        for c in range(data.ncon):
            con = data.contact[c]
            a = part_of(geom_names.get(con.geom1))
            b = part_of(geom_names.get(con.geom2))
            if a == "ground" or b == "ground":
                continue
            touches += 1

        w, x, y, _z = data.xquat[body_id]
        worst_tilt = max(worst_tilt, math.degrees(math.acos(
            max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y))))))
        lowest = min(lowest, data.xpos[body_id][2])

    end = data.xpos[body_id].copy()
    forward = end[0] - start[0]
    sideways = end[1] - start[1]
    return {
        "forward": forward,
        "sideways": sideways,
        "speed": forward / TEST_SECONDS,
        "tilt": worst_tilt,
        "lowest": lowest,
        "touches": touches,
        "fell": end[2] < 0.05 or worst_tilt > 45,
        "straightness": abs(sideways) / max(abs(forward), 1e-6),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args()

    if not os.path.exists(XML):
        print("spider.xml not found. run: python3 spider.py --save")
        return 1
    try:
        import walk
    except ImportError:
        print("walk.py must be beside this file")
        return 1

    model = mujoco.MjModel.from_xml_path(XML)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body")
    geom_names = {i: mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
                  for i in range(model.ngeom)}

    act = {}
    for leg, _s, _o in walk.LEGS:
        for joint in ("hip", "knee", "ankle"):
            name = f"{leg}_{joint}"
            act[name] = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)

    grid = QUICK_GRID if args.quick else FULL_GRID
    combos = list(itertools.product(
        grid["reach"], grid["lift"], grid["step_time"]))

    print()
    print(f"Trying {len(combos)} gait settings, {TEST_SECONDS:.0f} s each.")
    print("Current setting is in walk.py: "
          f"reach {walk.STEP_REACH:.0f}, lift {walk.STEP_LIFT:.0f}, "
          f"step time {walk.STEP_TIME:.1f}")
    print()
    print(f"{'reach':>6} {'lift':>5} {'step':>6} {'forward':>9} "
          f"{'drift':>8} {'tilt':>6} {'touches':>8}  verdict")
    print("-" * 72)

    results = []
    for reach, lift, step_time in combos:
        r = trial(model, act, geom_names, body_id, walk.LEGS,
                  reach, lift, step_time, walk.SWING_SHARE,
                  walk.ANKLE_FOLLOW)
        r.update(reach=reach, lift=lift, step_time=step_time)
        results.append(r)

        if r["fell"]:
            verdict = "fell over"
        elif r["touches"] > 0:
            verdict = f"legs touching"
        elif abs(r["forward"]) < 0.05:
            verdict = "not moving"
        elif r["forward"] < 0:
            verdict = "backwards"
        elif r["straightness"] > 0.25:
            verdict = "veers"
        else:
            verdict = "CLEAN"

        print(f"{reach:6.0f} {lift:5.0f} {step_time:6.1f} "
              f"{r['forward']:+8.3f}m {r['sideways']:+7.3f}m "
              f"{r['tilt']:5.1f}d {r['touches']:8d}  {verdict}")

    # Pick the best: no touching, no falling, straight, then fastest.
    clean = [r for r in results
             if r["touches"] == 0 and not r["fell"]
             and r["forward"] > 0.05 and r["straightness"] <= 0.25]

    print()
    print("=" * 72)
    if clean:
        best = max(clean, key=lambda r: r["speed"])
        print("BEST SETTING FOUND")
        print()
        print(f"  walked {best['forward']:.3f} m in {TEST_SECONDS:.0f} s "
              f"({best['speed']:.3f} m/s)")
        print(f"  drifted {best['sideways']:+.3f} m sideways")
        print(f"  tilted at most {best['tilt']:.1f} degrees")
        print(f"  no legs touched each other")
        print()
        print("  Put these in walk.py:")
        print()
        print(f"      STEP_REACH = {best['reach']:.1f}")
        print(f"      STEP_LIFT  = {best['lift']:.1f}")
        print(f"      STEP_TIME  = {best['step_time']:.1f}")
        print()
        print(f"  {len(clean)} of {len(results)} settings were clean.")
    else:
        touching = [r for r in results if not r["fell"] and r["touches"] > 0]
        print("NO SETTING WAS COMPLETELY CLEAN.")
        print()
        if touching:
            least = min(touching, key=lambda r: r["touches"])
            print(f"  The closest was reach {least['reach']:.0f}, "
                  f"lift {least['lift']:.0f}, step {least['step_time']:.1f}")
            print(f"  with {least['touches']} contacts "
                  f"(the current setting had far more).")
            print()
        print("  This means the legs cannot avoid each other at any gait")
        print("  setting, so the body itself needs changing. In spider.py,")
        print("  try one of:")
        print("      LEG_FAN     spread wider, e.g. (65, 22, -22, -65)")
        print("      BODY_LENGTH longer, so the legs mount further apart")
        print("      TIBIA_LENGTH shorter, so the legs reach less far")
        print("  then run spider.py --save and this again.")
    print("=" * 72)
    print()
    print("Paste this whole table back.")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
