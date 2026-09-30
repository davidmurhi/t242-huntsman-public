#!/usr/bin/env python3
"""
search.py

Stops guessing and searches.

Tries many combinations of body shape and gait, runs each one properly
with full physics, and reports which ones walk without the legs touching.

It varies four things:

    leg fan spread   how far the legs splay forward and back
    step reach       how far the hips swing
    step lift        how high the knees raise the feet
    ankle follow     how hard the ankle turns back against the knee

For each one it rebuilds the robot, walks it, and counts leg-to-leg
contacts. Nothing is assumed about which of these matters most.

    python3 search.py            the full search  (a few minutes)
    python3 search.py --quick    a smaller one    (under a minute)

Writes the winning settings to the screen. Nothing is changed on disk
except a scratch model file.
"""

from __future__ import annotations

import argparse
import importlib
import itertools
import math
import os
import sys

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
SCRATCH = os.path.join(HERE, "_search_model.xml")
LEGS = ["l1", "r1", "l2", "r2", "l3", "r3", "l4", "r4"]
SECONDS = 5.0

FULL = {
    "fan": [(40, 14, -14, -40), (48, 16, -16, -48),
            (56, 19, -19, -56), (66, 23, -23, -66)],
    "reach": [7.0, 10.0, 13.0],
    "lift": [8.0, 14.0, 20.0],
    "ankle": [0.5, 1.5, 2.6],
}
QUICK = {
    "fan": [(48, 16, -16, -48), (66, 23, -23, -66)],
    "reach": [8.0, 12.0],
    "lift": [10.0, 18.0],
    "ankle": [0.5, 2.6],
}


def part_of(name):
    if name is None or name == "ground":
        return "ground"
    if name.startswith("body") or name.startswith("front_marker"):
        return "body"
    for leg in LEGS:
        if name.startswith(leg + "_"):
            return leg
    return name


def ease(x):
    return x * x * (3.0 - 2.0 * x)


def build(spider, fan):
    """Regenerate the model with a different leg fan."""
    spider.LEG_FAN = tuple(float(f) for f in fan)
    xml = spider.build_model()
    with open(SCRATCH, "w", encoding="utf-8") as handle:
        handle.write(xml)
    return mujoco.MjModel.from_xml_path(SCRATCH)


def run(model, fan, reach, lift, ankle, step_time, swing, ease_in=1.5):
    data = mujoco.MjData(model)
    gname = {i: mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
             for i in range(model.ngeom)}
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "body")

    act = {}
    for leg in LEGS:
        for joint in ("hip", "knee", "ankle"):
            act[f"{leg}_{joint}"] = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{leg}_{joint}")

    # leg schedule: each side ripples back to front, right half a cycle behind
    schedule = []
    for position, pair in enumerate((4, 3, 2, 1)):
        for letter, side, side_off in (("l", 1.0, 0.0), ("r", -1.0, 0.5)):
            scale = 1.0 / math.cos(math.radians(fan[pair - 1]))
            schedule.append((f"{letter}{pair}", side,
                             (position / 4.0 + side_off) % 1.0, scale))

    for _ in range(500):
        mujoco.mj_step(model, data)
    start = data.xpos[body].copy()

    touches = 0
    worst_tilt = 0.0
    steps = int(SECONDS / model.opt.timestep)

    for i in range(steps):
        t = i * model.opt.timestep
        grow = min(1.0, t / ease_in)
        grow = grow * grow * (3.0 - 2.0 * grow)
        cycle = (t / step_time) % 1.0

        for leg, side, offset, scale in schedule:
            phase = (cycle + offset) % 1.0
            r = reach * scale * grow
            if phase < swing:
                through = phase / swing
                sweep = -r + 2.0 * r * ease(through)
                up = lift * math.sin(math.pi * through) * grow
            else:
                through = (phase - swing) / (1.0 - swing)
                sweep = r - 2.0 * r * through
                up = 0.0
            data.ctrl[act[f"{leg}_hip"]] = -side * sweep
            data.ctrl[act[f"{leg}_knee"]] = side * up
            data.ctrl[act[f"{leg}_ankle"]] = -side * up * ankle

        mujoco.mj_step(model, data)

        for c in range(data.ncon):
            con = data.contact[c]
            a = part_of(gname.get(con.geom1))
            b = part_of(gname.get(con.geom2))
            if a == "ground" or b == "ground" or a == b:
                continue
            touches += 1

        w, x, y, _z = data.xquat[body]
        worst_tilt = max(worst_tilt, math.degrees(math.acos(
            max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y))))))

    end = data.xpos[body].copy()
    forward = end[0] - start[0]
    return {
        "forward": forward,
        "drift": end[1] - start[1],
        "tilt": worst_tilt,
        "touches": touches,
        "height": float(end[2]),
        "fell": end[2] < 0.05 or worst_tilt > 45,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args()

    sys.path.insert(0, HERE)
    try:
        import spider
        import walk
    except ImportError as exc:
        print(f"need spider.py and walk.py beside this file ({exc})")
        return 1

    original_fan = spider.LEG_FAN
    grid = QUICK if args.quick else FULL
    combos = list(itertools.product(grid["fan"], grid["reach"],
                                    grid["lift"], grid["ankle"]))

    print()
    print(f"Searching {len(combos)} combinations, {SECONDS:.0f} s each.")
    print(f"Currently: fan {tuple(int(f) for f in original_fan)}, "
          f"reach {walk.STEP_REACH:.0f}, lift {walk.STEP_LIFT:.0f}, "
          f"ankle {walk.ANKLE_FOLLOW:.1f}")
    print()
    print(f"{'fan spread':>11} {'reach':>6} {'lift':>5} {'ankle':>6} "
          f"{'forward':>9} {'drift':>8} {'tilt':>6} {'touches':>8}")
    print("-" * 74)

    results = []
    cached = {}
    for fan, reach, lift, ankle in combos:
        if fan not in cached:
            cached[fan] = build(spider, fan)
        model = cached[fan]
        r = run(model, fan, reach, lift, ankle,
                walk.STEP_TIME, walk.SWING_SHARE)
        r.update(fan=fan, reach=reach, lift=lift, ankle=ankle)
        results.append(r)
        print(f"{str(int(fan[0])) + '/' + str(int(fan[1])):>11} "
              f"{reach:6.0f} {lift:5.0f} {ankle:6.1f} "
              f"{r['forward']:+8.3f}m {r['drift']:+7.3f}m "
              f"{r['tilt']:5.1f}d {r['touches']:8d}")

    spider.LEG_FAN = original_fan
    if os.path.exists(SCRATCH):
        os.remove(SCRATCH)

    good = [r for r in results
            if not r["fell"] and r["forward"] > 0.05
            and abs(r["drift"]) < r["forward"] * 0.35]

    print()
    print("=" * 74)
    clean = [r for r in good if r["touches"] == 0]
    if clean:
        best = max(clean, key=lambda r: r["forward"])
        pick, label = best, "NO LEG CONTACT AT ALL"
    elif good:
        best = min(good, key=lambda r: r["touches"])
        pick, label = best, f"LEAST CONTACT ({best['touches']} frames)"
    else:
        print("Nothing walked acceptably. Every setting fell, stalled or")
        print("veered. The leg layout itself may need rethinking.")
        print("=" * 74)
        return 0

    print(label)
    print()
    print(f"  walked {pick['forward']:+.3f} m in {SECONDS:.0f} s, "
          f"drifted {pick['drift']:+.3f} m")
    print(f"  worst tilt {pick['tilt']:.1f} deg, "
          f"ended at {pick['height']:.3f} m")
    print()
    print("  In spider.py:")
    print(f"      LEG_FAN = ({pick['fan'][0]:.1f}, {pick['fan'][1]:.1f}, "
          f"{pick['fan'][2]:.1f}, {pick['fan'][3]:.1f})")
    print()
    print("  In walk.py:")
    print(f"      STEP_REACH   = {pick['reach']:.1f}")
    print(f"      STEP_LIFT    = {pick['lift']:.1f}")
    print(f"      ANKLE_FOLLOW = {pick['ankle']:.1f}")
    print()
    print(f"  {len(clean)} of {len(results)} were completely clean, "
          f"{len(good)} walked acceptably.")
    print("=" * 74)
    print()
    print("Paste this back, including the table.")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
