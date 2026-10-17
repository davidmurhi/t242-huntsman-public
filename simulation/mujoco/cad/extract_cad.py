#!/usr/bin/env python3
"""
extract_cad.py

Turns V Source's Inventor CAD into huntsman_cad.json, which spider.py
builds the simulation from. Run it again whenever Ronan sends new CAD;
nothing about the robot's shape is typed in by hand.

    python3 cad/extract_cad.py path/to/Huntsman/     (folder of .ipt/.iam)

What it does
    1. Reads every part's solid-model vertices (cm).
    2. Reads where each part sits in 'mounted servo', 'full leg' and 'bot'.
    3. Composes those placements, so every part of every leg is in one
       frame.
    4. Finds the three joint axes from the servo horn frames.
    5. Converts to the simulator's frame (metres, x forward towards the
       battery, y left, z up) with the origin at the centre of the eight
       hip axes, level with the top of the body plate.
    6. Keeps only each part's convex-hull vertices, which is all MuJoCo
       needs for a collision mesh.

Parts the assembly references but the CAD pack did not include
(case2, rear rail v2, Makitabattery) are reported, not invented.
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import inventor  # noqa: E402

# Weights from Ronan's 'Spider Part Weight.xlsx' (15 Oct 2026), grams.
MASS_G = {
    "body": 180, "Battery LxT 18V v7": 671, "acrylic leg": 27,
    "9imod 55kg": 75, "Servo Bracket - U Shaped bis": 22,
    "Multi Purpose Bracket": 15, "Servo Mount - Horn Frame": 0,
    "payload": 867,
}

# Sheet-metal parts are open frames, and a convex hull would fill them in
# solid. Each is split into its plates, in the part's own coordinates (cm),
# using the plate planes found in the part files.
PIECES = {
    "Servo Bracket - U Shaped bis": [            # base plate and two arms
        lambda p: p[:, 1] <= 0.001,
        lambda p: p[:, 2] >= -0.301,
        lambda p: p[:, 2] <= -5.299,
    ],
    "Multi Purpose Bracket": [                   # spine and two flanges
        lambda p: p[:, 1] <= 0.301,
        lambda p: (p[:, 2] >= 2.95) & (p[:, 2] <= 3.40),
        lambda p: p[:, 2] <= 0.251,
    ],
}


# Assembly frame (Inventor): x along the body, y up, z across. Simulator
# frame: x forward, y left, z up.
TO_SIM = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)


def hull(points):
    try:
        from scipy.spatial import ConvexHull
        return points[ConvexHull(points).vertices]
    except Exception:
        return points


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    folder = sys.argv[1]
    f = lambda name: os.path.join(folder, name)          # noqa: E731

    parts = {}
    for fn in sorted(os.listdir(folder)):
        if fn.endswith(".ipt"):
            parts[fn[:-4]] = inventor.part_vertices(f(fn))

    def placed(asm):
        names = inventor.occurrence_names(f(asm))
        mats = inventor.occurrence_placements(f(asm))
        names = names[-len(mats):]
        return list(zip(names, mats))

    mounted = placed("mounted servo.iam")
    leg = placed("full leg.iam")
    bot = placed("bot.iam")

    # Flatten the leg: sub-assembly 'mounted servo' expands to its parts.
    def flatten(occ):
        out = []
        for name, M in occ:
            base = name.rsplit(":", 1)[0]
            if base == "mounted servo":
                tag = name.replace(" ", "_")
                for n2, M2 in mounted:
                    out.append((f"{tag}/{n2.rsplit(':', 1)[0]}", M @ M2))
            else:
                out.append((name.rsplit(":", 1)[0] + "#" + name.rsplit(":", 1)[1],
                            M))
        return out
    leg_parts = flatten(leg)

    def part_of(key):
        return key.split("/")[-1].split("#")[0]

    # Which link each leg part belongs to, from how the servos are built:
    #   hip servo body is fixed to the chassis; its horn frame turns with
    #   the leg. The knee servo ('mounted servo:1') rides the hip horn;
    #   its horn frame drives the U-bracket femur. The ankle servo
    #   ('mounted servo:2') sits at the femur's end; its horn frame drives
    #   the acrylic lower leg.
    link_of = {
        "9imod 55kg#1": "body",
        "Servo Mount - Horn Frame#1": "coxa",
        "mounted_servo:1/9imod 55kg": "coxa",
        "mounted_servo:1/Multi Purpose Bracket": "coxa",
        "mounted_servo:1/Servo Mount - Horn Frame": "femur",
        "Servo Bracket - U Shaped bis#1": "femur",
        "Servo Bracket - U Shaped bis#2": "femur",
        "mounted_servo:2/9imod 55kg": "femur",
        "mounted_servo:2/Multi Purpose Bracket": "femur",
        "mounted_servo:2/Servo Mount - Horn Frame": "tibia",
        "acrylic leg#1": "tibia",
    }
    missing = [k for k, _ in leg_parts if k not in link_of]
    if missing:
        raise SystemExit(f"unexpected parts in full leg: {missing}")

    lp = dict(leg_parts)
    # Horn frames: origin on the servo output shaft, shaft along local y.
    joints_leg = {
        "hip": lp["Servo Mount - Horn Frame#1"],
        "knee": lp["mounted_servo:1/Servo Mount - Horn Frame"],
        "ankle": lp["mounted_servo:2/Servo Mount - Horn Frame"],
    }

    # Body-level placements.
    botd = {n.rsplit(":", 1)[0] + "#" + n.rsplit(":", 1)[1]: M for n, M in bot}
    legs = [(k, M) for k, M in botd.items() if k.startswith("full leg#")]
    body_M = botd["body#1"]

    # Simulator origin: centre of the hip axes, top of the plate.
    hip_world = np.array([(M @ joints_leg["hip"])[:3, 3] for _, M in legs])
    plate = (parts["body"] @ body_M[:3, :3].T) + body_M[:3, 3]
    origin = np.array([hip_world[:, 0].mean(), plate[:, 1].max(),
                       hip_world[:, 2].mean()])

    def to_sim(pts_cm):
        return ((np.asarray(pts_cm) - origin) @ TO_SIM.T) / 100.0

    def dir_sim(v):
        return TO_SIM @ np.asarray(v)

    out = {"source": "V Source CAD, Huntsman.zip (15 Oct 2026)",
           "units": "metres, x forward (battery end), y left, z up",
           "parts_missing_from_pack": ["case2", "rear rail v2",
                                       "Makitabattery"],
           "body": [], "legs": []}

    def add(lst, key, base, M):
        local = parts[base]
        groups = ([local[sel(local)] for sel in PIECES[base]]
                  if base in PIECES else [local])
        groups = [g for g in groups if len(g) >= 4]
        mass = MASS_G.get(base, 0) / 1000.0
        for k, g in enumerate(groups):
            world = g @ M[:3, :3].T + M[:3, 3]
            lst.append({"name": f"{key}.{k}" if len(groups) > 1 else key,
                        "part": base, "mass": mass / len(groups),
                        "vertices": np.round(hull(to_sim(world)), 5).tolist()})

    add(out["body"], "plate", "body", body_M)
    add(out["body"], "battery", "Battery LxT 18V v7",
        botd["Battery LxT 18V v7#1"])
    for slot, (name, M) in enumerate(legs):
        add(out["body"], f"hip_servo_{slot}", "9imod 55kg",
            M @ lp["9imod 55kg#1"])

    # Unsupplied parts: keep their placements so they can be added later.
    out["placements_without_geometry"] = {
        k: to_sim(M[:3, 3][None, :])[0].round(4).tolist()
        for k, M in botd.items()
        if k.split("#")[0] in ("case2", "rear rail v2", "Makitabattery")}

    for name, M in legs:
        leg = {"cad_name": name, "links": {"coxa": [], "femur": [],
                                             "tibia": []}, "joints": {}}
        for key, Mp in leg_parts:
            link = link_of[key]
            if link == "body":
                continue
            add(leg["links"][link], key, part_of(key), M @ Mp)
        for j, J in joints_leg.items():
            W = M @ J
            leg["joints"][j] = {
                "pivot": to_sim(W[:3, 3][None, :])[0].round(5).tolist(),
                "axis": np.round(dir_sim(W[:3, 1]), 5).tolist()}
        # Foot tip: the acrylic leg's farthest point from the ankle axis.
        tib = np.array([v for p in leg["links"]["tibia"]
                        if p["part"] == "acrylic leg" for v in p["vertices"]])
        ank = np.array(leg["joints"]["ankle"]["pivot"])
        tip = tib[np.argmax(np.linalg.norm((tib - ank)[:, :2], axis=1))]
        leg["tip"] = np.round(tip, 5).tolist()
        out["legs"].append(leg)

    path = os.path.join(HERE, "huntsman_cad.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=1)
    total = (sum(p["mass"] for p in out["body"])
             + sum(p["mass"] for l in out["legs"]
                   for ps in l["links"].values() for p in ps))
    print(f"wrote {os.path.relpath(path)}")
    print(f"  parts read     {len(parts)}  ({', '.join(sorted(parts))})")
    print(f"  legs           {len(out['legs'])}")
    print(f"  mass in parts  {total:.3f} kg  (+ {MASS_G['payload'] / 1000:.3f}"
          f" kg payload, whose CAD was not in the pack)")
    print(f"  not in pack    {', '.join(out['parts_missing_from_pack'])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
