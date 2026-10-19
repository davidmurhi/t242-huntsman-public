#!/usr/bin/env python3
"""
terrain.py

The ground the Huntsman trains and is tested on. Each terrain is the
robot from spider.xml standing on a height map instead of a flat floor,
so the same model, gait and controller work on every one of them.

    flat      the flat floor (spider.xml itself)
    rubble    scattered rocks and debris up to 80 mm high
    slope     a 15-degree ramp up, a plateau and a ramp down
    steps     a staircase of 50 mm steps up and back down
    (at full difficulty; --difficulty 0..1 makes them gentler)
    mixed     a patchwork of all three, different every seed

These are the places Craig said his own controller fails: slopes, steps
and rocks. The robot starts on a flat pad, with the terrain beginning
0.6 m ahead of it.

    python terrain.py rubble            write spider_rubble.xml and report
    python controller.py --terrain rubble     drive it over the rubble

Every terrain is generated from a seed, so a training run can draw a
new one each episode (huntsman_env.py) and a test can repeat one exactly.
"""

from __future__ import annotations

import argparse
import os
import re

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TERRAINS = ("flat", "rubble", "slope", "steps", "mixed")

LENGTH, WIDTH = 10.0, 4.0     # m, the field (x from -2 to +8)
CELL = 0.025                  # m per height-map cell
START_PAD = 0.6               # m of flat ground in front of the robot
MAX_RISE = 0.25               # m, top of the height map's range


def _grid():
    nx, ny = int(LENGTH / CELL), int(WIDTH / CELL)
    x = np.linspace(-2.0, LENGTH - 2.0, nx)
    y = np.linspace(-WIDTH / 2, WIDTH / 2, ny)
    return np.meshgrid(x, y)                 # rows along y, columns along x


def heights(kind: str, seed: int = 0, difficulty: float = 1.0) -> np.ndarray:
    """Height (m) of every cell, rows along y, columns along x.

    difficulty 0..1 scales how rough it is (for a training curriculum):
    rocks 20 -> 80 mm, steps 15 -> 50 mm, slope 5 -> 15 degrees."""
    d = float(np.clip(difficulty, 0.0, 1.0))
    rng = np.random.default_rng(seed)
    X, Y = _grid()
    h = np.zeros_like(X)
    ahead = X > START_PAD

    if kind == "rubble":
        for _ in range(420):
            cx = rng.uniform(START_PAD, LENGTH - 2.0)
            cy = rng.uniform(-WIDTH / 2, WIDTH / 2)
            r = rng.uniform(0.02, 0.07)
            top = rng.uniform(0.008, 0.020 + 0.060 * d)
            d = np.hypot(X - cx, (Y - cy) * rng.uniform(0.6, 1.6))
            h = np.maximum(h, top * np.clip(1 - (d / r) ** 2, 0, None) ** 0.5)
        h += rng.normal(0, 0.002, h.shape) * ahead
    elif kind == "slope":
        up = np.tan(np.radians(5.0 + 10.0 * d))
        x = X - START_PAD
        h = np.clip(x * up, 0, 1.2 * up)                       # 1.2 m up
        h -= np.clip((x - 2.2) * up, 0, 1.2 * up)              # 1 m flat, down
    elif kind == "steps":
        rise, tread, n = 0.015 + 0.035 * d, 0.30, 5
        k = np.floor((X - START_PAD) / tread)
        up = np.clip(k + 1, 0, n)
        down = np.clip(k + 1 - (n + 2), 0, n)
        h = (up - down) * rise
    elif kind == "mixed":
        parts = {k: heights(k, seed + i + 1, d) for i, k in
                 enumerate(("rubble", "slope", "steps"))}
        patch = rng.integers(0, 3, size=(4, 8))
        iy = np.clip(((Y + WIDTH / 2) / WIDTH * 4).astype(int), 0, 3)
        ix = np.clip(((X + 2.0) / LENGTH * 8).astype(int), 0, 7)
        pick = patch[iy, ix]
        h = np.choose(pick, [parts["rubble"], parts["slope"], parts["steps"]])
    h = np.where(ahead, h, 0.0)
    return np.clip(h, 0.0, MAX_RISE)


def model_xml(kind: str = "flat", seed: int = 0, base: str | None = None,
              difficulty: float = 1.0) -> str:
    """spider.xml with its floor swapped for the chosen terrain."""
    with open(base or os.path.join(HERE, "spider.xml"), encoding="utf-8") as fh:
        xml = fh.read()
    if kind == "flat":
        return xml
    if kind not in TERRAINS:
        raise ValueError(f"unknown terrain {kind!r}; pick one of {TERRAINS}")
    h = heights(kind, seed, difficulty)
    rows, cols = h.shape
    # MuJoCo rescales a height map so its highest point is 1, so the
    # vertical size must be the real highest point for heights to be true.
    top = max(float(h.max()), 1e-4)
    elev = " ".join(f"{v / top:.4f}" for v in h.ravel())
    asset = (f'    <hfield name="terrain" nrow="{rows}" ncol="{cols}" '
             f'size="{LENGTH / 2} {WIDTH / 2} {top:.5f} 0.05" '
             f'elevation="{elev}"/>\n')
    xml = xml.replace("  </asset>", asset + "  </asset>", 1)
    ground = re.compile(r'<geom name="ground" type="plane"[^>]*/>', re.S)
    floor = ('<geom name="ground" type="hfield" hfield="terrain" '
             f'pos="{LENGTH / 2 - 2.0} 0 0" material="ground" '
             'friction="1.5 0.05 0.001" condim="4"/>')
    xml, n = ground.subn(floor, xml, count=1)
    if n != 1:
        raise RuntimeError("could not find the floor in spider.xml")
    return xml


def write(kind: str, seed: int = 0, difficulty: float = 1.0) -> str:
    """Write spider_<kind>.xml beside spider.xml and return its path."""
    path = os.path.join(HERE, "spider.xml" if kind == "flat"
                        else f"spider_{kind}.xml")
    if kind != "flat":
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(model_xml(kind, seed, difficulty=difficulty))
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("kind", choices=TERRAINS)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--difficulty", type=float, default=1.0)
    args = ap.parse_args()
    path = write(args.kind, args.seed, args.difficulty)
    h = heights(args.kind, args.seed, args.difficulty)
    print(f"wrote {os.path.basename(path)}: {args.kind}, seed {args.seed}, "
          f"highest point {h.max() * 1000:.0f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
