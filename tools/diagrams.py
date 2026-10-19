#!/usr/bin/env python3
"""
diagrams.py

Generates the project's diagrams as SVG into docs/media/diagrams/.
The gait timing chart and leg map are drawn from the code and CAD data
themselves (gait.py, cad/huntsman_cad.json), so they stay true.

    python tools/diagrams.py
"""

from __future__ import annotations

import math
import os
import sys
from html import escape

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
OUT = os.path.join(REPO, "docs", "media", "diagrams")
SIM = os.path.join(REPO, "simulation", "mujoco")

FONT = "Inter, Segoe UI, Helvetica, Arial, sans-serif"
INK, MUTED, LINE, BG, CARD = "#1f2933", "#52606d", "#9aa5b1", "#ffffff", "#f5f7fa"
TEAL, AMBER, BLUE, ROSE, GREEN, VIOLET = ("#0f766e", "#b45309", "#1d4ed8",
                                           "#be123c", "#15803d", "#6d28d9")


class Svg:
    def __init__(self, w, h, title):
        self.w, self.h, self.parts = w, h, []
        self.parts.append(f'<rect width="{w}" height="{h}" rx="14" fill="{BG}"/>')
        self.text(24, 38, title, 20, INK, weight=700)

    def text(self, x, y, s, size=14, fill=INK, anchor="start", weight=400,
             italic=False):
        style = ' font-style="italic"' if italic else ""
        self.parts.append(
            f'<text x="{x:.1f}" y="{y:.1f}" font-family="{FONT}" font-size="{size}" '
            f'font-weight="{weight}" fill="{fill}" text-anchor="{anchor}"{style}>'
            f'{escape(s)}</text>')

    def box(self, x, y, w, h, title, lines=(), colour=TEAL, fill=CARD):
        self.parts.append(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="10" '
            f'fill="{fill}" stroke="{colour}" stroke-width="2"/>')
        self.parts.append(
            f'<rect x="{x}" y="{y}" width="6" height="{h}" rx="3" fill="{colour}"/>')
        self.text(x + 18, y + 26, title, 15, colour, weight=700)
        for i, line in enumerate(lines):
            self.text(x + 18, y + 48 + 19 * i, line, 13, MUTED)

    def arrow(self, x1, y1, x2, y2, label="", colour=LINE, both=False,
              dashed=False, lx=0, ly=-8):
        dash = ' stroke-dasharray="6 5"' if dashed else ""
        start = ' marker-start="url(#a0)"' if both else ""
        self.parts.append(
            f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{colour}" '
            f'stroke-width="2"{dash} marker-end="url(#a1)"{start}/>')
        if label:
            self.text((x1 + x2) / 2 + lx, (y1 + y2) / 2 + ly, label, 12, MUTED,
                      anchor="middle")

    def save(self, name):
        defs = ('<defs><marker id="a1" viewBox="0 0 10 10" refX="9" refY="5" '
                'markerWidth="8" markerHeight="8" orient="auto">'
                f'<path d="M0,0 L10,5 L0,10 z" fill="{LINE}"/></marker>'
                '<marker id="a0" viewBox="0 0 10 10" refX="1" refY="5" '
                'markerWidth="8" markerHeight="8" orient="auto">'
                f'<path d="M10,0 L0,5 L10,10 z" fill="{LINE}"/></marker></defs>')
        svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" '
               f'height="{self.h}" viewBox="0 0 {self.w} {self.h}">{defs}'
               + "".join(self.parts) + "</svg>\n")
        with open(os.path.join(OUT, name), "w", encoding="utf-8") as fh:
            fh.write(svg)
        print("wrote", name)


def system_architecture():
    s = Svg(1100, 520, "System architecture: one drive protocol from simulation to robot")
    s.text(24, 62, "The simulation speaks the same WebSocket protocol the robot's payload will, "
           "so the dashboard and the gait code move to hardware unchanged.", 13, MUTED)
    s.box(30, 100, 260, 130, "Operator dashboard", ["SvelteKit (Tom)", "drive pad, e-stop, gait mode",
                                                    "roll / pitch / heading, battery"], BLUE)
    s.box(30, 330, 260, 150, "Simulation (MuJoCo)", ["built from V Source CAD",
                                                       "controller.py: same server",
                                                       "terrains + Gymnasium env",
                                                       "PPO training"], VIOLET)
    s.box(420, 90, 290, 160, "Raspberry Pi 5 - payload", ["gait controller (gait.py)",
                                                           "learned policy (PPO, residual)",
                                                           "IMU heading hold, telemetry",
                                                           "Python API, WebSocket server"], TEAL)
    s.box(420, 340, 290, 130, "ESP32 - chassis", ["receives 24 joint targets",
                                                   "PWM via two driver banks",
                                                   "(Prab: serial link proven)"], AMBER)
    s.box(830, 340, 240, 130, "24 x DSC55MG servos", ["8 legs x hip, knee, ankle",
                                                       "18 V Makita LXT pack",
                                                       "4.33 kg robot"], ROSE)
    s.box(830, 90, 240, 160, "Sensors", ["IMU (heading, tilt)", "servo positions",
                                          "Phase 2: payload sensors", "(V Source)"], GREEN)
    s.arrow(292, 165, 418, 165, "WebSocket JSON", both=True)
    s.arrow(160, 232, 160, 328, "same protocol", both=True, dashed=True, lx=-56, ly=4)
    s.arrow(292, 380, 418, 230, "trained policy", lx=52, ly=22)
    s.arrow(565, 252, 565, 338, "joint targets", lx=52, ly=4)
    s.arrow(712, 405, 828, 405, "PWM")
    s.arrow(828, 170, 712, 170, "measurements")
    s.save("system_architecture.svg")


def sim_pipeline():
    s = Svg(1100, 300, "Simulation pipeline: from V Source's CAD to a trained policy")
    steps = [("Inventor CAD", ["bot.iam, full leg.iam", "8 part files (.ipt)"], ROSE),
             ("extract_cad.py", ["reads Inventor directly", "shapes, joints, skins"], AMBER),
             ("spider.py", ["MuJoCo model", "masses = weight sheet"], TEAL),
             ("gait.py", ["foot paths + IK", "4 gaits, heading hold"], BLUE),
             ("huntsman_env.py", ["terrains, randomisation", "PPO (train.py)"], VIOLET)]
    w, gap = 190, 22
    for i, (t, lines, c) in enumerate(steps):
        x = 24 + i * (w + gap)
        s.box(x, 90, w, 110, t, lines, c)
        if i:
            s.arrow(x - gap, 145, x - 2, 145)
    s.text(24, 240, "CI re-runs the extraction and checks it reproduces the committed geometry, "
           "then builds the model and walks it headless on every push.", 13, MUTED)
    s.save("simulation_pipeline.svg")


def residual_rl():
    s = Svg(1100, 330, "Learning on rough ground: residual reinforcement learning")
    s.box(30, 90, 230, 110, "Drive command", ["speed, sideways, turn", "from operator / planner"], BLUE)
    s.box(330, 70, 250, 80, "Hand gait (gait.py)", ["walks on flat ground"], TEAL)
    s.box(330, 180, 250, 100, "Policy (PPO network)", ["sees 86 numbers (IMU, joints)",
                                                        "outputs +/-20 deg corrections"], VIOLET)
    s.box(660, 120, 160, 100, "Sum", ["24 servo targets"], AMBER)
    s.box(890, 90, 180, 160, "Robot / MuJoCo", ["rubble, slope, steps", "random friction,",
                                                 "payload, servo strength", "sensor noise"], ROSE)
    s.arrow(262, 130, 328, 110)
    s.arrow(262, 160, 328, 220)
    s.arrow(582, 110, 658, 150)
    s.arrow(582, 230, 658, 190)
    s.arrow(822, 170, 888, 170)
    s.arrow(980, 252, 455, 282, "observation + reward (speed tracking, heading, level, effort)",
            dashed=True, ly=26)
    s.save("residual_rl.svg")


def gait_timing():
    sys.path.insert(0, SIM)
    import gait
    g = gait.GAITS["walk"]
    s = Svg(1100, 520, "Walk gait timing (from gait.py): spider ripple, six feet down")
    s.text(24, 62, f"Cycle {g['cycle']} s. Each bar is when that foot is lifted "
           f"({g['swing']:.0%} of the cycle). Left side front to back, right side 5/8 "
           "of a cycle behind.", 13, MUTED)
    x0, x1, y0, row = 150, 1060, 100, 38
    for k in range(0, 9):
        x = x0 + (x1 - x0) * k / 8
        s.parts.append(f'<line x1="{x}" y1="{y0 - 8}" x2="{x}" y2="{y0 + row * 8}" '
                       f'stroke="#e4e7eb" stroke-width="1"/>')
        s.text(x, y0 + row * 8 + 22, f"{k / 8:.3g}".rstrip("0").rstrip(".") if k else "0",
               12, MUTED, anchor="middle")
    s.text((x0 + x1) / 2, y0 + row * 8 + 44, "share of the gait cycle", 12, MUTED,
           anchor="middle")
    order = ["l1", "l2", "l3", "l4", "r1", "r2", "r3", "r4"]
    for i, leg in enumerate(order):
        y = y0 + row * i
        colour = TEAL if leg[0] == "l" else AMBER
        s.text(x0 - 16, y + 24, f"{'left' if leg[0] == 'l' else 'right'} {leg[1]}",
               13, INK, anchor="end")
        s.parts.append(f'<rect x="{x0}" y="{y + 8}" width="{x1 - x0}" height="22" '
                       f'rx="4" fill="#eef2f6"/>')
        t0 = g["lift"][leg]
        for a, b in ((t0, min(1.0, t0 + g["swing"])), (0.0, max(0.0, t0 + g["swing"] - 1.0))):
            if b > a:
                s.parts.append(
                    f'<rect x="{x0 + (x1 - x0) * a:.1f}" y="{y + 8}" '
                    f'width="{(x1 - x0) * (b - a):.1f}" height="22" rx="4" fill="{colour}"/>')
    # feet down count
    s.save("gait_timing.svg")


def leg_map():
    sys.path.insert(0, SIM)
    import spider
    _cad, legs, _ground = spider.build_robot()
    s = Svg(700, 640, "Leg names and servo order (top view, from the CAD)")
    cx, cy, k = 350, 330, 900.0
    s.text(350, 70, "front (payload case)", 13, MUTED, anchor="middle")
    s.text(350, 620, "back (battery)", 13, MUTED, anchor="middle")
    pts = []
    for name, L in legs.items():
        hx, hy = L["pivot"]["hip"][:2]
        tx, ty = L["tip"][:2]
        X1, Y1 = cx - hy * k, cy - hx * k
        X2, Y2 = cx - ty * k, cy - tx * k
        pts.append((X1, Y1))
        colour = TEAL if name[0] == "l" else AMBER
        s.parts.append(f'<line x1="{X1:.1f}" y1="{Y1:.1f}" x2="{X2:.1f}" y2="{Y2:.1f}" '
                       f'stroke="{colour}" stroke-width="6" stroke-linecap="round"/>')
        s.parts.append(f'<circle cx="{X2:.1f}" cy="{Y2:.1f}" r="7" fill="{colour}"/>')
        idx = [f"{l}_{j}" for l in ("l1", "r1", "l2", "r2", "l3", "r3", "l4", "r4")
               for j in ("hip", "knee", "ankle")].index(f"{name}_hip")
        dx = -1 if name[0] == "l" else 1
        s.text(X2 + 16 * dx, Y2 + 5, f"{name}  ({idx}-{idx + 2})", 14, INK,
               anchor="end" if dx < 0 else "start", weight=600)
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    s.parts.insert(1, f'<rect x="{min(xs) - 18:.1f}" y="{min(ys) - 30:.1f}" '
                      f'width="{max(xs) - min(xs) + 36:.1f}" height="{max(ys) - min(ys) + 60:.1f}" '
                      f'rx="18" fill="#e4e7eb"/>')
    s.text(350, 335, "body", 14, MUTED, anchor="middle")
    s.text(24, 96, "Servo index = actuator order in gait.py, spider.xml and the RL action "
           "(leg, then hip, knee, ankle).", 12, MUTED)
    s.save("leg_map.svg")


def roadmap():
    rows = [("Sprint 0", 1, 6, "allocation, partner intro, research", TEAL),
            ("Sprint 1", 6, 8, "Craig kickoff, stack decision", TEAL),
            ("Sprint 2", 8, 10, "PyBullet spider v1-v5, Gymnasium env", TEAL),
            ("Sprint 3", 10, 12, "PPO pipeline, toolchain fix, telemetry", TEAL),
            ("Sprint 4", 12, 13, "CAD-built simulation, tests, CI", TEAL),
            ("P2-S1", 13, 15, "hardware bring-up, servo order map", VIOLET),
            ("P2-S2", 15, 17, "sim-to-real calibration", VIOLET),
            ("P2-S3", 17, 19, "rough-ground policy training", VIOLET),
            ("P2-S4", 19, 21, "policy on Pi 5 + ESP32 command API", VIOLET),
            ("P2-S5", 21, 23, "field trials, live dashboard", VIOLET),
            ("P2-S6", 23, 26, "evaluation, report, final demo", VIOLET)]
    top, row = 110, 32
    s = Svg(1100, top + row * len(rows) + 70,
            "Roadmap: Phase 1 (IFB398, 2026 S2) and Phase 2 (IFB399, 2027 S1)")
    x0, x1, weeks = 360, 1070, 26

    def X(w):
        return x0 + (x1 - x0) * w / weeks
    s.parts.append(f'<rect x="{X(0)}" y="72" width="{X(13) - X(0)}" height="24" rx="6" fill="#e0f2f1"/>')
    s.parts.append(f'<rect x="{X(13)}" y="72" width="{X(26) - X(13)}" height="24" rx="6" fill="#ede9fe"/>')
    s.text(X(6.5), 89, "Phase 1", 13, TEAL, "middle", 700)
    s.text(X(19.5), 89, "Phase 2", 13, VIOLET, "middle", 700)
    for i, (name, a, b, what, c) in enumerate(rows):
        y = top + i * row
        s.text(24, y + 19, name, 13, c, weight=700)
        s.text(100, y + 19, what, 13, MUTED)
        s.parts.append(f'<rect x="{X(a) + 1:.1f}" y="{y + 4}" width="{X(b) - X(a) - 2:.1f}" '
                       f'height="20" rx="5" fill="{c}"/>')
    yb = top + row * len(rows)
    for w, label in ((13, "A2 presentation"), (14, "A3 report")):  # noqa: B007
        s.parts.append(f'<line x1="{X(w)}" y1="{top - 6}" x2="{X(w)}" y2="{yb}" stroke="{ROSE}" '
                       'stroke-dasharray="4 4" stroke-width="1.5"/>')
    s.text(X(13) - 4, yb + 18, "A2 presentation (22 Oct)", 12, ROSE, "end")
    s.text(X(14) + 4, yb + 18, "A3 report", 12, ROSE)
    for w in range(1, 27):
        s.text(X(w - 0.5), yb + 40, str(w if w <= 13 else w - 13), 11, MUTED, "middle")
    s.text(x0 - 10, yb + 40, "week", 11, MUTED, "end")
    s.save("roadmap.svg")


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    system_architecture()
    sim_pipeline()
    residual_rl()
    gait_timing()
    leg_map()
    roadmap()
