"""
gait.py

The Huntsman's walking controller, kept apart from MuJoCo so the same
code can drive the simulation, the real robot's servo board, and serve
as the hand-written baseline a trained policy has to beat.

HOW IT WORKS
    Feet, not joints. Each foot follows a path in the body's frame and
    the three joint angles that put it there are solved for (inverse
    kinematics on V Source's own leg geometry, from spider.py).

    Stance: a planted foot moves backwards at exactly the speed the body
    moves forwards (and turns), so it does not scrape along the floor.
    Swinging an angle at the hip, as the first gait did, drags each foot
    round an arc while the body goes straight; the difference was slip.

    Swing: the foot leaves the floor moving at its stance speed, arcs
    over to its next foothold and lands moving at stance speed again,
    with zero vertical speed at lift-off and touch-down (a cubic Hermite
    curve across, sin^2 up and down). No jerk at either end of a step.

    Footholds are placed half a stride ahead of the leg's neutral point
    in whatever direction the body is commanded to move, so forwards,
    backwards, sideways, turning on the spot and any mix all come from
    the same rule.

COMMANDS (what the operator dashboard's drive pad sends)
    vx   forward speed, m/s (negative walks backwards)
    vy   sideways speed, m/s (positive to the left)
    wz   turn rate, deg/s (positive turns left)
    gait walk | trot | rubble | crouch
    hold stop and stand still     estop freeze every servo where it is

    Commands are rate-limited (ACCEL, TURN_ACCEL) so starts, stops and
    changes of direction are smooth.

GAITS
    walk    spider ripple: one leg at a time down each side, front to
            back, right side 5/8 of a cycle behind the left. Every leg on
            its own beat, six feet down. The default.
    trot    alternating tetrapod: l1 r2 l3 r4 together, then the other
            four. Faster, four feet always down.
    rubble  the ripple, slower, with a 50 mm step to clear debris.
    crouch  the ripple with the body 25 mm lower, short slow steps.
"""

from __future__ import annotations

import math

import numpy as np

LEGS = ("l1", "r1", "l2", "r2", "l3", "r3", "l4", "r4")
JOINTS = ("hip", "knee", "ankle")

# name: cycle s, swing share, step height m, body drop m, top speed m/s,
#       top turn deg/s, lift time of each leg as a share of the cycle
_RIPPLE = {"l1": 0.000, "l2": 0.250, "l3": 0.500, "l4": 0.750,
           "r1": 0.625, "r2": 0.875, "r3": 0.125, "r4": 0.375}
# The wave always runs from the end of a side that is leading, so a leg
# steps just after its neighbour ahead has landed and the two move the
# same way. Walking backwards, or on the side going backwards in a turn,
# the wave runs back to front: pair p takes pair (5 - p)'s moment.
_RIPPLE_BACK = {f"{n[0]}{5 - int(n[1])}": t for n, t in _RIPPLE.items()}
_TRIPOD = {"l1": 0.0, "r2": 0.0, "l3": 0.0, "r4": 0.0,
           "r1": 0.5, "l2": 0.5, "r3": 0.5, "l4": 0.5}
GAITS = {
    "walk":   dict(cycle=0.50, swing=0.22, height=0.028, drop=0.000,
                   speed=0.22, turn=60.0, stride=0.090,
                   lift=_RIPPLE, back=_RIPPLE_BACK),
    "trot":   dict(cycle=0.40, swing=0.45, height=0.028, drop=0.000,
                   speed=0.26, turn=60.0, stride=0.058,
                   lift=_TRIPOD, back=_TRIPOD),
    "rubble": dict(cycle=0.90, swing=0.22, height=0.050, drop=-0.010,
                   speed=0.12, turn=30.0, stride=0.090,
                   lift=_RIPPLE, back=_RIPPLE_BACK),
    "crouch": dict(cycle=0.75, swing=0.22, height=0.020, drop=0.025,
                   speed=0.10, turn=25.0, stride=0.090,
                   lift=_RIPPLE, back=_RIPPLE_BACK),
}
MAX_STRIDE = 0.090        # m, longest step any foot is asked to make
ACCEL = 1.0               # m/s^2, how fast commanded speed may change
TURN_ACCEL = 300.0        # deg/s^2
DROP_RATE = 0.03          # m/s, body raise / lower speed between gaits

# Heading hold: while no turn is commanded, steer to keep the heading the
# robot had when the last turn ended, and (walking straight) to stay on
# the line it started on. The real robot does this with its IMU.
HEADING_GAIN = 2.5        # deg/s of turn per deg of heading error
LINE_GAIN = 3.0           # rad of heading per metre off the line
MAX_CORRECTION = 15.0     # deg/s


def _rot(axis, ang):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * k + (1 - math.cos(ang)) * k @ k


class Leg:
    """One leg's kinematics, from the standing pose spider.py builds."""

    def __init__(self, name, pose, ranges):
        self.name = name
        self.H = np.array(pose["pivot"]["hip"], float)
        self.K = np.array(pose["pivot"]["knee"], float)
        self.A = np.array(pose["pivot"]["ankle"], float)
        self.T = np.array(pose["tip"], float)
        self.z = np.array([0.0, 0.0, 1.0])
        self.l = np.array(pose["axis"]["knee"], float)
        self.l /= np.linalg.norm(self.l)
        self.lo = np.radians([ranges[j][0] for j in JOINTS]) + 0.02
        self.hi = np.radians([ranges[j][1] for j in JOINTS]) - 0.02
        self.q = np.zeros(3)

    def fk(self, q):
        """Foot position and the Jacobian for joint angles q (rad)."""
        qh, qk, qa = q
        Rk, Ra = _rot(self.l, qk), _rot(self.l, qa)
        t1 = Ra @ (self.T - self.A) + self.A
        a2 = Rk @ (self.A - self.K) + self.K
        t2 = Rk @ (t1 - self.K) + self.K
        Rh = _rot(self.z, qh)
        foot = Rh @ (t2 - self.H) + self.H
        k_w = Rh @ (self.K - self.H) + self.H
        a_w = Rh @ (a2 - self.H) + self.H
        l_w = Rh @ self.l
        J = np.stack([np.cross(self.z, foot - self.H),
                      np.cross(l_w, foot - k_w),
                      np.cross(l_w, foot - a_w)], axis=1)
        return foot, J

    def ik(self, target, iters=4):
        q = self.q.copy()
        for _ in range(iters):
            foot, J = self.fk(q)
            err = target - foot
            if err @ err < 1e-10:
                break
            # Damped least squares: well behaved near the reach limits.
            q += J.T @ np.linalg.solve(J @ J.T + 1e-6 * np.eye(3), err)
            q = np.clip(q, self.lo, self.hi)
        self.q = q
        return q


class Gait:
    """Turns drive commands into 24 servo targets, one control tick at a
    time. Call step(dt, heading=..., position=...) every tick."""

    def __init__(self, poses, ranges):
        self.legs = {n: Leg(n, poses[n], ranges) for n in LEGS}
        self.neutral = {n: leg.T.copy() for n, leg in self.legs.items()}
        self.foot = {n: leg.T.copy() for n, leg in self.legs.items()}
        self.start = {n: leg.T.copy() for n, leg in self.legs.items()}
        self.swinging = {n: False for n in LEGS}
        self.start_v = {n: np.zeros(3) for n in LEGS}
        self.phase = 0.0
        self.gait = "walk"
        self.cmd = np.zeros(3)          # vx, vy (m/s), wz (rad/s) requested
        self.vel = np.zeros(3)          # what the feet are actually doing
        self.drop = 0.0
        self.stepping = 0.0             # 0 = standing still, 1 = stepping
        self.estop = False
        self.idle = 0.0
        self.heading_ref = None
        self.line_ref = None
        self.heading_hold = True
        self.side_back = {"l": False, "r": False}
        self._last = np.zeros(24)

    # ----------------------------------------------------------- commands
    def drive(self, vx=0.0, vy=0.0, wz_deg=0.0):
        g = GAITS[self.gait]
        v = np.array([vx, vy], float)
        s = np.linalg.norm(v)
        if s > g["speed"]:
            v *= g["speed"] / s
        wz = max(-g["turn"], min(g["turn"], wz_deg))
        self.cmd = np.array([v[0], v[1], math.radians(wz)])

    def hold(self):
        self.cmd = np.zeros(3)

    def set_gait(self, name):
        if name in GAITS:
            self.gait = name
            self.drive(*self.cmd[:2], math.degrees(self.cmd[2]))

    def set_estop(self, engaged):
        self.estop = bool(engaged)
        if engaged:
            self.cmd = np.zeros(3)
            self.vel = np.zeros(3)

    # ------------------------------------------------------------- ticking
    def _heading_correction(self, heading, position):
        """Turn rate (rad/s) that holds heading and line when no turn is
        commanded. heading in rad, position (x, y) in the world."""
        if not self.heading_hold or heading is None:
            return 0.0
        turning = abs(self.cmd[2]) > 1e-4
        moving = np.linalg.norm(self.cmd[:2]) > 1e-4
        if turning or not moving:
            self.heading_ref = None
            self.line_ref = None
            return 0.0
        if self.heading_ref is None:
            self.heading_ref = heading
            self.line_ref = (np.array(position, float)
                             if position is not None else None)
        aim = self.heading_ref
        if self.line_ref is not None and abs(self.cmd[1]) < 1e-4:
            d = np.array(position, float) - self.line_ref
            off = -d[0] * math.sin(aim) + d[1] * math.cos(aim)
            off *= 1.0 if self.cmd[0] >= 0 else -1.0
            aim -= math.atan(LINE_GAIN * off)
        err = math.atan2(math.sin(aim - heading), math.cos(aim - heading))
        lim = math.radians(MAX_CORRECTION)
        return max(-lim, min(lim, HEADING_GAIN * err))

    def step(self, dt, heading=None, position=None):
        """Advance dt seconds; returns the 24 joint targets (rad) in
        LEGS x JOINTS order."""
        if self.estop:
            return self._last.copy()
        g = GAITS[self.gait]

        # Smooth the command, then add heading hold.
        want = self.cmd.copy()
        want[2] += self._heading_correction(heading, position)
        dv = want[:2] - self.vel[:2]
        lim = ACCEL * dt
        if np.linalg.norm(dv) > lim:
            dv *= lim / np.linalg.norm(dv)
        self.vel[:2] += dv
        dw = want[2] - self.vel[2]
        lim = math.radians(TURN_ACCEL) * dt
        self.vel[2] += max(-lim, min(lim, dw))

        # Body height for the gait.
        dd = g["drop"] - self.drop
        self.drop += max(-DROP_RATE * dt, min(DROP_RATE * dt, dd))

        # Keep stepping while moving, or until every foot is back home.
        moving = (np.linalg.norm(self.vel[:2]) > 1e-3
                  or abs(self.vel[2]) > 1e-3)
        home = all(np.linalg.norm((self.foot[n] - self.neutral[n])[:2])
                   < 0.002 for n in LEGS)
        self.idle = 0.0 if moving or not home else self.idle + dt
        target = 1.0 if (moving or not home or self.idle < g["cycle"]) else 0.0
        self.stepping += max(-dt / 0.3, min(dt / 0.3, target - self.stepping))

        cycle, share = g["cycle"], g["swing"]
        t_swing, t_stance = share * cycle, (1 - share) * cycle
        if self.stepping > 0.0:
            self.phase = (self.phase + dt / cycle) % 1.0

        v = np.array([self.vel[0], self.vel[1], 0.0])
        w = np.array([0.0, 0.0, self.vel[2]])
        # Stride, limited so no foot is asked to step further than it can.
        strides = {n: (v + np.cross(w, self.neutral[n])) * t_stance
                   for n in LEGS}
        biggest = max(np.linalg.norm(s[:2]) for s in strides.values())
        cap = min(MAX_STRIDE, g.get("stride", MAX_STRIDE))
        scale = min(1.0, cap / biggest) if biggest > 0 else 1.0

        # Which way each side is travelling: its feet's mean stance
        # velocity along the body. Switch wave direction with hysteresis.
        for side in "lr":
            fx = np.mean([-(v + np.cross(w, self.neutral[n]))[0]
                          for n in LEGS if n[0] == side])
            if fx > 0.01 and not self.side_back[side]:
                self.side_back[side] = True
            elif fx < -0.01 and self.side_back[side]:
                self.side_back[side] = False

        out = np.zeros(24)
        for i, n in enumerate(LEGS):
            table = g["back"] if self.side_back[n[0]] else g["lift"]
            leg_phase = (self.phase - table[n]) % 1.0
            swinging = leg_phase < share and self.stepping > 0.0
            home_n = self.neutral[n].copy()
            home_n[2] += self.drop
            stride = strides[n] * scale
            stance_v = -stride / t_stance

            if swinging and not self.swinging[n]:
                self.start[n] = self.foot[n].copy()
                self.start_v[n] = stance_v.copy()
            if swinging:
                s = leg_phase / share
                land = home_n + stride / 2.0
                p0, p1 = self.start[n], land
                m0, m1 = self.start_v[n] * t_swing, stance_v * t_swing
                h00 = 2 * s ** 3 - 3 * s ** 2 + 1
                h10 = s ** 3 - 2 * s ** 2 + s
                h01 = -2 * s ** 3 + 3 * s ** 2
                h11 = s ** 3 - s ** 2
                p = h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1
                p[2] = (h00 * p0[2] + h01 * p1[2]
                        + g["height"] * self.stepping
                        * math.sin(math.pi * s) ** 2)
                self.foot[n] = p
            else:
                # Planted: from half a stride ahead of home to half a
                # stride behind, at the body's speed, so it does not slide.
                u = ((leg_phase - share) / (1.0 - share)
                     if self.stepping > 0.0 else 0.5)
                self.foot[n] = home_n + stride * (0.5 - u)
            self.swinging[n] = swinging
            out[3 * i:3 * i + 3] = self.legs[n].ik(self.foot[n])
        self._last = out
        return out


def from_spider():
    """A Gait built on the CAD model's standing pose."""
    import spider
    _cad, legs, _ground = spider.build_robot()
    poses = {n: {"pivot": legs[n]["pivot"], "axis": legs[n]["axis"],
                 "tip": legs[n]["tip"]} for n in LEGS}
    ranges = {"hip": spider.HIP_RANGE, "knee": spider.KNEE_RANGE,
              "ankle": spider.ANKLE_RANGE}
    return Gait(poses, ranges)
