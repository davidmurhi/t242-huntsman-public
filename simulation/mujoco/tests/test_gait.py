"""The hand-written gait (gait.py): inverse kinematics, joint limits,
how many feet stay planted, no foot slip, stop and e-stop."""

import math

import numpy as np
import pytest

import gait as gaitlib

DT = 0.01


@pytest.fixture()
def g():
    return gaitlib.from_spider()


def run(g, seconds, **cmd):
    if cmd:
        g.drive(cmd.get("vx", 0.0), cmd.get("vy", 0.0), cmd.get("wz", 0.0))
    out = None
    for _ in range(int(seconds / DT)):
        out = g.step(DT)
    return out


def test_ik_reaches_the_standing_foot(g):
    for leg in g.legs.values():
        q = leg.ik(leg.T.copy(), iters=10)
        foot, _J = leg.fk(q)
        assert np.linalg.norm(foot - leg.T) < 1e-4


def test_ik_round_trip_inside_reach(g):
    rng = np.random.default_rng(0)
    for leg in g.legs.values():
        for _ in range(5):
            q = rng.uniform(leg.lo * 0.5, leg.hi * 0.5)
            target, _J = leg.fk(q)
            leg.q = np.zeros(3)
            got, _J = leg.fk(leg.ik(target, iters=30))
            assert np.linalg.norm(got - target) < 1e-3


def test_targets_within_joint_limits(g):
    lo = np.concatenate([g.legs[n].lo for n in gaitlib.LEGS])
    hi = np.concatenate([g.legs[n].hi for n in gaitlib.LEGS])
    for cmd in ({"vx": 0.2}, {"vx": -0.15}, {"vy": 0.15}, {"wz": 60.0},
                {"vx": 0.15, "wz": 40.0}):
        g.drive(cmd.get("vx", 0.0), cmd.get("vy", 0.0), cmd.get("wz", 0.0))
        for _ in range(300):
            out = g.step(DT)
            assert out.shape == (24,)
            assert np.isfinite(out).all()
            assert (out >= lo - 1e-9).all() and (out <= hi + 1e-9).all()


@pytest.mark.parametrize("name,least", [("walk", 6), ("rubble", 6),
                                        ("crouch", 6), ("trot", 4)])
def test_feet_planted(g, name, least):
    """Ripple gaits keep six feet down; the trot keeps four."""
    g.set_gait(name)
    g.drive(gaitlib.GAITS[name]["speed"], 0.0, 0.0)
    fewest = 8
    for _ in range(400):
        g.step(DT)
        fewest = min(fewest, 8 - sum(g.swinging.values()))
    assert fewest >= least


def test_planted_feet_do_not_slip(g):
    """A planted foot moves backwards at exactly the body's speed."""
    vx = 0.10
    run(g, 3.0, vx=vx)                      # reach steady speed
    before = {n: g.foot[n].copy() for n in gaitlib.LEGS}
    was = dict(g.swinging)
    g.step(DT)
    for n in gaitlib.LEGS:
        if not was[n] and not g.swinging[n]:
            dx = (g.foot[n] - before[n])[0] / DT
            assert dx == pytest.approx(-vx, abs=0.02)


def test_speed_is_capped(g):
    g.drive(5.0, 0.0, 500.0)
    top = gaitlib.GAITS["walk"]
    assert math.hypot(*g.cmd[:2]) <= top["speed"] + 1e-9
    assert abs(math.degrees(g.cmd[2])) <= top["turn"] + 1e-9


def test_comes_to_rest_with_feet_home(g):
    run(g, 2.0, vx=0.15)
    g.hold()
    run(g, 3.0)
    for n in gaitlib.LEGS:
        assert np.linalg.norm((g.foot[n] - g.neutral[n])[:2]) < 0.003
    assert g.stepping == pytest.approx(0.0, abs=1e-6)


def test_estop_freezes_every_servo(g):
    run(g, 1.0, vx=0.15)
    frozen = g.step(DT).copy()
    g.set_estop(True)
    for _ in range(50):
        assert np.array_equal(g.step(DT), frozen)
    g.set_estop(False)
    g.step(DT)


def test_unknown_gait_is_ignored(g):
    g.set_gait("moonwalk")
    assert g.gait == "walk"
