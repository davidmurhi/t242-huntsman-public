"""Closed-loop physics in MuJoCo: the robot stands, walks straight,
turns, keeps its legs apart and copes with the start of each terrain.
Skipped when the mujoco package is not installed."""

import math
import os

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

import gait as gaitlib  # noqa: E402
import terrain  # noqa: E402

CONTROL_DT = 0.01


def _order(model):
    return [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{l}_{j}")
            for l in gaitlib.LEGS for j in gaitlib.JOINTS]


def _yaw(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _tilt(q):
    _w, x, y, _z = q
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def drive(model, seconds, vx=0.0, vy=0.0, wz=0.0, gait="walk"):
    """Run the gait in closed loop; return start pose, data and stats."""
    data = mujoco.MjData(model)
    order = _order(model)
    g = gaitlib.from_spider()
    g.set_gait(gait)
    for _ in range(400):                     # settle onto its feet
        mujoco.mj_step(model, data)
    start = data.qpos[:7].copy()
    per = max(1, round(CONTROL_DT / model.opt.timestep))
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
             for i in range(model.ngeom)]
    leg_hits, worst_tilt = 0, 0.0
    g.drive(vx, vy, wz)
    for _ in range(int(seconds / CONTROL_DT)):
        targets = g.step(CONTROL_DT, heading=_yaw(data.qpos[3:7]),
                         position=(data.qpos[0], data.qpos[1]))
        for k, a in enumerate(order):
            data.ctrl[a] = targets[k]
        for _ in range(per):
            mujoco.mj_step(model, data)
        for c in range(data.ncon):
            a, b = names[data.contact[c].geom1], names[data.contact[c].geom2]
            if "ground" not in (a, b):
                leg_hits += 1
        worst_tilt = max(worst_tilt, _tilt(data.qpos[3:7]))
    return start, data, leg_hits, worst_tilt


def test_model_loads_with_24_servos(model):
    assert model.nu == 24
    assert min(_order(model)) >= 0
    assert model.actuator_ctrlrange.max() < math.pi


def test_mass_matches_weight_sheet(model):
    assert sum(model.body_mass) == pytest.approx(4.326, abs=0.01)


def test_stands_level(model):
    start, data, hits, tilt = drive(model, 2.0)
    assert tilt < 2.0
    assert hits == 0
    assert data.qpos[2] > 0.05                  # body clear of the floor
    assert np.linalg.norm(data.qpos[:2] - start[:2]) < 0.01


def test_walks_straight_without_leg_contact(model):
    start, data, hits, tilt = drive(model, 10.0, vx=0.10)
    forward = data.qpos[0] - start[0]
    assert forward > 0.6                         # asked for 1.0 m
    assert abs(data.qpos[1] - start[1]) < 0.05
    assert hits == 0
    assert tilt < 5.0


def test_walks_backwards_and_sideways(model):
    start, data, hits, _ = drive(model, 6.0, vx=-0.08)
    assert data.qpos[0] - start[0] < -0.25 and hits == 0
    start, data, hits, _ = drive(model, 6.0, vy=0.07)
    assert data.qpos[1] - start[1] > 0.2 and hits == 0


def test_turns_on_the_spot(model):
    start, data, hits, _ = drive(model, 4.0, wz=45.0)
    turned = math.degrees(_yaw(data.qpos[3:7]) - _yaw(start[3:7]))
    turned = (turned + 180) % 360 - 180
    assert turned > 60.0
    assert hits == 0


@pytest.mark.parametrize("name", ["trot", "rubble", "crouch"])
def test_other_gaits_walk(model, name):
    start, data, hits, tilt = drive(model, 6.0,
                                    vx=gaitlib.GAITS[name]["speed"] * 0.8,
                                    gait=name)
    assert data.qpos[0] - start[0] > 0.2
    assert hits == 0 and tilt < 8.0


@pytest.mark.parametrize("kind", ["rubble", "slope", "steps", "mixed"])
def test_terrain_models_load_and_run(spider_xml_path, kind):
    xml = terrain.model_xml(kind, seed=2, base=spider_xml_path)
    # textures resolve relative to the model file, so write it beside spider.xml
    path = os.path.join(os.path.dirname(spider_xml_path), f".test_{kind}.xml")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(xml)
    try:
        m = mujoco.MjModel.from_xml_path(path)
    finally:
        os.remove(path)
    _start, data, _hits, _tilt = drive(m, 3.0, vx=0.10, gait="rubble")
    assert np.isfinite(data.qpos).all()
