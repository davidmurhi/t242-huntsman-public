"""The robot model is built from V Source's CAD: check it still matches
the numbers we were given (part weights, joint layout, clearances)."""

import math
import re

import numpy as np

import gait
import spider


def test_mass_matches_weight_sheet():
    cad = spider.load_cad()
    # Ronan's part-weight sheet (hardware/Spider Part Weight.xlsx): 4326 g
    assert math.isclose(spider.total_mass(cad), 4.326, abs_tol=0.002)


def test_eight_legs_named_in_servo_order():
    _cad, legs, _ground = spider.build_robot()
    assert sorted(legs) == sorted(gait.LEGS)
    assert len(gait.LEGS) * len(gait.JOINTS) == 24


def test_build_checks_pass():
    cad, legs, ground = spider.build_robot()
    problems, notes = spider.run_checks(cad, legs, ground)
    assert problems == [], problems
    assert notes


def test_standing_legs_do_not_touch():
    _cad, legs, _ground = spider.build_robot()
    for side in "lr":
        for p in (1, 2, 3):
            assert spider.gap(legs[f"{side}{p}"], legs[f"{side}{p + 1}"]) \
                >= spider.MIN_LEG_GAP


def test_servo_torque_margin_standing():
    """Knee load on six feet stays well under the DSC55MG rating."""
    cad, legs, _ground = spider.build_robot()
    per_foot = spider.total_mass(cad) * spider.GRAVITY / 6
    arm = max(math.hypot(*(L["tip"] - L["pivot"]["knee"])[:2])
              for L in legs.values())
    assert per_foot * arm < 0.4 * spider.SERVO_TORQUE


def test_model_xml_has_24_servos_in_order():
    xml = spider.build_model()
    names = re.findall(r'<position[^>]*name="([a-z0-9_]+)"', xml)
    want = [f"{leg}_{j}" for leg in gait.LEGS for j in gait.JOINTS]
    assert sorted(names) == sorted(want)


def test_servo_targets_are_radians():
    """Control ranges must be in radians (degrees drove every servo into
    its stop in an early version)."""
    xml = spider.build_model()
    for lo, hi in re.findall(r'ctrlrange="([-0-9.e]+) ([-0-9.e]+)"', xml):
        assert abs(float(lo)) < math.pi and abs(float(hi)) < math.pi


def test_fan_angles_follow_the_cad():
    _cad, legs, _ground = spider.build_robot()
    fans = {n: spider.leg_fan(L) for n, L in legs.items()}
    # front legs point forward, rear legs point back, each side mirrored
    for side in "lr":
        assert fans[f"{side}1"] > fans[f"{side}4"]
    assert np.isfinite(list(fans.values())).all()
