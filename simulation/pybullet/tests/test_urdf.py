"""Kai's PyBullet spider (spider8_v5_longbody_3joint.urdf): the robot
description every PyBullet script and Prab's environment load."""

import os
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(HERE, "spider8_v5_longbody_3joint.urdf")
LEGS = ("l1", "r1", "l2", "r2", "l3", "r3", "l4", "r4")
NAMES = [f"{leg}_{j}" for leg in LEGS for j in ("hip", "knee", "ankle")]


@pytest.fixture(scope="module")
def robot():
    return ET.parse(URDF).getroot()


def test_24_revolute_joints_named_like_the_robot(robot):
    joints = {j.get("name"): j for j in robot.iter("joint")
              if j.get("type") == "revolute"}
    assert sorted(joints) == sorted(NAMES)


def test_joint_limits_are_sane(robot):
    for j in robot.iter("joint"):
        if j.get("type") != "revolute":
            continue
        lim = j.find("limit")
        lo, hi = float(lim.get("lower")), float(lim.get("upper"))
        assert -3.15 < lo < hi < 3.15
        assert float(lim.get("effort")) > 0


def test_every_link_has_mass(robot):
    for link in robot.iter("link"):
        inertial = link.find("inertial")
        assert inertial is not None, link.get("name")
        assert float(inertial.find("mass").get("value")) > 0


def test_left_and_right_mirror(robot):
    limits = {j.get("name"): (j.find("limit").get("lower"),
                              j.find("limit").get("upper"))
              for j in robot.iter("joint") if j.get("type") == "revolute"}
    for name in NAMES:
        if name.startswith("l"):
            assert limits[name] == limits["r" + name[1:]]
