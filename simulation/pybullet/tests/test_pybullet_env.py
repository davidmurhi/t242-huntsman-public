"""Prab's Gymnasium environment around Kai's robot (spider_env.py) and
the joint-angle to servo-pulse mapping (pwm_mapping.py). Skipped without
pybullet / gymnasium."""

import os

import numpy as np
import pytest

p = pytest.importorskip("pybullet")
pytest.importorskip("gymnasium")

import pwm_mapping  # noqa: E402
from spider_env import SpiderEnv  # noqa: E402

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@pytest.fixture(scope="module")
def env():
    e = SpiderEnv(render=False)
    yield e
    e.close()


def test_spaces(env):
    assert env.action_space.shape == (24,)
    assert env.observation_space.shape == (31,)


def test_reset_observation(env):
    obs, info = env.reset(seed=0)
    assert obs.shape == env.observation_space.shape
    assert np.isfinite(obs).all()
    assert obs[27] > 0.1                      # body height above the floor


def test_steps_stay_finite(env):
    env.reset(seed=1)
    rng = np.random.default_rng(0)
    for _ in range(200):
        obs, r, term, trunc, _info = env.step(rng.uniform(-1, 1, 24))
        assert np.isfinite(obs).all() and np.isfinite(r)
        if term or trunc:
            break


def test_action_scaling_reaches_joint_limits(env):
    env.reset(seed=2)
    for name in env.JOINT_NAMES:
        lo, hi = env._joint_limits[name]
        assert env._scale_action(name, -1.0) == pytest.approx(lo)
        assert env._scale_action(name, 1.0) == pytest.approx(hi)
        assert env._scale_action(name, 5.0) == pytest.approx(hi)


def test_pwm_mapping():
    f = pwm_mapping.map_angle_to_pulse_us
    assert f(-0.7, -0.7, 0.7) == pytest.approx(pwm_mapping.PWM_MIN_US)
    assert f(0.7, -0.7, 0.7) == pytest.approx(pwm_mapping.PWM_MAX_US)
    assert f(0.0, -0.7, 0.7) == pytest.approx(1500)
    assert f(9.9, -0.7, 0.7) == pytest.approx(pwm_mapping.PWM_MAX_US)


def test_pwm_limits_come_from_the_urdf():
    limits = pwm_mapping.get_joint_limits_from_urdf(
        os.path.join(HERE, "spider8_v5_longbody_3joint.urdf"))
    assert len([n for n in limits if n.endswith(("_hip", "_knee", "_ankle"))]) == 24
