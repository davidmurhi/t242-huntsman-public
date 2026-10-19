"""The reinforcement-learning environment (huntsman_env.py) follows the
Gymnasium API. Skipped without mujoco or gymnasium."""

import numpy as np
import pytest

pytest.importorskip("mujoco")
pytest.importorskip("gymnasium")

from huntsman_env import HuntsmanEnv  # noqa: E402


@pytest.fixture(scope="module")
def env(spider_xml_path):
    return HuntsmanEnv(terrains=("flat", "slope"), seed=0)


def test_spaces(env):
    assert env.action_space.shape == (24,)
    assert env.observation_space.shape == (83,)


def test_reset_and_step(env):
    obs, info = env.reset(seed=1)
    assert obs.shape == (83,) and np.isfinite(obs).all()
    assert info["terrain"] in ("flat", "slope")
    total = 0.0
    for _ in range(50):
        obs, r, term, trunc, info = env.step(np.zeros(24, np.float32))
        assert np.isfinite(obs).all() and np.isfinite(r)
        total += r
        if term or trunc:
            break
    assert not term                      # the hand gait does not fall over
    assert total > 0.0


def test_zero_action_is_the_hand_gait(env):
    """A residual policy outputting zero must walk like the gait alone."""
    env.reset(seed=2)
    for _ in range(150):
        env.step(np.zeros(24, np.float32))
    assert env.data.qpos[0] > 0.05
