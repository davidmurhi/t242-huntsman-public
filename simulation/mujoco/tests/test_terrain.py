"""Rough-ground terrains for testing and training (terrain.py)."""

import numpy as np
import pytest

import terrain


@pytest.mark.parametrize("kind", ["rubble", "slope", "steps", "mixed"])
def test_heights_in_range_and_start_pad_flat(kind):
    h = terrain.heights(kind, seed=3)
    X, _Y = terrain._grid()
    assert h.shape == X.shape
    assert h.min() >= 0.0 and h.max() <= terrain.MAX_RISE
    assert np.all(h[X <= terrain.START_PAD] == 0.0)
    assert h.max() > 0.005


def test_same_seed_same_ground():
    a = terrain.heights("rubble", seed=11)
    b = terrain.heights("rubble", seed=11)
    c = terrain.heights("rubble", seed=12)
    assert np.array_equal(a, b)
    assert not np.array_equal(a, c)


def test_difficulty_scales_roughness():
    easy = terrain.heights("steps", seed=0, difficulty=0.0).max()
    hard = terrain.heights("steps", seed=0, difficulty=1.0).max()
    assert hard > easy
    assert hard == pytest.approx(5 * 0.050, rel=0.01)   # five 50 mm steps


def test_model_swaps_the_floor(spider_xml_path):
    xml = terrain.model_xml("slope", seed=1, base=spider_xml_path)
    assert '<hfield name="terrain"' in xml
    assert 'type="hfield"' in xml
    assert '<geom name="ground" type="hfield"' in xml


def test_unknown_terrain_rejected(spider_xml_path):
    with pytest.raises(ValueError):
        terrain.model_xml("lava", base=spider_xml_path)
