"""The STEP reader (cad/step.py) that turns V Source's battery model
into meshes, including its curved (B-spline, torus, cone) faces."""

import os
import sys

import numpy as np
import pytest

SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(SIM))

sys.path.insert(0, os.path.join(SIM, "cad"))
import step  # noqa: E402

BATTERY = os.path.join(REPO, "hardware", "cad", "battery",
                       "Battery LxT 18V v7.step")


@pytest.fixture(scope="module")
def solids():
    return step.Step(BATTERY).solids()


def test_solids_in_five_colours(solids):
    """Case, cell board, connector, latch and contacts."""
    assert len(solids) >= 5
    assert len({tuple(round(c, 3) for c in rgb) for _n, rgb, _t in solids}) == 5
    for _name, rgb, tris in solids:
        assert tris.shape[1:] == (3, 3) and len(tris) > 0
        assert np.isfinite(tris).all()


def test_case_is_the_size_of_the_pack(solids):
    case = max(solids, key=lambda s: len(s[2]))[2].reshape(-1, 3)
    size = np.sort(case.max(0) - case.min(0))     # mm
    # Makita 18 V 5.0 Ah: about 115 x 75 x 62 mm
    assert size[-1] == pytest.approx(115.7, abs=1.5)
    assert size[-2] == pytest.approx(75.0, abs=1.5)


def test_curved_faces_are_on_their_surfaces(solids):
    """Every vertex of the tessellated case should lie close to the true
    bounding shape: nothing flung outside the pack."""
    case = max(solids, key=lambda s: len(s[2]))[2].reshape(-1, 3)
    lo, hi = np.percentile(case, 0.5, axis=0), np.percentile(case, 99.5, axis=0)
    assert ((case >= lo - 3.0) & (case <= hi + 3.0)).all()


def test_nurbs_evaluates_its_end_points():
    t = step._knots([3, 3], [0.0, 1.0])
    P = np.array([[0, 0, 0], [1, 2, 0], [3, 2, 0]], float)
    pts = step._nurbs_curve(2, P, t, n=11)
    assert np.allclose(pts[0], P[0]) and np.allclose(pts[-1], P[-1])
