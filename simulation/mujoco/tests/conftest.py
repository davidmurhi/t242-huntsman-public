"""Shared set-up for the MuJoCo simulation tests.

The pure-Python tests (CAD model, gait, terrain, controller protocol,
STEP reader) need only numpy, scipy and pillow. The physics tests need
the mujoco package and are skipped without it.
"""

import os
import sys

import pytest

SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(SIM))
if SIM not in sys.path:
    sys.path.insert(0, SIM)


@pytest.fixture(scope="session")
def spider_xml_path():
    """Build spider.xml from the CAD once per test session."""
    import spider
    path = os.path.join(SIM, "spider.xml")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(spider.build_model())
    return path


@pytest.fixture(scope="session")
def model(spider_xml_path):
    mujoco = pytest.importorskip("mujoco")
    return mujoco.MjModel.from_xml_path(spider_xml_path)
