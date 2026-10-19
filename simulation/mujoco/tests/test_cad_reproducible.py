"""The simulation's geometry (cad/huntsman_cad.json) must be exactly
what cad/extract_cad.py produces from V Source's Inventor files in
hardware/cad/inventor. Skipped unless the CAD readers are installed
(pip install olefile zstandard opencv-python-headless)."""

import json
import os
import subprocess
import sys

import numpy as np
import pytest

SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(SIM))

pytest.importorskip("olefile")
pytest.importorskip("zstandard")
pytest.importorskip("cv2")


TOL = 5e-5   # metres (and radians, unitless for texture coordinates)


def _same(a, b, path="$"):
    if isinstance(a, dict):
        assert set(a) == set(b), f"{path}: keys differ {set(a) ^ set(b)}"
        for k in a:
            _same(a[k], b[k], f"{path}.{k}")
    elif isinstance(a, list) and a and not isinstance(a[0], (int, float)):
        assert len(a) == len(b), f"{path}: length {len(a)} != {len(b)}"
        for i, (x, y) in enumerate(zip(a, b)):
            _same(x, y, f"{path}[{i}]")
    elif isinstance(a, (list, int, float)) and not isinstance(a, bool):
        x, y = np.asarray(a, float), np.asarray(b, float)
        assert x.shape == y.shape, f"{path}: shape {x.shape} != {y.shape}"
        worst = float(np.abs(x - y).max()) if x.size else 0.0
        # 0.05 mm: well below anything the simulation could notice
        assert worst <= TOL, f"{path}: differs by up to {worst:.3g}"
    else:
        assert a == b, f"{path}: {str(a)[:80]} != {str(b)[:80]}"


def test_extracted_geometry_matches_committed(tmp_path):
    folder = os.path.join(REPO, "hardware", "cad", "inventor")
    committed = os.path.join(SIM, "cad", "huntsman_cad.json")
    with open(committed, encoding="utf-8") as fh:
        want = json.load(fh)
    backup = committed + ".bak"
    os.replace(committed, backup)
    try:
        subprocess.run([sys.executable, os.path.join(SIM, "cad", "extract_cad.py"),
                        folder], check=True, cwd=SIM, capture_output=True)
        with open(committed, encoding="utf-8") as fh:
            got = json.load(fh)
    finally:
        os.replace(backup, committed)
    _same(got, want)
