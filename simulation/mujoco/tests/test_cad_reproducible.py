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


def _same(a, b, path="$"):
    if isinstance(a, dict):
        assert set(a) == set(b), f"{path}: keys differ"
        for k in a:
            _same(a[k], b[k], f"{path}.{k}")
    elif isinstance(a, list) and a and not isinstance(a[0], (int, float)):
        assert len(a) == len(b), f"{path}: length differs"
        for i, (x, y) in enumerate(zip(a, b)):
            _same(x, y, f"{path}[{i}]")
    elif isinstance(a, (list, int, float)) and not isinstance(a, bool):
        np.testing.assert_allclose(np.asarray(a, float), np.asarray(b, float),
                                   rtol=0, atol=1e-6, err_msg=path)
    else:
        assert a == b, path


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
