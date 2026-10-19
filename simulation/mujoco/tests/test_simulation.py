"""
Project Huntsman — MuJoCo Simulation Unit Tests
T242 | IFB398 26se2 | V Source Development

Run with:
    python -m pytest simulation/mujoco/tests/ -v

Tests validate that the MJCF models load correctly, physics are stable,
and key geometric/mass properties match design specifications derived from
Craig's confirmed hardware.
"""

import os
import numpy as np
import pytest

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
MJCF_DIR  = os.path.join(REPO_ROOT, "simulation", "mujoco", "mjcf")

BODY_XML = os.path.join(MJCF_DIR, "huntsman_body.xml")
L1_XML   = os.path.join(MJCF_DIR, "huntsman_l1.xml")


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def mujoco():
    try:
        import mujoco as mj
        return mj
    except ImportError:
        pytest.skip("MuJoCo not installed — skipping simulation tests")


@pytest.fixture(scope="module")
def body_model(mujoco):
    if not os.path.exists(BODY_XML):
        pytest.skip(f"Model not found: {BODY_XML}")
    return mujoco.MjModel.from_xml_path(BODY_XML)


@pytest.fixture(scope="module")
def l1_model(mujoco):
    if not os.path.exists(L1_XML):
        pytest.skip(f"Model not found: {L1_XML}")
    return mujoco.MjModel.from_xml_path(L1_XML)


# ---------------------------------------------------------------------------
# Step 1: Body-only tests
# ---------------------------------------------------------------------------

class TestBodyModel:
    """Tests for huntsman_body.xml — Step 1 (body only)."""

    def test_loads_without_error(self, body_model):
        assert body_model is not None

    def test_body_count(self, body_model):
        # nbody includes world body
        assert body_model.nbody >= 2, f"Expected ≥2 bodies, got {body_model.nbody}"

    def test_body_mass(self, mujoco, body_model):
        """Torso mass should match hardware budget: 0.447 kg."""
        data = mujoco.MjData(body_model)
        mujoco.mj_forward(body_model, data)
        torso_mass = body_model.body_mass[1]
        assert abs(torso_mass - 0.447) < 0.01, \
            f"Torso mass {torso_mass:.4f} kg ≠ 0.447 kg"

    def test_settles_flat(self, mujoco, body_model):
        """Body dropped from rest should settle with near-zero tilt."""
        data = mujoco.MjData(body_model)
        mujoco.mj_resetData(body_model, data)
        for _ in range(500):
            mujoco.mj_step(body_model, data)
        quat = data.qpos[3:7]
        tilt = np.sqrt(quat[1]**2 + quat[2]**2)
        assert tilt < 0.05, f"Body not settling flat — tilt={tilt:.4f}"

    def test_settles_at_correct_height(self, mujoco, body_model):
        """Body z should settle near 0.035 m (half body height above floor)."""
        data = mujoco.MjData(body_model)
        mujoco.mj_resetData(body_model, data)
        for _ in range(500):
            mujoco.mj_step(body_model, data)
        z = data.qpos[2]
        assert 0.025 < z < 0.060, f"Body z={z:.4f} m outside expected range"

    def test_no_deep_interpenetration(self, mujoco, body_model):
        """After settling, contact penetration should be minimal."""
        data = mujoco.MjData(body_model)
        mujoco.mj_resetData(body_model, data)
        for _ in range(500):
            mujoco.mj_step(body_model, data)
        for i in range(data.ncon):
            dist = data.contact[i].dist
            assert dist > -0.005, f"Contact {i} penetration {dist:.5f} m too deep"


# ---------------------------------------------------------------------------
# Step 2: Body + L1 leg tests
# ---------------------------------------------------------------------------

class TestL1Model:
    """Tests for huntsman_l1.xml — Step 2 (body + one leg)."""

    def test_loads_without_error(self, l1_model):
        assert l1_model is not None

    def test_body_count(self, l1_model):
        """world + torso + coxa + femur + tibia = 5."""
        assert l1_model.nbody == 5, f"Expected 5 bodies, got {l1_model.nbody}"

    def test_actuator_count(self, l1_model):
        """Exactly 3 actuators: hip, knee, ankle."""
        assert l1_model.nu == 3, f"Expected 3 actuators, got {l1_model.nu}"

    def test_sensor_count(self, l1_model):
        """Exactly 1 touch sensor on L1 foot."""
        assert l1_model.nsensor == 1, f"Expected 1 sensor, got {l1_model.nsensor}"

    def test_joints_in_range(self, mujoco, l1_model):
        """All joint angles in home keyframe within joint limits."""
        data = mujoco.MjData(l1_model)
        mujoco.mj_resetDataKeyframe(l1_model, data, 0)
        mujoco.mj_forward(l1_model, data)
        for i in range(l1_model.njnt):
            if l1_model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE:
                continue
            q  = data.qpos[l1_model.jnt_qposadr[i]]
            lo = l1_model.jnt_range[i, 0]
            hi = l1_model.jnt_range[i, 1]
            assert lo <= q <= hi, \
                f"Joint {i} qpos={q:.4f} outside [{lo:.4f}, {hi:.4f}]"

    def test_foot_touch_sensor_non_negative(self, mujoco, l1_model):
        """Touch sensor reading should be non-negative."""
        data = mujoco.MjData(l1_model)
        mujoco.mj_resetDataKeyframe(l1_model, data, 0)
        for _ in range(200):
            mujoco.mj_step(l1_model, data)
        assert data.sensordata[0] >= 0

    def test_foot_height_reasonable(self, mujoco, l1_model):
        """Foot should be near ground level in home pose."""
        data = mujoco.MjData(l1_model)
        mujoco.mj_resetDataKeyframe(l1_model, data, 0)
        mujoco.mj_forward(l1_model, data)
        foot_id = mujoco.mj_name2id(l1_model, mujoco.mjtObj.mjOBJ_GEOM, "l1_foot")
        if foot_id == -1:
            pytest.skip("l1_foot geom not found")
        foot_z = data.geom_xpos[foot_id, 2]
        assert foot_z < 0.10, f"Foot z={foot_z:.4f} seems too high"

    def test_stable_no_nan(self, mujoco, l1_model):
        """Simulation should not diverge (NaN/Inf) over 500 steps."""
        data = mujoco.MjData(l1_model)
        mujoco.mj_resetDataKeyframe(l1_model, data, 0)
        for _ in range(500):
            mujoco.mj_step(l1_model, data)
        assert not np.any(np.isnan(data.qpos)), "NaN in qpos"
        assert not np.any(np.isinf(data.qpos)), "Inf in qpos"

    def test_actuator_responds_to_control(self, mujoco, l1_model):
        """Non-zero control input should produce non-zero actuator force."""
        data = mujoco.MjData(l1_model)
        mujoco.mj_resetDataKeyframe(l1_model, data, 0)
        mujoco.mj_forward(l1_model, data)
        data.ctrl[0] = 0.3
        mujoco.mj_step(l1_model, data)
        assert data.actuator_force[0] != 0.0


# ---------------------------------------------------------------------------
# Cross-model consistency
# ---------------------------------------------------------------------------

class TestModelConsistency:

    def test_l1_has_more_bodies(self, body_model, l1_model):
        assert l1_model.nbody > body_model.nbody

    def test_body_only_has_no_actuators(self, body_model):
        assert body_model.nu == 0

    def test_l1_has_more_mass(self, mujoco, body_model, l1_model):
        assert sum(l1_model.body_mass) > sum(body_model.body_mass)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
