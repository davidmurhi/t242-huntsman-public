# Testing and continuous integration

Every push and pull request runs the [CI workflow](../.github/workflows/ci.yml).
A change merges to `main` only when it is green.

## What runs

| Job | What it proves |
|---|---|
| **MuJoCo simulation** (Python 3.11 and 3.12) | the robot builds from the CAD and passes its build checks (`spider.py --save`); a 30 s headless walk goes straight with no leg contact (`walk.py --check`); the test suite below passes |
| **CAD extraction is reproducible** | re-reading V Source's Inventor files (`hardware/cad/inventor`) reproduces the committed geometry to within 0.05 mm; the STEP reader builds the battery correctly |
| **PyBullet simulation and environment** | Kai's URDF and Prab's Gymnasium environment and PWM mapping |
| **Operator dashboard** | the SvelteKit app type-checks and builds |
| **Python lint** | no syntax errors or undefined names anywhere under `simulation/` and `tools/` |

Failures appear as annotations on the pull request, so the cause is
visible without opening the log.

## Test suites

`simulation/mujoco/tests/` (54 tests)

| File | Covers |
|---|---|
| `test_cad_model.py` | mass equals the weight sheet (4.326 kg); 8 legs and 24 servos in the shared order; build checks; leg clearances; servo torque margin; control ranges in radians |
| `test_gait.py` | inverse kinematics round trips; targets always within joint limits; feet planted (6 walking, 4 trotting); planted feet do not slip; speed caps; comes to rest with feet home; e-stop freezes every servo |
| `test_terrain.py` | heights in range; flat start pad; same seed, same ground; difficulty scales; the floor is swapped for the terrain |
| `test_controller_protocol.py` | drive, hold, gait and e-stop messages; dead-man timeout; a real WebSocket session: handshake, drive, telemetry back, and the robot stopping when the connection drops; bad messages ignored; drive pad page served |
| `test_physics.py` | in MuJoCo: stands level; walks 10 s straight with no leg contact; backwards, sideways and turning; trot, rubble and crouch gaits; every terrain loads and runs |
| `test_rl_env.py` | the Gymnasium API (86 observations, 24 actions); a zero policy (the hand gait) walks and does not fall |
| `test_step_reader.py` | the battery STEP: five colours, real dimensions, curved faces on their surfaces, NURBS evaluation |
| `test_cad_reproducible.py` | the extraction comparison above |

`simulation/pybullet/tests/` (10 tests): URDF joints, limits, masses and
mirroring; environment spaces, reset, stepping, action scaling; PWM
mapping and limits from the URDF.

## Running them

```bash
pip install -r requirements.txt     # from the repository root
python -m pytest                    # both suites
python -m pytest simulation/mujoco/tests -k gait -v
```

Tests that need an optional package (mujoco, pybullet, gymnasium, the CAD
readers) are skipped, not failed, when it is missing. CI installs
everything.

## Beyond unit tests

* `python walk.py --check` and `python train.py --check` (in
  `simulation/mujoco`) are the quick end-to-end checks to run after any
  gait or model change.
* `diagnostics/` holds the contact and actuator probes used while tuning.
* Phase 2 adds a fixed-seed evaluation suite comparing the learned
  policy with the baseline on every terrain, and a hardware test plan
  for each robot session (see the [Phase 2 plan](sprints/phase2_plan.md)).
