# 0002: Unblocking PyBullet on Apple Silicon

**Status:** Accepted in Sprint 3 (30 Sep 2026).
**Owner:** David.

## Context

Kai found that PyBullet would not install under Python 3.13: the upstream
release is about two years old and fails to compile on Apple Silicon. Only
Kai's machine could run the simulation, so nobody else could review or
extend it. This was a blocker on the critical path.

## Options

1. Downgrade everyone to an older Python. This works, but fragments the
   team's environments, and Prab had already hit Python 3.14 issues.
2. Move the simulation to MuJoCo, which has maintained wheels for every
   platform.
3. Find a maintained build of PyBullet.

## What we did

David evaluated option 2 properly before deciding. He built a parametric
MuJoCo model generator from the robot's dimensions, a static MJCF
validator, and a headless diagnostic harness that reports stance
stability, ground contact, servo load and gait performance without a
display (`simulation/mujoco-evaluation/`). The harness found three
geometry faults in the model under evaluation:

* adjacent legs overlapping in their sweep range
* leg-to-body proportions far outside a real arachnid's
* a velocity jump at the swing-to-stance transition

All three were fixed and verified numerically.

He kept looking for a lower-cost fix and found **`pybullet-arm64`**, a
maintained fork with prebuilt Python 3.13 Apple Silicon wheels. It is a
drop-in replacement that needs no code changes.

## Decision

Adopt `pybullet-arm64` for the PyBullet code (documented in
`simulation/pybullet/README.md`) and do not migrate frameworks at that
point. Keep the MuJoCo evaluation as documented evidence.

## Consequences

* The whole team could run Kai's simulation within a day.
* The evaluation was not wasted. When the CAD arrived two weeks later,
  the MuJoCo tooling and its diagnostics became the basis of the
  CAD-calibrated model ([0004](0004-cad-calibrated-mujoco.md)).
