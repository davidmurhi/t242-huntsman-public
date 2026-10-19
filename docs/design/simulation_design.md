# Simulation Design Decisions — Project Huntsman
**T242 | IFB398 26se2**

---

## Simulator Selection: MuJoCo over PyBullet

We began with PyBullet (URDF-based, Sprints 2–4) because of its accessible Python API. By Sprint 4 we identified two problems that PyBullet's contact model couldn't solve:

1. **Foot slipping during stance** — PyBullet's contact friction model is less physically accurate for small high-friction contact patches (our 30mm foot spheres).
2. **Contact instability at gait transitions** — rapid foot lift-off/strike caused energy spikes in the simulation that training couldn't converge through.

We switched to MuJoCo for Sprint 5 because:
- More accurate contact physics (convex contact, constraint-based solver)
- Used in He et al. 2024 and most SOTA legged locomotion papers
- Native Gymnasium integration via `gymnasium[mujoco]`
- MJCF XML format is more maintainable than URDF for complex articulated bodies

The PyBullet environment (Kai's work, Sprints 2–4) remains in the repo as the primary RL output for Phase 1. The MuJoCo rebuild (David, Sprint 5) is the foundation for Phase 2 RL training.

---

## Why We Rebuilt from Scratch (Sprint 5)

The v1 prototype geometry was eyeballed from Craig's sketches, not derived from measured hardware. This caused:

- Foot placement off-centre laterally → lateral instability RL couldn't compensate
- Leg segments too short → unrealistic torque requirements
- Mass model wrong → body inertia didn't match the real robot

He et al. contact-ratio feedback (Sprint 4) improved stability but couldn't fix geometry. The only correct solution was to rederive everything from Craig's confirmed part dimensions.

**Principle:** every number in the MJCF comes from a measured or calculated hardware value, not from trial-and-error.

---

## Simulation Build Sequence

Each step is validated before the next is started.

| Step | Output | Validation criteria |
|---|---|---|
| 1 | `huntsman_body.xml` | Settles at z=0.035m, zero tilt, no penetration |
| 2 | `huntsman_l1.xml` | Foot under CoM, joints in range, sensor reading, no NaN in 500 steps |
| 3 | `huntsman_full.xml` | All 8 legs symmetric, static stability |
| 4 | Gait controller | Forward locomotion >0.5 m/s, wave pattern correct |
| 5 | Gymnasium wrapper | RL training starts without crash |
| 6 | PPO training | Policy converges, contact ratio β > 0.8 |

---

## MJCF Key Choices

**Integrator: RK4**  
Better energy conservation at dt=0.002s (500Hz). Required for stable contact physics during high-frequency foot strikes.

**Solver: Newton, tolerance 1e-8**  
Reliable constraint resolution for up to 8 simultaneous foot contacts. Euler with default tolerance diverges during multi-contact stances.

**Joint damping:** 0.8 (hip/knee), 0.5 (ankle). Approximates servo gear friction without overdamping.

**Foot geometry:** sphere radius=30mm, friction μ=2.0 torsional=0.2. Avoids mesh-mesh contact ambiguity; gives clean touch sensor signals.

**Actuators:** position-controlled (kp=40 hip, 55 knee, 30 ankle). Matches real servo behaviour — ESP32 sends position targets, not torques.

---

## RL Reward (He et al. 2024)

```
R = v_forward - 0.6 * |v_lateral| - 0.01 * ||torque||²
contact_bonus = 0.1 * (feet_grounded / 8)
```

The contact-ratio term discourages mid-air stumbling without explicitly programming a gait pattern. The robot learns to keep feet grounded as a natural consequence of the reward.

---

## Coordinate System

MuJoCo right-handed: X forward, Y left, Z up. Body origin at geometric centre of torso. Floor at Z=0.

---

## References

- He et al. (2024). *Agile But Safe: Learning Collision-Free High-Speed Legged Locomotion*. [arxiv:2409.09473](https://arxiv.org/abs/2409.09473)  
- Todorov et al. (2012). MuJoCo: A physics engine for model-based control. *IROS 2012*.  
- MuJoCo docs: https://mujoco.readthedocs.io
