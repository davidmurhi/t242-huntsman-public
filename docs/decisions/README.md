# Architecture decision records

Short records of the decisions that shaped the project: the context at
the time, the options we weighed, what we chose and what it cost us. New
decisions get the next number. Superseded records stay, marked as such,
so the reasoning is never lost.

| # | Decision | Sprint | Status |
|---|---|---|---|
| [0001](0001-simulation-and-rl-stack.md) | PyBullet + Gymnasium + Stable-Baselines3 (PPO) for the first simulation | 1 | Superseded in part by 0004 |
| [0002](0002-pybullet-arm64.md) | Unblock Apple Silicon with `pybullet-arm64`; keep the MuJoCo evaluation | 3 | Accepted |
| [0003](0003-websocket-telemetry.md) | WebSockets with JSON for dashboard telemetry and commands | 2 | Accepted |
| [0004](0004-cad-calibrated-mujoco.md) | Build the simulation from V Source's CAD in MuJoCo; make it the Phase 2 platform | 4 | Accepted |
| [0005](0005-residual-learning.md) | Hand-written foot-path gait as the baseline; learn residual corrections on top | 4 | Accepted |
| [0006](0006-repository-and-quality.md) | One `main`, short-lived branches, tests and CI on every push | 4 | Accepted |
