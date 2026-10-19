# Changelog

## Phase 1 release (October 2026)

### Added
* CAD-calibrated MuJoCo simulation (`simulation/mujoco`):
  * reads V Source's Inventor CAD and the battery STEP directly
  * build checks against the weight sheet
  * foot-path gait with walk, trot, rubble and crouch
  * live driving (keyboard, drive pad, dashboard WebSocket)
  * rough terrains
  * residual-RL Gymnasium environment and PPO trainer
* PyBullet prototype v6 with keyboard driving and gears (Kai);
  telemetry bridge and PWM mapping (Prab).
* SvelteKit operator console (Tom).
* `hardware/`: V Source's CAD, battery files, part weights, photos, video.
* 64 automated tests; CI for both simulations, CAD reproducibility, the
  dashboard and lint; issue and PR templates; CODEOWNERS; Dependabot.
* Documentation:
  * project definition, risks and ethics
  * Phase 1 sprint record and Phase 2 plan
  * backlog and way of working
  * decision records, architecture, meetings and testing
  * diagrams and renders

### Changed
* All branches consolidated into `main`; components moved into
  `simulation/`, `dashboard/`, `firmware/` and `hardware/`
  ([ADR 0006](docs/decisions/0006-repository-and-quality.md)).

### Removed
* An interim first-principles MuJoCo rebuild whose specification
  (servos, mass, power) did not match V Source's CAD. It remains in
  the git history.

## Sprint history
See [docs/sprints/phase1.md](docs/sprints/phase1.md).
