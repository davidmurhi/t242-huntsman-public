# 0006: Repository structure, branching and quality gates

**Status:** Accepted in Sprint 4 (19 Oct 2026). Revises the Sprint 1
five-branch layout.
**Owner:** David.

## Context

The Sprint 1 layout gave each area a long-lived branch (`simulation`,
`dashboard`, `firmware`, `dev`). By October, work had spread across
those branches and several personal ones. Nothing ran the code
automatically, and Phase 2 needs four people changing shared code safely.

## Decision

* **One `main`**, always working. All Phase 1 branches were merged into
  it with their history preserved.
* **Short-lived branches** per issue, named `area/short-description`,
  merged by pull request with one review.
* **Folders by component:** `simulation/mujoco`, `simulation/pybullet`,
  `dashboard`, `firmware`, `hardware`, `docs`, `research`.
* **CI on every push and pull request**
  ([`.github/workflows/ci.yml`](../../.github/workflows/ci.yml)):
  * build the robot from the CAD and run its checks
  * a 30 s headless walk check
  * unit and physics tests on Python 3.11 and 3.12
  * CAD re-extraction compared with the committed geometry
  * PyBullet environment tests
  * dashboard type-check and build
  * lint
* **Templates:** user stories, tasks and bugs; a pull request template
  carrying the definition of done; CODEOWNERS routing reviews to each
  area's owner.

## Consequences

* "It works on my machine" is replaced by a green tick on every change.
* Work is traceable from user story to pull request to test.
