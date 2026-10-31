# How the team works

Team T242 uses Scrum-style sprints of one to two weeks, run on a Kanban board.
The cadence follows the two fixed rhythms of the project: fortnightly
tutor meetings (QUT) and fortnightly partner meetings (V Source, in the
off week). Each sprint therefore ends with a review in front of one of
them.

## Roles

| Person | Primary ownership | Scrum role |
|---|---|---|
| David Murhi | team lead, project management, partner and tutor communication; MuJoCo simulation and gait | Scrum master; product owner proxy for V Source |
| Kai Langley | ML and simulation: PyBullet prototype, PPO training | developer |
| Prabhanjan Muthukannan | hardware and embedded: Pi 5, ESP32, servo mapping, Gymnasium environment | developer |
| Tom Huynh | operator interface: SvelteKit dashboard, telemetry protocol | developer |
| Craig Schneider (V Source) | priorities, acceptance, hardware | product owner (customer) |

Roles are primary ownership, not silos (team decision, 18 Aug). David
works across all areas, and anyone can pick up any item.

## Cadence

| Event | When | Where | Output |
|---|---|---|---|
| Sprint planning | first day of each sprint | Discord call | sprint goal; items pulled onto the board with owners |
| Weekly sync | Mondays 10:00 | Discord | progress, blockers, re-plan |
| Async stand-up | as needed | Discord channel | blockers posted immediately, not saved for meetings |
| Partner review | fortnightly, off-week from tutor, after 6 pm | Teams or Discord with Craig | demo, questions, priorities; minutes in `docs/meetings/` |
| Tutor review | fortnightly | QUT tutorial with Sarah | written summary (past fortnight, fortnight ahead, to discuss) |
| Retrospective | end of each sprint | Discord | one keep, one change; recorded in [sprint records](../sprints/phase1.md) |

## Board

GitHub Projects board (**T242 Huntsman Board**) with columns Backlog,
Ready, In progress, In review and Done. Every item is an issue made from
a template:

* **User story:** "As a ..., I want ..., so that ...", with acceptance
  criteria, area, story points and sprint.
* **Task:** a technical step or spike, linked to its story.
* **Bug:** what happened, steps to reproduce, environment.

Labels: `simulation`, `rl`, `hardware`, `dashboard`, `docs`, `testing`.
Estimates use story points (1, 2, 3, 5, 8, 13), sized relative to "add a
terrain type" = 3.

## Definition of ready

An item can enter a sprint when:

* it has acceptance criteria that can be checked
* its dependencies are available, or it is explicitly a spike to find out
* it is small enough to finish in one sprint (8 points or fewer;
  otherwise it is split)

## Definition of done

* Acceptance criteria met and demonstrated (numbers or screenshots in
  the PR).
* Code merged to `main` through a pull request reviewed by one teammate.
* CI green: tests, model build checks, walk check, dashboard build, lint.
* Tests added or updated for the behaviour.
* README or docs updated if commands or behaviour changed.
* Partner-confidential files only under `hardware/`.

## Source control

* `main` is always working. Branch per issue: `sim/...`, `rl/...`,
  `hw/...`, `ui/...`, `docs/...`.
* Commit messages: `[area] what changed` (for example `[sim] heading hold
  for straight walking`).
* Pull requests use the template, which carries the definition of done.
  CODEOWNERS requests a review from the area's owner.
* See [CONTRIBUTING.md](../../CONTRIBUTING.md) for commands.

## Communication

| Channel | Use |
|---|---|
| Discord (Project Huntsman server; Craig is a member) | day-to-day, blockers, partner questions |
| GitHub issues and pull requests | anything that needs tracking or review |
| OneDrive (T242 IFB398 26se2) | unit documents, work logs, meeting notes |
| Email | formal communication with the teaching team |

Each member keeps an individual work log (about 15 hours a week, agreed
18 Aug). The team research log records sources as they are used
(`research/`).
