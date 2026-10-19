# Contributing

## Set up

```bash
git clone https://github.com/davidmurhi/T242-Project-Huntsman.git
cd T242-Project-Huntsman
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python -m pytest                       # everything should pass or skip
```

The dashboard needs Node 20 or newer: `cd dashboard && npm ci && npm run dev`.

## Workflow

1. Pick an issue from the board, or create one from a template (user
   story, task or bug).
2. Branch from `main`: `sim/heading-hold`, `rl/curriculum`,
   `hw/servo-order`, `ui/estop-button`, `docs/...`.
3. Commit with the area prefix: `[sim] hold heading while walking straight`.
4. Push and open a pull request. The template lists the definition of
   done. CODEOWNERS requests a review from the area owner.
5. Merge when CI is green and one teammate has approved.

## Before you push

| Changed | Run |
|---|---|
| anything in `simulation/mujoco` | `python -m pytest simulation/mujoco/tests` |
| the gait or the model | `python spider.py --save` and `python walk.py --check` (no leg contact) |
| the RL environment | `python train.py --check` |
| `simulation/pybullet` | `python -m pytest simulation/pybullet/tests` |
| the dashboard | `npm run check && npm run build` |
| V Source's CAD files | `python cad/extract_cad.py ../../hardware/cad/inventor/`, then commit the new `huntsman_cad.json` |
| diagrams | `python tools/diagrams.py` |

## Conventions

* Python: standard library style, `ruff` clean for errors. Units are SI
  (metres, radians) inside the code; degrees only at the user interface.
* Servo order everywhere: legs `l1 r1 l2 r2 l3 r3 l4 r4`, each `hip knee
  ankle` (see `docs/media/diagrams/leg_map.svg`).
* Decisions with lasting consequences get an ADR in `docs/decisions/`.
* Partner files (CAD, photos, video) go in `hardware/` only.
