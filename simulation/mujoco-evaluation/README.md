# MuJoCo evaluation

**T242 | Project Huntsman | David**

A second simulation of the spider, built in MuJoCo alongside Kai's PyBullet
one. This is an evaluation, not a replacement. It is kept here because the
work and the findings feed into the Phase 1 report.

## Why it exists

The upstream PyBullet release is two years old and will not build on Apple
Silicon under Python 3.13, so the simulation could only be run on Kai's
machine. MuJoCo was evaluated as a possible replacement.

That turned out not to be necessary. `pybullet-arm64` is a maintained fork
publishing prebuilt wheels for Python 3.13 on Apple Silicon, and it is a
drop-in replacement:

```
pip install pybullet-arm64
```

No code changes. Anyone on the team blocked by the build failure should
use that rather than switching simulators.

## Findings worth knowing, whichever simulator we use

**The servos were badly wrong.** The actuators were saturating on 99% of
frames, with up to 50 degrees of tracking error and joint speeds eleven
times what the gait asked for. The legs were not being driven, they were
being flung. This is a parameter problem, not a physics-engine problem, so
it is worth checking the PyBullet model for the same thing.

Fixed by raising torque limits, increasing damping to a critically damped
ratio, adding rotor inertia (`armature`), and easing the gait in from
standing instead of starting on a step command.

**Self-collision is a geometry problem, not a gait problem.** Neighbouring
legs kept clashing. Every commanded pose was collision-free when checked
with the physics frozen, so the gait was not driving them together. The
fix was to give each pair of legs its own angular wedge, enforced through
the joint limits in the model rather than in the gait code, so no gait can
violate it however it is written.

**Lifting the knee moves the foot sideways.** Because the tibia angles down
and outward, raising the knee swings the foot outward by more than it
lifts it. Worth knowing when designing any swing trajectory for this leg
layout.

## Running it

```
pip install mujoco numpy
```

```
python3 spider.py --check     # build report, no viewer
python3 spider.py --save      # write spider.xml
mjpython spider.py            # look at the body standing (macOS)
mjpython walk.py              # watch it walk
python3 walk.py --check       # walk it headless and report
```

On macOS MuJoCo needs `mjpython` for anything that opens a window.
Windows and Linux use plain `python`.

## The files

| file | what it does |
|---|---|
| `spider.py` | The robot. All dimensions are constants at the top; the model is generated from them, so when Craig's measurements arrive it is a matter of changing numbers rather than editing XML. |
| `walk.py` | The gait. Row by row, back pair first, two legs off the ground at a time. |

### Diagnostics

Written because tuning by eye kept giving wrong answers. Each one measures
something specific and prints a verdict.

| file | question it answers |
|---|---|
| `rest.py` | Standing still, is anything touching that should not be? |
| `poses.py` | Does the gait ever ask for a pose where the legs overlap? Physics frozen, so this isolates geometry from balance. |
| `tangle.py` | While walking, what touches what, how often, and how hard? |
| `contacts.py` | The same, but listing every geom and every contact position in full. |
| `probe.py` | Are the actuators following orders, or saturating? Where does a foot actually go when lifted? What was happening at the first collision? |
| `tune.py` | Sweeps gait settings and reports which ones walk cleanly. |
| `search.py` | Sweeps body shape as well as gait, rebuilding the robot each time. |

## Status

The body stands correctly on all eight feet and walks forward in a
straight line. Leg self-collision is resolved by the joint wedges.

Dimensions are placeholders until Craig's measurements arrive, so nothing
here should be read as representing the real robot yet.
