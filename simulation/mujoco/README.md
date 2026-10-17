# Huntsman in MuJoCo, built from V Source's CAD

The simulated robot is generated from V Source's own Inventor CAD
(`Huntsman.zip`, sent by Ronan Donahoe on 15 Oct 2026), so its shape, joint
positions and masses match the robot they are building.

```
pip install mujoco numpy
python3 spider.py --save      # build spider.xml and print the build report
mjpython walk.py              # watch it walk (macOS; plain python elsewhere)
python3 walk.py --check       # walk headless for 20 s and report
```

## How the model is made

```
Huntsman/*.ipt, *.iam  --cad/extract_cad.py-->  cad/huntsman_cad.json  --spider.py-->  spider.xml
   (Ronan's CAD)                                  (part shapes, joints)                 (MuJoCo)
```

`cad/inventor.py` reads Inventor 2025+ files directly: each data segment is
Zstandard-compressed, part shapes come from the ACIS solid inside, and
assembly placements are sparse 4x4 matrices. `cad/extract_cad.py` composes
`mounted servo` -> `full leg` -> `bot`, finds the hip, knee and ankle axes
from the servo horn frames, and writes every part as a convex mesh in the
simulator's frame. When the CAD changes, run it again:

```
pip install olefile zstandard numpy scipy
python3 cad/extract_cad.py path/to/Huntsman/
```

## What comes from the CAD and what does not

| From the CAD | Not in the CAD pack |
|---|---|
| body plate 349 x 230 x 5 mm | standing pose (taken from Ronan's photos) |
| hip positions, 62 mm apart along each side | servo gains |
| leg fans: 22.7 deg front, 0 middle, 30 deg rear | `case2` payload box (sized from the render) |
| femur 106 mm, acrylic lower leg 148 mm | `rear rail v2`, `Makitabattery` mount |
| every part's shape | |
| masses: Ronan's weight sheet, 4.326 kg | |

## How it walks (measured in MuJoCo 3.3)

| over 30 s on flat ground | |
|---|---|
| distance | 3.0 m forward (10 cm/s), battery at the back |
| off its line | under 2 cm |
| body tilt | under 1 deg |
| leg-to-leg contact | none |
| servos at their torque limit | about 3% of the time |

The gait is a paired wave: four beats, two legs up per beat, six planted.
The two middle legs on a side always step together (their brackets are
6 mm apart), and front/rear pairs step together so their sideways pushes
cancel. Heading hold steers back to the starting line by lengthening one
side's stride, as the real robot will with its IMU; the legs are one
assembly repeated rather than mirrored, so without it the robot crabs
about 6% sideways.

**Servo targets are radians.** MuJoCo converts joint ranges to radians
under `angle="degree"` but not actuator control ranges, so `walk.py`
converts every target. Earlier versions sent degrees, which drove every
servo into its stop.

## Findings for V Source

* The two middle legs on each side have hips 62 mm apart and femur brackets
  56 mm wide, leaving about 6 mm. Any gait that swings them in opposite
  directions makes them collide, so the gait moves them together. Worth
  confirming against the built robot, whose middle legs look more splayed
  in the photos.
* Standing on six feet the knee servo holds about 1.1 N.m, 20% of the
  DSC55MG's 5.4 N.m stall rating; on four feet about 30%.

## Files

| file | purpose |
|---|---|
| `spider.py` | builds the robot from `cad/huntsman_cad.json`, prints the build report and checks |
| `walk.py` | paired wave gait with heading hold: two legs up per beat, middle legs always together |
| `cad/` | Inventor reader, extractor and the extracted geometry |
| `diagnostics/` | contact, pose and actuator probes used while tuning |
