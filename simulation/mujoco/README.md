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
| leg fans: ~30 deg front, 0 middle, ~23 deg rear | `case2` payload box (sized from the render) |
| femur 106 mm, acrylic lower leg 148 mm | `rear rail v2`, `Makitabattery` mount |
| every part's shape | |
| masses: Ronan's weight sheet, 4.326 kg | |

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
| `walk.py` | paired wave gait: two legs up per beat, middle legs always together |
| `cad/` | Inventor reader, extractor and the extracted geometry |
| `diagnostics/` | contact, pose and actuator probes used while tuning |
