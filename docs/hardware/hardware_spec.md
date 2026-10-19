# Hardware Specification — Project Huntsman
**T242 | IFB398 26se2 | Confirmed with Craig Schneider, V Source Development**

---

## Control Architecture

```
[Raspberry Pi 5]
       │  UART
       ▼
   [ESP32]
  ┌────────────────────────┐
  │  24× PWM servo channels │
  └────────────────────────┘
       │
  8 legs × 3 joints = 24 servos
  - Hip   (yaw,   Z-axis) × 8
  - Knee  (pitch, X-axis) × 8
  - Ankle (pitch, X-axis) × 8
```

---

## Component Mass Budget

| Component | Part | Mass |
|---|---|---|
| SBC | Raspberry Pi 5 (4GB) | 37g |
| MCU | ESP32 38-pin DevKit | 10g |
| Battery | 3S LiPo 2200mAh | 180g |
| Distribution board | Power dist / BEC | 20g |
| Wiring | JST + Dupont | 50g |
| Shell / chassis | ABS 3D print | 120g |
| Mounting | M2/M3 standoffs + screws | 30g |
| **Body total** | | **447g** |
| 24× Servos | MG90S @ ~9g | 216g |
| 8× Leg links (3D print) | ~68g per leg | ~544g |
| **Robot total** | | **~1,207g** |

---

## Leg Geometry

Derived from Craig's CAD and confirmed hardware dimensions.

| Segment | Length | Angle | Notes |
|---|---|---|---|
| Coxa | 80mm | 30° forward from lateral | Hip yaw; gives front-leg reach without body collision |
| Femur | 140mm | 10° outward lean | Knee pitch; natural wide stance |
| Tibia | 160mm | ~15.8° inward | Ankle pitch; foot lands under CoM |

**Result:** foot position under the robot's centre of mass → lateral stability without needing active balance compensation.

### L1 (front-left) attachment point
- Body pad: x=+0.135m, y=+0.110m, z=+0.010m
- Coxa end: [+0.175, +0.179, +0.010]
- Femur end: [+0.175, +0.203, -0.128]
- **Foot:** [+0.175, +0.159, -0.282] world frame
- **Ground clearance in home pose:** ~0.247m

---

## Servo Specification

| Property | Value |
|---|---|
| Model | MG90S (or equivalent metal-gear) |
| Operating voltage | 4.8–6.0V (5V BEC off LiPo) |
| Stall torque | ~1.8 kg·cm @ 4.8V |
| Speed | ~0.1 s/60° |
| Position range | ±90° (180° total) |
| PWM | 50Hz, 500–2400μs |

### Joint limits in simulation

| Joint | Min | Max |
|---|---|---|
| Hip (yaw) | −0.6 rad | +0.6 rad |
| Knee (pitch) | −0.3 rad | +1.4 rad |
| Ankle (pitch) | −0.6 rad | +0.6 rad |

---

## Power Budget

| Component | Current (est.) | Power |
|---|---|---|
| Raspberry Pi 5 | 2.5A @ 5V | 12.5W |
| ESP32 | 0.25A @ 3.3V | 0.8W |
| 24× Servos (active) | 24 × 0.5A avg | 60W |
| **Total** | | **~73W** |

3S LiPo 2200mAh = 24.4Wh → **~20 min runtime** at full locomotion load.

---

## Communication

- Pi 5 → ESP32: UART at 115200 baud
- Message: 24 × int16 servo targets (0.01° resolution)
- Control loop: 50Hz (matches servo PWM refresh)
- Target latency: <5ms Pi5→ESP32→servo
