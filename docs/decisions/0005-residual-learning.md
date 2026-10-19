# 0005: A hand-written gait as the baseline, residual learning on top

**Status:** Accepted in Sprint 4 (Oct 2026).
**Owner:** David (gait); Phase 2 training shared with Kai.

## Context

Craig's existing controller works on flat ground only and fails on
slopes, steps and rocks. That failure is the problem this project
exists to solve. Kai's Phase 1 runs showed that PPO learning to walk
from scratch needs millions of steps. Early on it finds that standing
still scores almost as well as walking. On a 4.3 kg robot with real
servos, a policy that flails while it learns is also a hardware risk.

## Decision

1. **Baseline:** a hand-written, engineered gait (`gait.py`). It moves
   feet, not joints: each foot follows a planned path and inverse
   kinematics on the CAD leg geometry gives the joint angles. It has
   four gaits (walk, trot, rubble, crouch), any direction, heading hold,
   and planted feet that move exactly with the floor so they do not slip.
2. **Learning:** the policy outputs corrections of up to +/-20 deg per
   servo, added to the gait's targets (residual RL). It is trained with
   PPO on randomised terrain, friction, payload and servo strength, with
   a curriculum from gentle to rough ground.

## Consequences

* A zero policy already walks, so training starts from competent
  behaviour and every episode produces useful data.
* The baseline gives a measurable bar. On flat ground it walks straight
  with no leg contact; on 15 deg slopes it walks up and down; it stalls
  on 50 mm steps and 80 mm rubble. The learned policy has to beat those
  numbers, which makes Phase 2 success criteria concrete.
* The gait is the safe fallback on hardware if the policy misbehaves.
