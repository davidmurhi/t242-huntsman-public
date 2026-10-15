
# Controls 
#   UP / W       walk forward
#   DOWN / S     walk backward
#   LEFT / A     turn left
#   RIGHT / D    turn right
#   E            shift up a gear (faster)
#   Q            shift down a gear (slower)
#   SPACE        stop

import pybullet as p
import pybullet_data
import time
import os

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)
# PyBullet binds W/A/S/D 
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)

plane_id = p.loadURDF("plane.urdf")
urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "spider8_v5_longbody_3joint.urdf")
robot_id = p.loadURDF(urdf_path, [0, 0, 0.35], useFixedBase=False)

p.changeDynamics(plane_id, -1, lateralFriction=1.5)
for j in range(p.getNumJoints(robot_id)):
    p.changeDynamics(robot_id, j, lateralFriction=1.5)

joint_indices = {}
for j in range(p.getNumJoints(robot_id)):
    joint_indices[p.getJointInfo(robot_id, j)[1].decode("utf-8")] = j

leg_pairs = [("l1", "r1"), ("l2", "r2"), ("l3", "r3"), ("l4", "r4")]
n_groups = len(leg_pairs)

# Gait settings (same values as the v5 walk script)
HIP_AMPLITUDE = 0.35
KNEE_LIFT = 0.6
KNEE_STANCE = 0.15
ANKLE_LIFT = 0.35
ANKLE_STANCE = 0.05
CYCLE_PERIOD = 2.0
SWING_FRACTION = 0.25

# Gears: (stride scale, step rate). Faster gears take longer steps AND step more often.
# lower gears for rough terrian and the higher gears for straight line speed 
#7 gears this is for moving over objects or straight line speed
GEARS = [(0.6, 0.6), (0.8, 0.8), (1.0, 1.0), (1.2, 1.3), (1.4, 1.6), (1.6, 1.9), (1.8, 2.2)]
gear = 2   # index into GEARS, starts in gear 3

# Control settings
TURN_GAIN = 0.6       # how hard the two sides differ when turning (higher = sharper turn)
MAX_STRIDE = 1.9      # cap on stride scale so the hip stays inside its +/-0.7 rad limit
SPEED_SMOOTHING = 0.04   # lower = slower, slower acclerartion for smoothness  
TURN_SMOOTHING = 0.08
FOLLOW_CAMERA = True

DT = 1.0 / 240.0


def key_down(keys, *codes):
    return any(c in keys and keys[c] & p.KEY_IS_DOWN for c in codes)


def key_pressed(keys, *codes):
    return any(c in keys and keys[c] & p.KEY_WAS_TRIGGERED for c in codes)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def show_gear():
    text = f"Gear {gear + 1}/{len(GEARS)}"
    return p.addUserDebugText(text, [0, 0, 0.5], textColorRGB=[1, 1, 0], textSize=1.6,
                              parentObjectUniqueId=robot_id, replaceItemUniqueId=hud_id)


print("Controls: UP/W forward, DOWN/S back, LEFT/A turn left, RIGHT/D turn right, "
      "E gear up, Q gear down, SPACE stop")

hud_id = -1
hud_id = show_gear()

speed = 0.0   # smoothed forward command, -1..1
turn = 0.0    # smoothed turn command, -1 (right) +1 (left)
base_phase = 0.0

while p.isConnected():
    keys = p.getKeyboardEvents()

    if key_pressed(keys, ord("e")) and gear < len(GEARS) - 1:
        gear += 1
        hud_id = show_gear()
        print(f"Gear {gear + 1}/{len(GEARS)}")
    if key_pressed(keys, ord("q")) and gear > 0:
        gear -= 1
        hud_id = show_gear()
        print(f"Gear {gear + 1}/{len(GEARS)}")
    stride_scale, step_rate = GEARS[gear]

    target_speed = 0.0
    target_turn = 0.0
    if key_down(keys, p.B3G_UP_ARROW, ord("w")):
        target_speed += 1.0
    if key_down(keys, p.B3G_DOWN_ARROW, ord("s")):
        target_speed -= 1.0
    if key_down(keys, p.B3G_LEFT_ARROW, ord("a")):
        target_turn += 1.0
    if key_down(keys, p.B3G_RIGHT_ARROW, ord("d")):
        target_turn -= 1.0
    if key_down(keys, ord(" ")):
        target_speed = 0.0
        target_turn = 0.0

    # Ease toward the commanded values so the legs don't jerk when a key changes
    speed += (target_speed - speed) * SPEED_SMOOTHING
    turn += (target_turn - turn) * TURN_SMOOTHING

    # Tank style steering the two sides take different stride sizes
    
    # Turning left means the right side strides longer than the left side 
    stride = {
        "l": clamp((speed - TURN_GAIN * turn) * stride_scale, -MAX_STRIDE, MAX_STRIDE),
        "r": clamp((speed + TURN_GAIN * turn) * stride_scale, -MAX_STRIDE, MAX_STRIDE),
    }

    # Only advance the gait while something is commanded otherwise settle in the standing pose
    if max(abs(stride["l"]), abs(stride["r"])) > 0.02:
        base_phase = (base_phase + DT / CYCLE_PERIOD * step_rate) % 1.0

    for g, (leg_a, leg_b) in enumerate(leg_pairs):
        phase = (base_phase + g / n_groups) % 1.0
        swinging = phase < SWING_FRACTION

        if swinging:
            s = phase / SWING_FRACTION
            sweep = -1.0 + 2.0 * s
        else:
            s = (phase - SWING_FRACTION) / (1.0 - SWING_FRACTION)
            sweep = 1.0 - 2.0 * s

        for leg in (leg_a, leg_b):
            side = 1 if leg.startswith("l") else -1
            amp = stride[leg[0]]
            lift = min(abs(amp), 1.0)

            hip_target = -side * amp * HIP_AMPLITUDE * sweep
            if swinging:
                knee_target = KNEE_STANCE + KNEE_LIFT * lift
                ankle_target = ANKLE_STANCE + ANKLE_LIFT * lift
            else:
                knee_target = KNEE_STANCE
                ankle_target = ANKLE_STANCE

            p.setJointMotorControl2(robot_id, joint_indices[f"{leg}_hip"], p.POSITION_CONTROL,
                                    targetPosition=hip_target, force=25)
            p.setJointMotorControl2(robot_id, joint_indices[f"{leg}_knee"], p.POSITION_CONTROL,
                                    targetPosition=knee_target, force=25)
            p.setJointMotorControl2(robot_id, joint_indices[f"{leg}_ankle"], p.POSITION_CONTROL,
                                    targetPosition=ankle_target, force=20)

    p.stepSimulation()

    if FOLLOW_CAMERA:
        # Keep whatever angle and zoom set with the mouse so i can track the robot
        cam = p.getDebugVisualizerCamera()
        pos, _ = p.getBasePositionAndOrientation(robot_id)
        p.resetDebugVisualizerCamera(cam[10], cam[8], cam[9], pos)

    time.sleep(DT)