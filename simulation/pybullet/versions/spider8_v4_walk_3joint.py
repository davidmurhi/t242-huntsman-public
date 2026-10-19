import pybullet as p
import pybullet_data
import time
import math
import os

# ---------------- Setup ----------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)

plane_id = p.loadURDF("plane.urdf")

urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "spider8_v4_longbody_3joint.urdf")
start_pos = [0, 0, 0.35]
robot_id = p.loadURDF(urdf_path, start_pos, useFixedBase=False)

p.changeDynamics(plane_id, -1, lateralFriction=1.5)
for j in range(p.getNumJoints(robot_id)):
    p.changeDynamics(robot_id, j, lateralFriction=1.5)

# ---------------- Map joint names to indices ----------------
joint_indices = {}
for j in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, j)
    name = info[1].decode("utf-8")
    joint_indices[name] = j


leg_pairs = [
    ("l1", "r1"),
    ("l2", "r2"),
    ("l3", "r3"),
    ("l4", "r4"),
]
n_groups = len(leg_pairs)

HIP_AMPLITUDE = 0.35     # how far forward/back each foot reaches (radians)

KNEE_LIFT = 0.6          # knee bend while a leg is swinging through the air

KNEE_STANCE = 0.15       # knee bend while a leg is planted on the ground

ANKLE_LIFT = 0.35        # ankle bend while swinging helps the foot clear the ground

ANKLE_STANCE = 0.05      # ankle bend while planted keeps the foot flatter against the ground
CYCLE_PERIOD = 2.0       # seconds for one full wave to pass around all pairs
SWING_FRACTION = 0.25    # each pair spends 25% of the cycle swinging - with
                         

start_time = time.time()

while True:
    t = time.time() - start_time
    base_phase = (t / CYCLE_PERIOD) % 1.0  

    for g, (leg_a, leg_b) in enumerate(leg_pairs):
        # Both legs in a pair share the same phase so they are swing together
        phase = (base_phase + g / n_groups) % 1.0

        if phase < SWING_FRACTION:
            
            s = phase / SWING_FRACTION  
            forward_target = -HIP_AMPLITUDE + 2 * HIP_AMPLITUDE * s
            knee_target = KNEE_STANCE + KNEE_LIFT
            ankle_target = ANKLE_STANCE + ANKLE_LIFT
        else:
            # STANCE: feet planted, sweeping backward to push the body forward
            s = (phase - SWING_FRACTION) / (1.0 - SWING_FRACTION)  # 0 -> 1
            forward_target = HIP_AMPLITUDE - 2 * HIP_AMPLITUDE * s
            knee_target = KNEE_STANCE
            ankle_target = ANKLE_STANCE

        for leg in (leg_a, leg_b):
            side = 1 if leg.startswith("l") else -1  # left = +1, right = -1

            
            hip_target = -side * forward_target

            p.setJointMotorControl2(robot_id, joint_indices[f"{leg}_hip"], p.POSITION_CONTROL,
                                     targetPosition=hip_target, force=25)
            p.setJointMotorControl2(robot_id, joint_indices[f"{leg}_knee"], p.POSITION_CONTROL,
                                     targetPosition=knee_target, force=25)
            p.setJointMotorControl2(robot_id, joint_indices[f"{leg}_ankle"], p.POSITION_CONTROL,
                                     targetPosition=ankle_target, force=20)

    p.stepSimulation()
    time.sleep(1.0 / 240.0)
