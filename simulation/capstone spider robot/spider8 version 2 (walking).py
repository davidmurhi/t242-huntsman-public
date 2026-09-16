import pybullet as p
import pybullet_data
import time
import math
import os

#  Setup 
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)

plane_id = p.loadURDF("plane.urdf")

urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "spider8 version 3(straight walking).urdf")
start_pos = [0, 0, 0.15]
robot_id = p.loadURDF(urdf_path, start_pos, useFixedBase=False)

p.changeDynamics(plane_id, -1, lateralFriction=1.5)
for j in range(p.getNumJoints(robot_id)):
    p.changeDynamics(robot_id, j, lateralFriction=1.5)


joint_indices = {}
for j in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, j)
    name = info[1].decode("utf-8")
    joint_indices[name] = j

legs = ["l1", "l2", "l3", "l4", "r1", "r2", "r3", "r4"]

group_a = {"l2", "l4", "r1", "r3"}
group_b = {"l1", "l3", "r2", "r4"}

HIP_AMPLITUDE = 0.4      # stride length 
KNEE_LIFT = 0.5          # how much the knee bends to lift the foot when swinging
KNEE_STANCE = 0.15       # slight standing bend when foot is planted
GAIT_FREQ = 1.0          # gait cycles per second

start_time = time.time()

while True:
    t = time.time() - start_time
    base_phase = 2 * math.pi * GAIT_FREQ * t

    for leg in legs:
        side = 1 if leg.startswith("l") else -1  # left = +1, right = -1
        phase = base_phase if leg in group_a else base_phase + math.pi

        swing = math.sin(phase)  # >0 during swing (foot lifted, moving fwd)

        hip_target = -side * HIP_AMPLITUDE * swing

        knee_target = KNEE_STANCE + KNEE_LIFT * max(0.0, swing)

        p.setJointMotorControl2(robot_id, joint_indices[f"{leg}_hip"], p.POSITION_CONTROL,
                                 targetPosition=hip_target, force=15)
        p.setJointMotorControl2(robot_id, joint_indices[f"{leg}_knee"], p.POSITION_CONTROL,
                                 targetPosition=knee_target, force=15)
#time
    p.stepSimulation()
    time.sleep(1.0 / 240.0)
