import pybullet as p
import pybullet_data
import time
import os


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

plane_id = p.loadURDF("plane.urdf")

urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "spider8 version 1.urdf")
start_pos = [0, 0, 0.35]
robot_id = p.loadURDF(urdf_path, start_pos, useFixedBase=False)




while True:
    p.stepSimulation()
    time.sleep(1.0 / 240.0)
