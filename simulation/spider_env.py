import os
import numpy as np
import pybullet as p
import pybullet_data
import gymnasium as gym
from gymnasium import spaces


class SpiderEnv(gym.Env):
    

    JOINT_NAMES = [
        f"{leg}_{part}"
        for leg in ("l1", "r1", "l2", "r2", "l3", "r3", "l4", "r4")
        for part in ("hip", "knee", "ankle")
    ]

    def __init__(self, render=False, urdf_path=None):
        super().__init__()

        self.render_mode = render
        self._urdf_path = urdf_path or os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "spider8_v5_longbody_3joint.urdf",
        )

        self.action_space = spaces.Box(low=-1, high=1, shape=(24,), dtype=np.float32)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(31,), dtype=np.float32
        )

        self._client = None
        self._robot_id = None
        self._plane_id = None
        self._joint_indices = {}
        self._joint_limits = {}  
        self._start_pos = [0, 0, 0.35]
        self._max_force = {"hip": 25, "knee": 25, "ankle": 20}
        self._step_count = 0
        self._max_steps = 1000

        self._connect()

    def _connect(self):
        mode = p.GUI if self.render_mode else p.DIRECT
        self._client = p.connect(mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

    def _load_robot(self):
        p.resetSimulation()
        p.setGravity(0, 0, -9.8)
        p.setRealTimeSimulation(0)

        self._plane_id = p.loadURDF("plane.urdf")
        self._robot_id = p.loadURDF(self._urdf_path, self._start_pos, useFixedBase=False)

        p.changeDynamics(self._plane_id, -1, lateralFriction=1.5)
        for j in range(p.getNumJoints(self._robot_id)):
            p.changeDynamics(self._robot_id, j, lateralFriction=1.5)

        self._joint_indices = {}
        self._joint_limits = {}
        for j in range(p.getNumJoints(self._robot_id)):
            info = p.getJointInfo(self._robot_id, j)
            name = info[1].decode("utf-8")
            self._joint_indices[name] = j
            lower, upper = info[8], info[9]
            self._joint_limits[name] = (lower, upper)

        missing = [n for n in self.JOINT_NAMES if n not in self._joint_indices]
        if missing:
            raise RuntimeError(
                f"URDF is missing expected joints: {missing}. "
                f"Check spider8_v5_longbody_3joint.urdf matches JOINT_NAMES."
            )

    def _scale_action(self, name, normalised_value):
        """Map a -1..1 action value to that joint's real URDF limit range."""
        lower, upper = self._joint_limits[name]
        normalised_value = float(np.clip(normalised_value, -1.0, 1.0))
       
        return lower + (normalised_value + 1.0) * 0.5 * (upper - lower)

    def _get_observation(self):
        joint_angles = np.zeros(24, dtype=np.float32)
        for i, name in enumerate(self.JOINT_NAMES):
            state = p.getJointState(self._robot_id, self._joint_indices[name])
            joint_angles[i] = state[0]  # joint position (angle)

        pos, orn = p.getBasePositionAndOrientation(self._robot_id)
        roll, pitch, yaw = p.getEulerFromQuaternion(orn)
        lin_vel, _ = p.getBaseVelocity(self._robot_id)

        obs = np.concatenate([
            joint_angles,
            np.array([roll, pitch, yaw], dtype=np.float32),
            np.array([pos[2]], dtype=np.float32),
            np.array(lin_vel, dtype=np.float32),
        ])
        return obs.astype(np.float32)

    # ---------------- Gymnasium API ----------------

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._load_robot()
        self._step_count = 0


        for _ in range(10):
            p.stepSimulation()

        observation = self._get_observation()
        info = {}
        return observation, info

    def step(self, action):
        self._step_count += 1

        for i, name in enumerate(self.JOINT_NAMES):
            target = self._scale_action(name, action[i])
            joint_type = name.split("_")[1]  # hip / knee / ankle
            p.setJointMotorControl2(
                self._robot_id,
                self._joint_indices[name],
                p.POSITION_CONTROL,
                targetPosition=target,
                force=self._max_force[joint_type],
            )

        p.stepSimulation()

        observation = self._get_observation()
        pos, orn = p.getBasePositionAndOrientation(self._robot_id)
        roll, pitch, yaw = p.getEulerFromQuaternion(orn)

    
        forward_velocity = observation[28]
        tilt_penalty = abs(roll) + abs(pitch)
        alive_penalty = 0.01
        reward = 3.0 * forward_velocity - 0.05 * tilt_penalty - alive_penalty

        terminated = bool(pos[2] < 0.12 or abs(roll) > 1.0 or abs(pitch) > 1.0)
        truncated = bool(self._step_count >= self._max_steps)

        info = {}
        return observation, reward, terminated, truncated, info

    def close(self):
        if self._client is not None:
            p.disconnect(self._client)
            self._client = None


if __name__ == "__main__":
    env = SpiderEnv(render=False)
    obs, info = env.reset()
    print("Reset observation shape:", obs.shape)
    print("Action space:", env.action_space)
    print("Observation space:", env.observation_space)

  
    for _ in range(5):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        print("reward:", reward, "terminated:", terminated)

    env.close()