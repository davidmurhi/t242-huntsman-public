import gymnasium as gym
from gymnasium import spaces
import numpy as np

class SpiderEnv(gym.Env):
    """
    Gymnasium environment for the 8-legged spider robot.

    Action space (24 values):
        One value per motor, range -1 to 1 (maps to PWM command per motor).
        8 legs x 3 joints per leg (1 horizontal + 2 vertical) = 24 motors.

    Observation space (28 values):
        [0:24]  - Joint angles for each of the 24 motors (radians)
        [24:27] - Body orientation as roll, pitch, yaw (radians)
        [27]    - Body height / z-position relative to ground

    NOTE: exact observation layout to be confirmed with Kai once PyBullet
    model is built - this is a working draft based on what a PPO gait
    policy typically needs (joint state + body pose).
    """

    def __init__(self):
        super().__init__()

        self.action_space = spaces.Box(low=-1, high=1, shape=(24,), dtype=np.float32)

        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(28,), dtype=np.float32
        )

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        # Placeholder - Kai replaces with real PyBullet reset (load URDF, reset joints)
        observation = np.zeros(28, dtype=np.float32)
        info = {}
        return observation, info

    def step(self, action):
        # Placeholder - Kai plugs in: apply `action` as PWM/joint targets in
        # PyBullet, step physics forward, read back joint angles + orientation
        observation = np.zeros(28, dtype=np.float32)
        reward = 0.0
        terminated = False
        truncated = False
        info = {}
        return observation, reward, terminated, truncated, info


if __name__ == "__main__":
    env = SpiderEnv()
    obs, info = env.reset()
    print("Reset observation:", obs)
    print("Action space:", env.action_space)
    print("Observation space:", env.observation_space)