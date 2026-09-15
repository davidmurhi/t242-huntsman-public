import gymnasium as gym
from gymnasium import spaces
import numpy as np

class SpiderEnv(gym.Env):
    def __init__(self):
        super().__init__()
        # Action: 24 motors, each gets a value between -1 and 1 (PWM command)
        self.action_space = spaces.Box(low=-1, high=1, shape=(24,), dtype=np.float32)

        # Observation: 24 joint angles + 4 body orientation values (placeholder — confirm with Kai)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(28,), dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        observation = np.zeros(28, dtype=np.float32)  # Kai replaces with real PyBullet reset
        info = {}
        return observation, info

    def step(self, action):
        # Kai plugs in: send `action` to PyBullet, step physics forward
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