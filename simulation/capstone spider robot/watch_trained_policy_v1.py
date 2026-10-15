"""
Load a trained PPO model and watch spider8 walk using it, in the PyBullet GUI.

VERSION 1 - original version. Superseded once train_ppo.py added VecNormalize,
since a model trained with normalized observations needs those same stats
applied at inference time too - see watch_trained_policy_v2.py.

Usage:
    python watch_trained_policy.py
"""

import time
from stable_baselines3 import PPO
from spider_env import SpiderEnv

MODEL_PATH = "./models/spider8_ppo.zip"
N_EPISODES = 5

env = SpiderEnv(render=True)  
model = PPO.load(MODEL_PATH, env=env)

for episode in range(N_EPISODES):
    obs, info = env.reset()
    terminated = truncated = False
    total_reward = 0.0

    while not (terminated or truncated):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        time.sleep(1.0 / 240.0) 

    print(f"Episode {episode + 1}: total reward = {total_reward:.2f}")

env.close()
