"""
Load a trained PPO model and watch spider8 walk using it, in the PyBullet GUI.

Usage:
    python watch_trained_policy.py
"""

import time
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from spider_env import SpiderEnv

MODEL_PATH = "./models/spider8_ppo.zip"
VECNORM_PATH = "./models/vecnormalize.pkl"
N_EPISODES = 5

# Must wrap in the same VecNormalize used during training, loading its saved
# statistics, or the policy will see observations on a totally different
# scale than what it actually learned on and won't behave correctly.
env = DummyVecEnv([lambda: SpiderEnv(render=True)])
env = VecNormalize.load(VECNORM_PATH, env)
env.training = False    # don't keep updating normalization stats at inference
env.norm_reward = False  # we want to see the real, unnormalized reward printed

model = PPO.load(MODEL_PATH, env=env)

for episode in range(N_EPISODES):
    obs = env.reset()
    done = False
    total_reward = 0.0

    while not done:
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        total_reward += reward[0]
        time.sleep(1.0 / 240.0)  # roughly real-time playback

    print(f"Episode {episode + 1}: total reward = {total_reward:.2f}")

env.close()