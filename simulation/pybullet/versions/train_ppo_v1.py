"""
VERSION 1 - original single-environment version - Train a PPO policy to walk spider8 using the SpiderEnv Gymnasium environment.
Usage:
    python train_ppo.py
This will:
  1. Create the environment (headless, no GUI - much faster for training)
  2. Train a PPO policy for TOTAL_TIMESTEPS steps
  3. Save checkpoints periodically to ./checkpoints/
  4. Save the final trained model to ./models/spider8_ppo.zip
when training is done, use watch_trained_policy.py to actually see the trained policy walk in the PyBullet GUI.
"""

import os
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor

import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from spider_env import SpiderEnv  # lives one folder up

# ---------------- Config ----------------
TOTAL_TIMESTEPS = 200_000   
                             
CHECKPOINT_FREQ = 10_000    
LOG_DIR = "./logs"
CHECKPOINT_DIR = "./checkpoints"
MODEL_DIR = "./models"

os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(CHECKPOINT_DIR, exist_ok=True)
os.makedirs(MODEL_DIR, exist_ok=True)

#enviroment 
env = Monitor(SpiderEnv(render=False), LOG_DIR)

# model
model = PPO(
    "MlpPolicy",
    env,
    verbose=1,
    tensorboard_log=LOG_DIR,
)

checkpoint_callback = CheckpointCallback(
    save_freq=CHECKPOINT_FREQ,
    save_path=CHECKPOINT_DIR,
    name_prefix="spider8_ppo",
)

# -train
if __name__ == "__main__":
    print(f"Training for {TOTAL_TIMESTEPS} timesteps...")
    model.learn(
        total_timesteps=TOTAL_TIMESTEPS,
        callback=checkpoint_callback,
        progress_bar=True,
    )

    final_path = os.path.join(MODEL_DIR, "spider8_ppo")
    model.save(final_path)
    print(f"Training complete. Final model saved to {final_path}.zip")

    env.close()
