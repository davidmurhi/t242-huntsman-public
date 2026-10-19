"""
v2: removed the crashing tensorboard/progress bar options, switched to 19 parallel environments 
via SubprocVecEnv for a big speed boost. and increased TOTAL_TIMESTEPS to 500,000 so the policy has more 
chances to actually learn to move.

Usage:
    python train_ppo.py

What it does:
  1. Runs N_ENVS parallel environments, headless (no GUI) for speed
  2. Trains a PPO policy for TOTAL_TIMESTEPS steps
  3. Saves checkpoints periodically to ./checkpoints/
  4. Saves the final trained model to ./models/spider8_ppo.zip

Once training is done, run watch_trained_policy.py to see the trained policy walk in the PyBullet GUI.
"""

import os
import multiprocessing
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv

import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from spider_env import SpiderEnv  # lives one folder up

# ---------------- Config ----------------
TOTAL_TIMESTEPS = 500_000   


N_ENVS = max(1, multiprocessing.cpu_count() - 1)

CHECKPOINT_FREQ = 10_000    #
LOG_DIR = "./logs"
CHECKPOINT_DIR = "./checkpoints"
MODEL_DIR = "./models"

os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(CHECKPOINT_DIR, exist_ok=True)
os.makedirs(MODEL_DIR, exist_ok=True)


def main():
    
        lambda: SpiderEnv(render=False),
        n_envs=N_ENVS,
        vec_env_cls=SubprocVecEnv,
        monitor_dir=LOG_DIR,
    )

    # ---------------- Model ----------------
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
    )

    checkpoint_callback = CheckpointCallback(
        save_freq=max(CHECKPOINT_FREQ // N_ENVS, 1),  # save_freq is per-env
        save_path=CHECKPOINT_DIR,
        name_prefix="spider8_ppo",
    )

    # ---------------- Train ----------------
    print(f"Training for {TOTAL_TIMESTEPS} timesteps across {N_ENVS} parallel envs...")
    model.learn(
        total_timesteps=TOTAL_TIMESTEPS,
        callback=checkpoint_callback,
    )

    final_path = os.path.join(MODEL_DIR, "spider8_ppo")
    model.save(final_path)
    print(f"Training complete. Final model saved to {final_path}.zip")

    env.close()



if __name__ == "__main__":
    main()
