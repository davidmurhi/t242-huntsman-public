"""
Couldnt see trained policy walk in the PyBullet GUI so total_timesteps was Increased to 2,000,000 so the definitly
has more chances to actually learn to move rather then stand still. 

Usage:
    python train_ppo.py

will take around 30 minutes to run as total_timesteps = 2,000,000
Once training is done, use watch_trained_policy.py to actually see the
trained policy walk in the PyBullet GUI.

"""

import os
import multiprocessing
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize

from spider_env import SpiderEnv

# ---------------- Config ----------------
TOTAL_TIMESTEPS = 2_000_000  
                             

N_ENVS = max(1, multiprocessing.cpu_count() - 1)

CHECKPOINT_FREQ = 10_000    
LOG_DIR = "./logs"
CHECKPOINT_DIR = "./checkpoints"
MODEL_DIR = "./models"

os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(CHECKPOINT_DIR, exist_ok=True)
os.makedirs(MODEL_DIR, exist_ok=True)


def main():
    # enviroment 
    
    env = make_vec_env(
        lambda: SpiderEnv(render=False),
        n_envs=N_ENVS,
        vec_env_cls=SubprocVecEnv,
        monitor_dir=LOG_DIR,
    )

    env = VecNormalize(env, norm_obs=True, norm_reward=True)

    # model
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

    # train 
    print(f"Training for {TOTAL_TIMESTEPS} timesteps across {N_ENVS} parallel envs...")
    model.learn(
        total_timesteps=TOTAL_TIMESTEPS,
        callback=checkpoint_callback,
    )

    final_path = os.path.join(MODEL_DIR, "spider8_ppo")
    model.save(final_path)

    
    vecnorm_path = os.path.join(MODEL_DIR, "vecnormalize.pkl")
    env.save(vecnorm_path)

    print(f"Training complete. Final model saved to {final_path}.zip")
    print(f"Normalization stats saved to {vecnorm_path}")

    env.close()



if __name__ == "__main__":
    main()