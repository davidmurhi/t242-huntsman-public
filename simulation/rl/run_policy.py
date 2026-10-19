"""
Project Huntsman — Run Trained PPO Policy
T242 | IFB398 26se2 | V Source Development

Loads a trained PPO model and runs it in the MuJoCo viewer.
Use this to show Craig what the spider has learned.

Run:
    python run_policy.py                          # loads best model
    python run_policy.py --model models/spider8_ppo_500000_steps.zip
"""

import os, argparse
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
from spider_env import make_spider_env

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR  = os.path.join(SCRIPT_DIR, "models")
BEST_PATH   = os.path.join(MODELS_DIR, "best", "best_model.zip")
NORM_PATH   = os.path.join(MODELS_DIR, "vec_normalize.pkl")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str, default=BEST_PATH,
                        help="Path to trained .zip model")
    parser.add_argument("--steps", type=int, default=2000,
                        help="Steps to run (default 2000 ≈ ~16s)")
    args = parser.parse_args()

    if not os.path.exists(args.model):
        print(f"No model found at: {args.model}")
        print("Train first:  python train_ppo.py")
        return

    print(f"Loading: {args.model}")

    # Wrap in vec env for normalisation
    env = DummyVecEnv([lambda: make_spider_env(render=True)])
    if os.path.exists(NORM_PATH):
        env = VecNormalize.load(NORM_PATH, env)
        env.training = False
        env.norm_reward = False
        print("  VecNormalize loaded.")

    model = PPO.load(args.model, env=env)
    print(f"  Model loaded. Running {args.steps} steps...\n")

    obs = env.reset()
    total_reward = 0
    for i in range(args.steps):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        total_reward += reward[0]
        if done[0]:
            print(f"  Episode done at step {i}. Resetting.")
            obs = env.reset()

    env.close()
    print(f"\nDone. Total reward: {total_reward:.2f}")


if __name__ == "__main__":
    main()
