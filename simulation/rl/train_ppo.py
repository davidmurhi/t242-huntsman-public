"""
Project Huntsman — PPO Training Script
T242 | IFB398 26se2 | V Source Development

Trains the Spider8 to walk using PPO via Stable Baselines 3.
Uses our Gymnasium Ant-v5 wrapper (spider_env.py).

Install:
    pip install mujoco gymnasium[mujoco] stable-baselines3 tensorboard

Run (local):
    python train_ppo.py

Run (cloud — Craig's compute):
    python train_ppo.py --timesteps 5000000

Monitor training:
    tensorboard --logdir ./logs

Load and watch a trained policy:
    python run_policy.py
"""

import os, argparse
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import (
    CheckpointCallback, EvalCallback
)
from stable_baselines3.common.vec_env import VecNormalize
from spider_env import make_spider_env

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR   = os.path.join(SCRIPT_DIR, "models")
LOGS_DIR     = os.path.join(SCRIPT_DIR, "logs")
os.makedirs(MODELS_DIR, exist_ok=True)
os.makedirs(LOGS_DIR,   exist_ok=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps", type=int, default=1_000_000,
                        help="Total training timesteps (default 1M local, 5M+ on cloud)")
    parser.add_argument("--n_envs", type=int, default=4,
                        help="Parallel environments (default 4, increase on cloud)")
    parser.add_argument("--resume", type=str, default=None,
                        help="Path to existing model .zip to continue training")
    args = parser.parse_args()

    print(f"\nProject Huntsman — Spider8 PPO Training")
    print(f"  Timesteps : {args.timesteps:,}")
    print(f"  Envs      : {args.n_envs}")
    print(f"  Models    : {MODELS_DIR}")
    print(f"  Logs      : {LOGS_DIR}\n")

    # Vectorised + normalised environments — standard SB3 practice
    vec_env = make_vec_env(make_spider_env, n_envs=args.n_envs)
    vec_env = VecNormalize(vec_env, norm_obs=True, norm_reward=True)

    # Eval environment (single, not normalised separately)
    eval_env = make_vec_env(make_spider_env, n_envs=1)
    eval_env = VecNormalize(eval_env, norm_obs=True, norm_reward=False, training=False)

    # PPO hyperparameters tuned for MuJoCo locomotion
    # Based on SB3 zoo defaults for Ant-v5
    ppo_kwargs = dict(
        policy            = "MlpPolicy",
        env               = vec_env,
        learning_rate     = 3e-4,
        n_steps           = 2048,
        batch_size        = 64,
        n_epochs          = 10,
        gamma             = 0.99,
        gae_lambda        = 0.95,
        clip_range        = 0.2,
        ent_coef          = 0.01,
        vf_coef           = 0.5,
        max_grad_norm     = 0.5,
        tensorboard_log   = LOGS_DIR,
        verbose           = 1,
    )

    if args.resume:
        print(f"Resuming from: {args.resume}")
        model = PPO.load(args.resume, env=vec_env, **{k:v for k,v in ppo_kwargs.items()
                                                      if k not in ("policy","env")})
    else:
        model = PPO(**ppo_kwargs)

    callbacks = [
        # Save checkpoint every 50k steps
        CheckpointCallback(
            save_freq   = 50_000 // args.n_envs,
            save_path   = MODELS_DIR,
            name_prefix = "spider8_ppo",
        ),
        # Evaluate every 25k steps, keep best model
        EvalCallback(
            eval_env,
            best_model_save_path = os.path.join(MODELS_DIR, "best"),
            log_path             = LOGS_DIR,
            eval_freq            = 25_000 // args.n_envs,
            n_eval_episodes      = 5,
            deterministic        = True,
        ),
    ]

    model.learn(total_timesteps=args.timesteps, callback=callbacks, reset_num_timesteps=not args.resume)

    final_path = os.path.join(MODELS_DIR, "spider8_ppo_final")
    model.save(final_path)
    vec_env.save(os.path.join(MODELS_DIR, "vec_normalize.pkl"))
    print(f"\nTraining complete. Final model: {final_path}.zip")
    print(f"Run:  python run_policy.py  to watch the trained spider.")


if __name__ == "__main__":
    main()
