#!/usr/bin/env python3
"""
train.py

Train the Huntsman to walk over rough ground with PPO
(Stable-Baselines3), starting from our hand-written gait.

    pip install -r requirements-train.txt

    python train.py --check                 1-minute sanity check, no training
    python train.py --steps 200000          a short trial run (~10-20 min, laptop)
    python train.py                         the real run: 20 M steps, cloud box

    tensorboard --logdir runs                watch the reward climb
    mjpython watch_policy.py runs/<name>/best_model.zip --terrain rubble

WHAT A RUN DOES
    1. Starts N copies of the robot (one per CPU core), each on its own
       random terrain, friction, payload and servo strength.
    2. Lets the current policy drive them for a while, recording what it
       saw, what it did and the reward it got.
    3. PPO updates the network towards the actions that scored better
       than expected, a little at a time so it never forgets how to walk.
    4. Every 100 k steps it tests the policy on fixed terrains it is not
       training on, and keeps the best one as best_model.zip.
    5. Curriculum: terrains start gentle (difficulty 0.3) and get
       rougher as the policy improves.

Everything lands in runs/<date-time>/: checkpoints, best_model.zip, the
observation normaliser (vecnormalize.pkl, needed to run the policy) and
TensorBoard logs. A run can be stopped and resumed with --resume.
"""

from __future__ import annotations

import argparse
import os
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TERRAINS = ("flat", "rubble", "slope", "steps", "mixed")


def check():
    """Build the environment, run it with zero and random actions, and
    report - proves the set-up works before spending hours training."""
    from huntsman_env import HuntsmanEnv
    print("building the environment ...")
    env = HuntsmanEnv(terrains=TERRAINS, seed=0)
    print(f"  observation {env.observation_space.shape[0]} numbers, "
          f"action {env.action_space.shape[0]} servos")
    for label, policy in (("zero action (the hand gait alone)", "zero"),
                          ("random actions", "random")):
        print(f"\n  {label}:")
        for kind in TERRAINS:
            env.terrains = (kind,)
            obs, _ = env.reset(seed=1)
            assert obs.shape == env.observation_space.shape
            total, steps, t0 = 0.0, 0, time.time()
            for _ in range(500):                     # 10 s of robot time
                a = (np.zeros(env.action_space.shape) if policy == "zero"
                     else env.action_space.sample())
                obs, r, term, trunc, info = env.step(a)
                total += r
                steps += 1
                if term or trunc:
                    break
            print(f"    {kind:<7} reward {total:7.1f} over {steps} steps, "
                  f"walked {info['x']:+.2f} m"
                  f"{'  (fell)' if term else ''}   "
                  f"[{steps / (time.time() - t0):.0f} steps/s]")
    print("\nThe environment works. Train with:  python train.py")
    return 0


def train(args):
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import (CallbackList,
                                                    CheckpointCallback,
                                                    EvalCallback,
                                                    BaseCallback)
    from stable_baselines3.common.vec_env import (SubprocVecEnv, VecMonitor,
                                                  VecNormalize, DummyVecEnv)
    from huntsman_env import make

    name = args.name or time.strftime("%Y%m%d-%H%M%S")
    out = os.path.join(HERE, "runs", name)
    os.makedirs(out, exist_ok=True)
    n = args.envs or max(1, (os.cpu_count() or 2) - 1)

    def vec(count, difficulty, seed0):
        fns = [make(TERRAINS, difficulty=difficulty, seed=seed0 + i,
                    gait=args.gait) for i in range(count)]
        return SubprocVecEnv(fns) if count > 1 else DummyVecEnv(fns)

    env = VecMonitor(vec(n, args.start_difficulty, 0))
    if args.resume:
        env = VecNormalize.load(os.path.join(args.resume, "vecnormalize.pkl"),
                                env)
        model = PPO.load(os.path.join(args.resume, "last_model.zip"), env=env,
                         tensorboard_log=os.path.join(out, "tb"))
    else:
        env = VecNormalize(env, norm_obs=True, norm_reward=True,
                           clip_obs=10.0)
        model = PPO("MlpPolicy", env, verbose=1,
                    n_steps=1024, batch_size=4096, n_epochs=5,
                    learning_rate=3e-4, gamma=0.99, gae_lambda=0.95,
                    clip_range=0.2, ent_coef=0.0, vf_coef=0.5,
                    max_grad_norm=1.0, target_kl=0.03,
                    policy_kwargs=dict(net_arch=dict(pi=[256, 256],
                                                     vf=[256, 256]),
                                       log_std_init=-1.0),
                    tensorboard_log=os.path.join(out, "tb"))

    eval_env = VecNormalize(VecMonitor(vec(1, 1.0, 10_000)), training=False,
                            norm_reward=False)
    eval_env.obs_rms = env.obs_rms

    class Curriculum(BaseCallback):
        """Make the ground rougher as the policy gets better."""
        def _on_step(self):
            frac = min(1.0, self.num_timesteps / (0.6 * args.steps))
            d = args.start_difficulty + (1.0 - args.start_difficulty) * frac
            self.training_env.set_attr("difficulty", d)
            self.logger.record("curriculum/difficulty", d)
            return True

    class SaveNorm(BaseCallback):
        def _on_step(self):
            return True

        def _on_rollout_end(self):
            env.save(os.path.join(out, "vecnormalize.pkl"))
            eval_env.obs_rms = env.obs_rms

    callbacks = CallbackList([
        Curriculum(), SaveNorm(),
        CheckpointCallback(max(100_000 // n, 1), os.path.join(out, "ckpt")),
        EvalCallback(eval_env, best_model_save_path=out,
                     eval_freq=max(100_000 // n, 1), n_eval_episodes=5,
                     deterministic=True),
    ])
    print(f"training {args.steps:,} steps on {n} robots -> {out}")
    model.learn(total_timesteps=args.steps, callback=callbacks,
                reset_num_timesteps=not args.resume)
    model.save(os.path.join(out, "last_model.zip"))
    env.save(os.path.join(out, "vecnormalize.pkl"))
    print(f"done. Watch it:  mjpython watch_policy.py {out}/best_model.zip")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="test the environment, no training")
    ap.add_argument("--steps", type=int, default=20_000_000)
    ap.add_argument("--envs", type=int, default=0,
                    help="parallel robots (default: CPU cores - 1)")
    ap.add_argument("--gait", default="rubble",
                    help="hand gait the policy corrects (walk, rubble, ...)")
    ap.add_argument("--start-difficulty", type=float, default=0.3)
    ap.add_argument("--name", default="")
    ap.add_argument("--resume", default="", help="a runs/<name> folder")
    args = ap.parse_args()
    if args.check:
        return check()
    return train(args)


if __name__ == "__main__":
    raise SystemExit(main())
