"""
Train an RL policy to grasp and lift the red box (WarehouseGraspEnv).
====================================================================

Algorithm: SAC (Soft Actor-Critic) from stable-baselines3.
  - Off-policy: every transition goes into a replay buffer and is reused
    many times, so it needs far fewer simulator steps than PPO.
  - Good default for continuous actions (our 4-D Cartesian + gripper action).

Train (≈15–25 min on a laptop CPU for 150k steps):
    uv run --python .venv/bin/python lessons/16-capstone-warehouse-robot/train_grasp_rl.py --steps 150000

Watch learning curves (optional, in another terminal):
    .venv/bin/tensorboard --logdir lessons/16-capstone-warehouse-robot/rl_runs

Evaluate a saved policy (success rate over 50 random episodes):
    uv run --python .venv/bin/python lessons/16-capstone-warehouse-robot/train_grasp_rl.py --eval models/grasp_sac_best/best_model.zip

Use it inside the full warehouse task:
    uv run --python .venv/bin/python lessons/16-capstone-warehouse-robot/16_capstone_closed_loop.py --headless --policy lessons/16-capstone-warehouse-robot/models/grasp_sac_best/best_model.zip
"""

import argparse
from pathlib import Path

import numpy as np
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor

from warehouse_grasp_env import WarehouseGraspEnv

HERE = Path(__file__).parent


def make_env():
    # Monitor records episode return/length/success for logging
    return Monitor(WarehouseGraspEnv(), info_keywords=("is_success",))


def train(steps, seed):
    env = make_env()
    eval_env = make_env()
    model = SAC(
        "MlpPolicy", env,
        learning_rate=3e-4,
        buffer_size=300_000,
        batch_size=256,
        gamma=0.98,              # ~50-step horizon: an episode is ≤100 steps
        tau=0.005,
        learning_starts=5_000,   # random exploration first, fills the buffer
        policy_kwargs=dict(net_arch=[256, 256]),
        tensorboard_log=str(HERE / "rl_runs"),
        seed=seed,
        verbose=1,
        device="cpu",
    )
    # Every 5k steps: run 20 deterministic episodes, keep the best model
    eval_cb = EvalCallback(eval_env, n_eval_episodes=20, eval_freq=5_000,
                           best_model_save_path=str(HERE / "models" / "grasp_sac_best"),
                           log_path=str(HERE / "models" / "eval_logs"),
                           deterministic=True)
    model.learn(total_timesteps=steps, callback=eval_cb, tb_log_name="grasp_sac")
    out = HERE / "models" / "grasp_sac_final"
    model.save(out)
    print(f"\nSaved final policy to {out}.zip")
    print(f"Best policy (by eval return): {HERE / 'models' / 'grasp_sac_best' / 'best_model.zip'}")


def evaluate(path, episodes):
    env = WarehouseGraspEnv()
    model = SAC.load(path, device="cpu")
    successes, returns, lengths = 0, [], []
    for ep in range(episodes):
        obs, _ = env.reset(seed=10_000 + ep)    # seeds never seen in training
        done, ret = False, 0.0
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, info = env.step(action)
            ret += r
            done = term or trunc
        successes += info["is_success"]
        returns.append(ret)
        lengths.append(env.steps)
    print(f"Policy: {path}")
    print(f"  success rate:     {successes}/{episodes} = {successes / episodes:.0%}")
    print(f"  mean return:      {np.mean(returns):.1f}")
    print(f"  mean length:      {np.mean(lengths):.1f} steps "
          f"({np.mean(lengths) * 0.05:.1f} s at 20 Hz)")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--steps", type=int, default=150_000)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--eval", type=str, default=None, help="path to a saved .zip policy")
    p.add_argument("--episodes", type=int, default=50)
    args = p.parse_args()
    if args.eval:
        evaluate(args.eval, args.episodes)
    else:
        train(args.steps, args.seed)
