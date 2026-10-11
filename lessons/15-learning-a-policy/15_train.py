"""Lesson 15 (the slow part): train SAC on the lesson 9 pendulum and save small result files.

    cd /Users/mohwit/Desktop/mujoco && .venv/bin/python lessons/15-learning-a-policy/15_train.py

What it does (CPU only, fixed seeds, about 7 minutes on an Apple M5):
  * trains SAC (stable_baselines3) on PendulumHoldEnv from lesson 9, imported, not copied;
  * also trains on four changed rewards (one change each) to show why reward design matters;
  * at checkpoints 0, 1k, 2k, 5k, 10k, 20k env steps it pauses, evaluates the policy
    deterministically on seeds 0-4, and records the seed-0 rollout (angle and action);
  * writes results/curves.json (all numbers) and results/sac_base_seed0_actor.pt (the final policy network).

The lesson script 15_learning_a_policy.py only reads these files and runs quick checks.
"""
import importlib.util
import json
import pathlib
import sys
import time

import numpy as np
import torch
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback

HERE = pathlib.Path(__file__).parent
sys.dont_write_bytecode = True   # do not leave __pycache__ folders in the lesson 9 or 15 folders
OUT = HERE / "results"
_spec = importlib.util.spec_from_file_location(
    "lesson9", HERE.parent / "09-environments-and-next-steps" / "09_environments_and_next_steps.py")
lesson9 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(lesson9)

CHECKPOINTS = [0, 1000, 2000, 5000, 10000, 20000]   # env steps (one env step = 0.02 s)
EVAL_SEEDS = [0, 1, 2, 3, 4]                         # same start angles as lesson 9
TOL = 0.05                                           # rad, "close enough" for the sparse reward


class VariantEnv(lesson9.PendulumHoldEnv):
    """PendulumHoldEnv with ONE reward change. Physics, observation and action are untouched.

    base      : lesson 9 reward  -(err^2 + 0.1 qvel^2 + 0.01 a^2)
    no_act    : the same without the action term  -(err^2 + 0.1 qvel^2)
    big_act   : the action term 100x heavier  -(err^2 + 0.1 qvel^2 + 1.0 a^2)
    sparse    : +1 when |err| < 0.05 rad, else 0
    approach  : pays for every step that brings the pendulum closer:  max(0, |err_prev| - |err|)
                (the loophole: moving away costs nothing)
    signed    : the fix: |err_prev| - |err|, so moving away costs exactly what coming back pays
    """

    def __init__(self, variant="base", **kw):
        super().__init__(**kw)
        self.variant = variant
        self.prev_abs_err = 0.0

    def reset(self, *, seed=None, options=None):
        obs, info = super().reset(seed=seed, options=options)
        self.prev_abs_err = abs(self.data.qpos[0] - self.target)
        return obs, info

    def step(self, action):
        obs, r_base, term, trunc, info = super().step(action)
        a = float(np.clip(action[0], -1.0, 1.0))
        err = self.data.qpos[0] - self.target
        qd = self.data.qvel[0]
        v = self.variant
        if v == "base":
            r = r_base
        elif v == "no_act":
            r = -(err**2 + 0.1 * qd**2)
        elif v == "big_act":
            r = -(err**2 + 0.1 * qd**2 + 1.0 * a**2)
        elif v == "sparse":
            r = 1.0 if abs(err) < TOL else 0.0
        elif v == "approach":
            r = max(0.0, self.prev_abs_err - abs(err))
        elif v == "signed":
            r = self.prev_abs_err - abs(err)
        else:
            raise ValueError(v)
        self.prev_abs_err = abs(err)
        info["base_reward"] = r_base           # the lesson 9 score: what we MEANT
        return obs, float(r), term, trunc, info


def evaluate(policy_fn, variant):
    """Run policy_fn on seeds 0-4. Returns summary numbers and the seed-0 rollout."""
    env = VariantEnv(variant)
    rets, base_rets, mean_abs_a, mean_da, final_err, crossings = [], [], [], [], [], []
    roll = None
    for s in EVAL_SEEDS:
        obs, info = env.reset(seed=s)
        env.action_space.seed(s)
        ret = bret = 0.0
        qs, acts = [float(env.data.qpos[0])], []
        while True:
            a = policy_fn(obs, env.action_space)
            obs, r, term, trunc, info = env.step(a)
            ret += r
            bret += info["base_reward"]
            qs.append(float(env.data.qpos[0]))
            acts.append(float(np.clip(a[0], -1, 1)))
            if term or trunc:
                break
        acts = np.array(acts)
        q = np.array(qs)
        rets.append(ret)
        base_rets.append(bret)
        mean_abs_a.append(np.mean(np.abs(acts)))
        mean_da.append(np.mean(np.abs(np.diff(acts))))
        final_err.append(abs(q[-1] - env.target))
        # passes: times the angle swung right through the band target +- 0.05 rad, from one side to the other
        side = np.where(q < env.target - TOL, -1, np.where(q > env.target + TOL, 1, 0))
        side = side[side != 0]
        crossings.append(int(np.sum(side[1:] != side[:-1])))
        if s == 0:
            roll = {"q": [round(x, 3) for x in q[::2]],              # every 2nd env step = 0.04 s
                    "a": [round(x, 2) for x in acts[::2]]}
    return {"return": float(np.mean(rets)), "returns": [round(x, 3) for x in rets],
            "base_return": float(np.mean(base_rets)),
            "mean_abs_a": float(np.mean(mean_abs_a)), "mean_da": float(np.mean(mean_da)),
            "final_err": float(np.mean(final_err)), "crossings": float(np.mean(crossings)),
            "rollout": roll}


def sac_fn(model):
    def f(obs, rng):
        a, _ = model.predict(obs, deterministic=True)   # evaluation: no exploration noise
        return a
    return f


class CheckpointEval(BaseCallback):
    def __init__(self, variant, results):
        super().__init__()
        self.variant, self.results = variant, results

    def _on_step(self):
        # the last checkpoint is evaluated after learn() returns, so it includes the final update
        if self.num_timesteps in CHECKPOINTS[:-1]:
            self.results.append({"steps": self.num_timesteps, **evaluate(sac_fn(self.model), self.variant)})
        return True


def train(variant, seed):
    torch.set_num_threads(1)
    env = VariantEnv(variant)
    model = SAC("MlpPolicy", env, seed=seed, verbose=0, device="cpu")
    results = [{"steps": 0, **evaluate(sac_fn(model), variant)}]    # untrained network
    t0 = time.time()
    model.learn(total_timesteps=CHECKPOINTS[-1], callback=CheckpointEval(variant, results))
    secs = time.time() - t0
    results.append({"steps": CHECKPOINTS[-1], **evaluate(sac_fn(model), variant)})
    last = results[-1]
    print(f"{variant:>8} seed {seed}: {secs:5.1f} s   final return {last['return']:9.3f}"
          f"   lesson-9 return {last['base_return']:9.3f}   mean|a| {last['mean_abs_a']:.3f}"
          f"   mean|da| {last['mean_da']:.3f}   final err {last['final_err']:.3f} rad")
    return model, results, secs


def main():
    OUT.mkdir(exist_ok=True)
    data = {"checkpoints": CHECKPOINTS, "eval_seeds": EVAL_SEEDS, "runs": {}, "refs": {}, "train_seconds": {}}
    t_all = time.time()
    plan = [("base", 0), ("base", 1), ("base", 2), ("no_act", 0), ("big_act", 0), ("sparse", 0),
            ("approach", 0), ("approach", 1), ("approach", 2), ("signed", 0)]
    for variant, seed in plan:
        model, res, secs = train(variant, seed)
        key = f"{variant}_seed{seed}"
        data["runs"][key] = res
        data["train_seconds"][key] = round(secs, 1)
        if key == "base_seed0":
            # only the policy network (the actor), about 0.3 MB; the critics are not needed to act
            torch.save(model.actor.state_dict(), OUT / "sac_base_seed0_actor.pt")
    # reference policies from lesson 9, scored with every reward variant
    for variant in ["base", "no_act", "big_act", "sparse", "approach", "signed"]:
        data["refs"][variant] = {name: evaluate(lesson9.POLICIES[name], variant)
                                 for name in ["zero", "random", "PD+grav"]}
    data["total_seconds"] = round(time.time() - t_all, 1)
    (OUT / "curves.json").write_text(json.dumps(data, separators=(",", ":")))
    print(f"total {data['total_seconds']:.0f} s, wrote {OUT / 'curves.json'}")


if __name__ == "__main__":
    main()
