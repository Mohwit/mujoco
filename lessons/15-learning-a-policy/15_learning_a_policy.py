"""Lesson 15: learning a policy with reinforcement learning (SAC) on the lesson 9 pendulum.

Run the slow part once (about 7 minutes on CPU), then this script (a few seconds):
    cd /Users/mohwit/Desktop/mujoco
    .venv/bin/python lessons/15-learning-a-policy/15_train.py
    .venv/bin/python lessons/15-learning-a-policy/15_learning_a_policy.py

This script reads results/curves.json and results/sac_base_seed0_actor.pt written by 15_train.py,
re-runs the trained policy in MuJoCo to check it, and prints every number the lesson page uses.
"""
import importlib.util
import json
import pathlib
import sys

import numpy as np
import torch
from stable_baselines3 import SAC

HERE = pathlib.Path(__file__).parent
sys.dont_write_bytecode = True   # do not leave __pycache__ folders in the lesson 9 or 15 folders
_spec = importlib.util.spec_from_file_location("train15", HERE / "15_train.py")
train15 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(train15)
lesson9 = train15.lesson9                # PendulumHoldEnv, policies, run_episode from lesson 9
VariantEnv, evaluate = train15.VariantEnv, train15.evaluate

R = json.loads((HERE / "results" / "curves.json").read_text())
runs, refs = R["runs"], R["refs"]


def section(title):
    print(f"\n== {title} ==")


def main():
    torch.set_num_threads(1)
    env = lesson9.PendulumHoldEnv()

    section("1. The learner: SAC and its networks")
    model = SAC("MlpPolicy", env, seed=0, device="cpu")       # untrained, same settings as 15_train.py
    print(f"gamma (discount)                 : {model.gamma}")
    print(f"buffer_size (replay diary, steps): {model.buffer_size:,}")
    print(f"learning_starts (random steps)   : {model.learning_starts}")
    print(f"batch_size (steps per update)    : {model.batch_size}")
    print(f"updates per env step             : {model.gradient_steps}")
    print(f"learning_rate                    : {model.learning_rate}")
    print(f"ent_coef (exploration bonus)     : {model.ent_coef}  (tuned automatically)")
    a = model.actor
    sizes = [m.in_features for m in a.latent_pi if hasattr(m, "in_features")] + [a.mu.in_features]
    n_actor = sum(p.numel() for p in a.parameters())
    q0 = model.critic.q_networks[0]
    n_q = sum(p.numel() for p in q0.parameters())
    print(f"actor (the policy) layers        : {sizes[0]} obs -> {' -> '.join(str(s) for s in sizes[1:])} -> mean and log_std of 1 action, squashed by tanh to -1..1")
    print(f"actor parameters                 : {n_actor:,}")
    print(f"critics (score guessers)         : {len(model.critic.q_networks)} networks, each 4 inputs (3 obs + 1 action) -> 256 -> 256 -> 1, {n_q:,} parameters each")

    section("2. Discount: what a reward far in the future is worth now")
    g = model.gamma
    for k in [1, 10, 50, 100, 250]:
        print(f"gamma^{k:<3} = {g**k:.4f}   (a reward {k} env steps = {k * 0.02:.2f} s away counts {100 * g**k:.1f}%)")
    print(f"sum of gamma^k for k = 0..inf = 1/(1-gamma) = {1 / (1 - g):.0f} steps = {0.02 / (1 - g):.1f} s 'look-ahead'")
    print(f"one step at the target costs -0.0084 (holding effort); worth now if it is 100 steps away: {-0.0084 * g**100:.5f}")

    section("3. Learning curve: mean return over seeds 0-4 at each checkpoint (lesson 9 reward)")
    print("env steps  episodes   train seed 0   seed 1     seed 2     mean")
    for i, steps in enumerate(R["checkpoints"]):
        vals = [runs[f"base_seed{s}"][i]["return"] for s in range(3)]
        print(f"{steps:>9}  {steps // 250:>8}   {vals[0]:10.3f}  {vals[1]:9.3f}  {vals[2]:9.3f}  {np.mean(vals):9.3f}")
    for name in ["zero", "random", "PD+grav"]:
        print(f"reference {name:>8}: {refs['base'][name]['return']:9.3f}")
    print(f"training time (20,000 env steps, one CPU thread): seed 0 {R['train_seconds']['base_seed0']:.1f} s, "
          f"seed 1 {R['train_seconds']['base_seed1']:.1f} s, seed 2 {R['train_seconds']['base_seed2']:.1f} s; "
          f"all {len(R['train_seconds'])} runs plus references: {R['total_seconds']:.0f} s")

    section("4. Reload the trained policy (train seed 0, 20,000 steps) and replay it in MuJoCo")
    model.actor.load_state_dict(torch.load(HERE / "results" / "sac_base_seed0_actor.pt"))
    sac = train15.sac_fn(model)
    live = evaluate(sac, "base")
    stored = runs["base_seed0"][-1]
    print(f"mean return now {live['return']:.3f}   stored by 15_train.py {stored['return']:.3f}   "
          f"same: {abs(live['return'] - stored['return']) < 1e-3}")
    print("seed  start (rad)    SAC     PD+grav   SAC first 1 s  PD+grav first 1 s  SAC final q  PD+grav final q")
    pdg = lesson9.pd_grav_policy
    sums = np.zeros(6)
    for s in train15.EVAL_SEEDS:
        row = []
        for pol in (sac, pdg):
            obs, info = env.reset(seed=s)
            rews = []
            while True:
                obs, r, term, trunc, info = env.step(pol(obs, env.action_space))
                rews.append(r)
                if term or trunc:
                    break
            row += [sum(rews), sum(rews[:50]), env.data.qpos[0]]
        _, i0 = env.reset(seed=s)
        sums += np.array(row)
        print(f"{s:>4}  {i0['angle']: .5f}  {row[0]:9.3f} {row[3]:9.3f}   {row[1]:9.3f}      {row[4]:9.3f}        {row[2]: .4f}      {row[5]: .4f}")
    m = sums / 5
    print(f"mean            {m[0]:9.3f} {m[3]:9.3f}   {m[1]:9.3f}      {m[4]:9.3f}        {m[2]: .4f}      {m[5]: .4f}")
    print(f"SAC     : mean |action| {live['mean_abs_a']:.3f}, mean |change of action per step| {live['mean_da']:.4f}, "
          f"final |error| {live['final_err']:.3f} rad, passes through the target band {live['crossings']:.1f}")
    p = refs["base"]["PD+grav"]
    print(f"PD+grav : mean |action| {p['mean_abs_a']:.3f}, mean |change of action per step| {p['mean_da']:.4f}, "
          f"final |error| {p['final_err']:.3f} rad, passes through the target band {p['crossings']:.1f}")
    # new start angles the policy never trained on (outside -0.5..0.5 rad is NOT tested: same range)
    new = []
    for s in range(100, 105):
        r1, _, _ = lesson9.run_episode(env, sac, s)
        r2, _, _ = lesson9.run_episode(env, pdg, s)
        new.append((r1, r2))
    new = np.array(new)
    print(f"unseen seeds 100-104: SAC mean {new[:, 0].mean():.3f}, PD+grav mean {new[:, 1].mean():.3f}")
    # exploration: the same trained network, sampling instead of taking the mean
    obs, _ = env.reset(seed=0)
    det = [float(model.predict(obs, deterministic=True)[0][0]) for _ in range(3)]
    torch.manual_seed(0)
    sto = [float(model.predict(obs, deterministic=False)[0][0]) for _ in range(3)]
    print(f"seed 0 first observation, deterministic action x3: {[round(x, 4) for x in det]}")
    print(f"seed 0 first observation, exploring action x3    : {[round(x, 4) for x in sto]}")
    torch.manual_seed(0)
    explo = evaluate(lambda o, rng: model.predict(o, deterministic=False)[0], "base")
    print(f"whole evaluation while still exploring: mean return {explo['return']:.3f}, mean |change of action per step| {explo['mean_da']:.4f}")

    section("5. One reward change at a time (each trained 20,000 env steps, train seed 0)")
    print("variant   reward per step                          own return  lesson-9 return  mean|a|  mean|da|  final|err| (rad)  passes")
    desc = {"base": "-(err^2 + 0.1 qvel^2 + 0.01 a^2)", "no_act": "-(err^2 + 0.1 qvel^2)",
            "big_act": "-(err^2 + 0.1 qvel^2 + 1.0 a^2)", "sparse": "+1 if |err| < 0.05 rad else 0",
            "approach": "max(0, |err_prev| - |err|)", "signed": "|err_prev| - |err|"}
    for v in desc:
        r = runs[f"{v}_seed0"][-1]
        print(f"{v:>8}  {desc[v]:<40} {r['return']:10.3f}  {r['base_return']:14.3f}  {r['mean_abs_a']:7.3f}  {r['mean_da']:8.4f}  {r['final_err']:12.3f}  {r['crossings']:8.1f}")
    print("PD+grav scored with each reward (same seeds):")
    for v in desc:
        print(f"{v:>8}: own return {refs[v]['PD+grav']['return']:9.3f}")
    print("own return at each checkpoint (train seed 0):")
    print("variant   " + "  ".join(f"{c:>8}" for c in R["checkpoints"]))
    for v in desc:
        print(f"{v:>8}  " + "  ".join(f"{x['return']:8.3f}" for x in runs[f"{v}_seed0"]))

    section("6. Where should a policy hold the pendulum? (best constant hold angle for each reward)")
    qs = np.linspace(0.0, 1.2, 120001)
    for w in [0.01, 1.0]:
        cost = (qs - 1.0) ** 2 + w * (lesson9.MGD * np.sin(qs) / 100) ** 2   # hold ctrl = m g d sin q / 100
        k = np.argmin(cost)
        print(f"action weight {w:<4}: cheapest hold angle {qs[k]:.4f} rad (ctrl {lesson9.MGD * np.sin(qs[k]) / 100:.4f}), cost per step {cost[k]:.4f}")
    q_sac = runs["base_seed0"][-1]["rollout"]["q"][-1]
    c_sac = (q_sac - 1.0) ** 2 + 0.01 * (lesson9.MGD * np.sin(q_sac) / 100) ** 2
    c_best = np.min((qs - 1.0) ** 2 + 0.01 * (lesson9.MGD * np.sin(qs) / 100) ** 2)
    print(f"SAC holds at {q_sac:.3f} rad: cost per step {c_sac:.4f} vs best {c_best:.4f}; "
          f"over the last 4 s (200 steps) that wastes {200 * (c_sac - c_best):.2f} points")
    for v in ["base", "big_act"]:
        q_end = runs[f"{v}_seed0"][-1]["rollout"]["q"][-1]
        print(f"{v:>8} SAC policy, seed 0, angle at t = 5.0 s: {q_end:.3f} rad")

    section("7. The loophole: 'approach' pays for getting closer, and moving away is free")
    starts = [env.reset(seed=s)[1]["angle"] for s in train15.EVAL_SEEDS]
    print(f"mean starting error |q0 - 1.0| over seeds 0-4: {np.mean([abs(q - 1.0) for q in starts]):.3f} rad "
          f"(the most an honest 'go there and stay' policy can collect)")
    for s in range(3):
        r = runs[f"approach_seed{s}"][-1]
        print(f"train seed {s}: own return {r['return']:.3f}   lesson-9 return {r['base_return']:.3f}   "
              f"passes through the target band {r['crossings']:.1f}   final |err| {r['final_err']:.3f} rad")
    p = refs["approach"]["PD+grav"]
    print(f"PD+grav     : own return {p['return']:.3f}   lesson-9 return {p['base_return']:.3f}   passes through the target band {p['crossings']:.1f}")
    q = np.array(runs["approach_seed0"][-1]["rollout"]["q"])
    print(f"approach seed 0 rollout: min q {q.min():.3f} rad, max q {q.max():.3f} rad over 5 s")
    s = runs["signed_seed0"][-1]
    print(f"fix ('signed'): own return {s['return']:.3f}, lesson-9 return {s['base_return']:.3f}, passes {s['crossings']:.1f}, final |err| {s['final_err']:.3f} rad")


if __name__ == "__main__":
    main()
