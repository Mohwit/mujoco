"""Optional: train SAC (stable-baselines3) on PendulumHoldEnv and compare with the PD + gravity compensation policy.

    uv run --python .venv/bin/python lessons/09-environments-and-next-steps/09_try_sac.py

Uses the same environment class as the lesson script. Takes a few minutes on a laptop CPU.
"""
import importlib.util
import pathlib
import time

import numpy as np
from stable_baselines3 import SAC

here = pathlib.Path(__file__).parent
spec = importlib.util.spec_from_file_location("lesson9", here / "09_environments_and_next_steps.py")
lesson9 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lesson9)

STEPS = 20_000
env = lesson9.PendulumHoldEnv()
t0 = time.time()
model = SAC("MlpPolicy", env, seed=0, verbose=0, device="cpu")
model.learn(total_timesteps=STEPS)
print(f"trained SAC for {STEPS} env steps ({STEPS // env.max_steps} episodes) in {time.time() - t0:.0f} s")


def sac_policy(obs, rng):
    action, _ = model.predict(obs, deterministic=True)
    return action


print("seed   zero      PD+grav   SAC")
rows = {"zero": [], "PD+grav": [], "SAC": []}
for s in range(5):
    z, _, _ = lesson9.run_episode(env, lesson9.zero_policy, s)
    p, _, _ = lesson9.run_episode(env, lesson9.pd_grav_policy, s)
    a, _, _ = lesson9.run_episode(env, sac_policy, s)
    rows["zero"].append(z); rows["PD+grav"].append(p); rows["SAC"].append(a)
    print(f"{s:>4}  {z:8.3f}  {p:8.3f}  {a:8.3f}")
print("mean  " + "  ".join(f"{np.mean(v):8.3f}" for v in rows.values()))
