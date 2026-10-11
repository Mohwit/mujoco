"""Check that the page's in-browser pendulum env matches MuJoCo (PendulumHoldEnv).

Runs the JavaScript core from _fragments/script.js (between the @@CORE9 markers) in node,
and the real env in Python, for the zero, PD and PD + gravity policies, seeds 0-4, frame_skip 10 and 20.

    uv run --python .venv/bin/python lessons/09-environments-and-next-steps/09_check_browser_physics.py
"""
import importlib.util
import json
import pathlib
import subprocess

import numpy as np

here = pathlib.Path(__file__).parent
spec = importlib.util.spec_from_file_location("lesson9", here / "09_environments_and_next_steps.py")
lesson9 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lesson9)

src = (here / "_fragments" / "script.js").read_text()
core = src.split("// @@CORE9-START")[1].split("// @@CORE9-END")[0]
js = core + """
const out = {};
for (const fs of [10, 20]) for (const p of ['zero', 'pd', 'pdg']) for (let s = 0; s < 5; s++) {
  const rec = []; const e = L9.episode(p, s, fs, 5 / (0.002 * fs), rec);
  out[`${fs}_${p}_${s}`] = { rec, ret: e.ret };
}
console.log(JSON.stringify(out));
"""
res = json.loads(subprocess.run(["node", "-e", js], capture_output=True, text=True, check=True).stdout)

worst = {"qpos": 0.0, "qvel": 0.0, "reward": 0.0, "return": 0.0}
pols = {"zero": lesson9.zero_policy, "pd": lesson9.pd_policy, "pdg": lesson9.pd_grav_policy}
for fs in (10, 20):
    env = lesson9.PendulumHoldEnv(frame_skip=fs, max_steps=int(round(5 / (0.002 * fs))))
    for p, pol in pols.items():
        for s in range(5):
            obs, _ = env.reset(seed=s)
            js_run = res[f"{fs}_{p}_{s}"]
            ret = 0.0
            for k, (t, q, v, a, r) in enumerate(js_run["rec"]):
                obs, rew, term, trunc, _ = env.step(pol(obs, env.action_space))
                ret += rew
                worst["qpos"] = max(worst["qpos"], abs(env.data.qpos[0] - q))
                worst["qvel"] = max(worst["qvel"], abs(env.data.qvel[0] - v))
                worst["reward"] = max(worst["reward"], abs(rew - r))
            assert trunc and k + 1 == env.max_steps
            worst["return"] = max(worst["return"], abs(ret - js_run["ret"]))
            if s == 0:
                print(f"fs={fs:>2} {p:>4} seed 0: MuJoCo return {ret: .6f}  browser {js_run['ret']: .6f}  final qpos MuJoCo {env.data.qpos[0]: .6f} browser {q: .6f}")
print("largest difference over 30 episodes, every env step:")
for k, v in worst.items():
    print(f"  {k:<7}: {v:.2e}")
print("PASS (4 decimals)" if max(worst.values()) < 5e-5 else "FAIL")
