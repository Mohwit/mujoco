"""Lesson 08 helper: check that the web page's pendulum maths matches real MuJoCo.

Takes the simulation code between SIM-CORE-BEGIN and SIM-CORE-END in _fragments/script.js,
runs it with node, runs the same gains in MuJoCo, and prints the largest difference in angle
over every step of the run. Needs node on PATH.

    uv run --python .venv/bin/python lessons/08-controllers-and-experiments/08_check_browser_copy.py
"""

import importlib.util
import json
import pathlib
import subprocess

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("l8", HERE / "08_controllers_and_experiments.py")
l8 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(l8)

CASES = [  # kp, kd, target, h, gravity comp, seconds, motor limit on
    (400, 40, 1.0, 0.002, False, 3.0, True),    # the logged run; motor clips at 100 N m early on
    (400, 112, 1.0, 0.002, True, 3.0, True),
    (60, 25, 1.0, 0.002, False, 3.0, True),     # never reaches the limit
    (400, 400, 1.0, 0.04, True, 10.0, True),    # unstable timestep: buzzing
    (400, 400, 1.0, 0.04, True, 10.0, False),   # unstable timestep, no motor limit: explodes
    (250, 60, 0.7, 0.005, False, 3.0, True),
]

src = (HERE / "_fragments" / "script.js").read_text()
core = src.split("// SIM-CORE-BEGIN")[1].split("// SIM-CORE-END")[0]
js = core + f"\nconst C = {json.dumps(CASES)};\n" + \
    "console.log(JSON.stringify(C.map(c => { const r = l8Run(c[0], c[1], c[2], c[3], c[4], c[5], c[6]);" \
    " const m = l8Metrics(r, c[0], c[2], c[4]); return { q: r.q, m }; })));"
out = json.loads(subprocess.run(["node", "-e", js], capture_output=True, text=True, check=True).stdout)

print("case                                            steps  max |q_browser - q_mujoco| (rad)  overshoot % (browser / python)")
for case, res in zip(CASES, out):
    kp, kd, tg, h, gc, T, lim = case
    t, q, v, u, f = l8.run(kp, kd, h=h, t_end=T, target=tg, gravity_comp=gc, limited=lim)
    qb = np.array(res["q"])
    pm = l8.metrics(t, q, f, kp, target=tg, gravity_comp=gc)
    assert len(qb) == len(q), (len(qb), len(q))
    err = np.abs(qb - q).max() / (1.0 if lim else max(1.0, np.abs(q).max()))
    print(f"kp={kp:4d} kd={kd:3d} target={tg} h={h} gc={'on ' if gc else 'off'} limit={'on ' if lim else 'off'}"
          f"  {len(q) - 1:5d}  {err:.2e}{'' if lim else ' (relative)'}                {res['m']['overshoot']:.4f} / {pm['overshoot']:.4f}")
