"""Check that the web page's copy of the cart-pole physics matches MuJoCo.

Runs the SIM-CORE block of _fragments/script.js in node and compares every step with
MuJoCo for the same runs. Needs `node` on the PATH.

    cd /Users/mohwit/Desktop/mujoco && .venv/bin/python lessons/10-capstone-cart-pole/10_check_browser_copy.py
"""
import importlib.util
import json
import pathlib
import subprocess

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("cp", HERE / "10_capstone_cart_pole.py")
cp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cp)

K, *_ = cp.lqr_gains(np.diag([1.0, 1.0, 0.0, 0.0]), np.array([[1.0]]))
LQR = list(-K)
CASES = [  # name, knobs (None = controller off), start tilt rad, seconds, push [N m, s]
    ("LQR knobs, start 0.1 rad", LQR, 0.1, 2.0, None),
    ("other knobs [0.5, 6, 0.5, 1], start 0.1 rad", [0.5, 6.0, 0.5, 1.0], 0.1, 2.0, None),
    ("LQR knobs, push +1 N m for 0.1 s", LQR, 0.0, 2.0, [1.0, 0.1]),
    ("LQR knobs, start 0.4 rad (ctrl clipped)", LQR, 0.4, 2.0, None),
    ("lean-only knobs, 4 s (inside the rail)", [0.0, LQR[1], 0.0, LQR[3]], 0.1, 4.0, None),
    ("controller off, 1 s fall", None, 0.1, 1.0, None),
]

src = (HERE / "_fragments" / "script.js").read_text()
core = src.split("// SIM-CORE-BEGIN")[1].split("// SIM-CORE-END")[0]
js = core + "\nconst cases = " + json.dumps([[c[1], c[2], c[3], c[4]] for c in CASES]) + ";\n" + \
    "console.log(JSON.stringify(cases.map(c => { const r = c10Run(c[0], c[1], c[2], c[3], false); return [r.x, r.th, r.u]; })));"
out = json.loads(subprocess.run(["node", "-e", js], capture_output=True, text=True, check=True).stdout)
print(f"LQR knobs used by the page: {np.round(LQR, 4).tolist()}  (script.js: {core.split('LQR: ')[1].split(']')[0]}])")
worst = 0.0
for (name, k, th0, T, push), (x, th, u) in zip(CASES, out):
    rec, rail = cp.run(tuple(k) if k else (0, 0, 0, 0), th0=th0, T=T, push=tuple(push) if push else None)
    dx = np.max(np.abs(rec[:, 1] - np.array(x)))
    dth = np.max(np.abs(rec[:, 2] - np.array(th)))
    du = np.max(np.abs(rec[:-1, 5] - np.array(u))) if k else 0.0
    worst = max(worst, dx, dth)
    print(f"{name:<46} {len(x) - 1:>5} steps  max |x diff| {dx:.1e} m  max |theta diff| {dth:.1e} rad  max |ctrl diff| {du:.1e}"
          f"  peak ctrl {np.max(np.abs(rec[:-1, 5])):.4f}  rail hit: {rail}")
print(f"worst difference over all runs: {worst:.1e} (agrees to {int(-np.log10(worst)) if worst > 0 else 16}+ decimals)")
