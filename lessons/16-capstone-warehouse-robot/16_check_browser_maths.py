"""
Check the lesson page's copy of the controller maths against the Python originals.
================================================================================
The capstone page (Capstone 2 in lessons/index.html) lets you move sliders for the
navigation law and for the arm IK. Its JavaScript (the @@CORE16 block in
_fragments/script.js) re-implements:
    navigate_step   from 16_capstone_closed_loop.py  (wheel commands from heading error, distance, yaw rate)
    arm_ik          from warehouse_common.py         (joint targets for a grasp point)
This script runs that JavaScript in node on random inputs and compares it with the
Python functions. No physics is involved: these are pure formulas.

Run:
    uv run --python .venv/bin/python lessons/16-capstone-warehouse-robot/16_check_browser_maths.py
"""
import importlib
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
from warehouse_common import arm_ik  # noqa: E402

cl = importlib.import_module("16_capstone_closed_loop")


def py_nav(herr, dist, yr):
    """navigate_step's arithmetic, without touching MjData."""
    if dist < cl.NAV_ARRIVE_DIST:
        return 0.0, 0.0
    turn = np.clip(cl.NAV_KP_HEADING * herr - cl.NAV_KD_HEADING * yr, -cl.NAV_MAX_WHEEL, cl.NAV_MAX_WHEEL)
    align = max(0.0, 1.0 - abs(herr) / 0.8)
    creep = 1.5 if abs(herr) < 0.4 else 0.0
    speed = np.clip(max(cl.NAV_KP_SPEED * dist * align, creep), 0.0, cl.NAV_MAX_WHEEL)
    m = cl.NAV_MAX_WHEEL                       # set_wheels clips to +-NAV_MAX_WHEEL
    return float(np.clip(speed - turn, -m, m)), float(np.clip(speed + turn, -m, m))


def main():
    js = (HERE / "_fragments" / "script.js").read_text()
    core = js.split("// @@CORE16-START")[1].split("// @@CORE16-END")[0]
    rng = np.random.default_rng(7)
    nav_in = [[rng.uniform(-1.5, 1.5), rng.uniform(0, 3), rng.uniform(-1, 1)] for _ in range(200)]
    nav_in += [[0.19155179261801303, 2.2661577221868634, 0.020788224610271883]]                      # the t = 3.000 s moment on the page
    ik_in = [[rng.uniform(0.2, 0.6), rng.uniform(-0.15, 0.15), rng.uniform(0.3, 0.7)] for _ in range(200)]
    ik_in += [[0.3791830256719205, 0.043019462472155295, 0.48506338256065573], [0.22, 0.0, 0.55]]    # page presets
    prog = core + f"""
const NAV = {json.dumps(nav_in)}, IK = {json.dumps(ik_in)};
console.log(JSON.stringify({{
  nav: NAV.map(a => {{ const r = C16.nav(a[0], a[1], a[2]); return [r.left, r.right]; }}),
  ik: IK.map(p => {{ const q = C16.ik(p).q; return q ? [q.yaw, q.sh, q.el, q.wp] : null; }}),
}}));"""
    out = json.loads(subprocess.run(["node", "-e", prog], capture_output=True, text=True, check=True).stdout)
    nav_err = max(abs(a - b) for x, js_lr in zip(nav_in, out["nav"]) for a, b in zip(py_nav(*x), js_lr))
    ik_err, n_ok, n_none = 0.0, 0, 0
    for p, jq in zip(ik_in, out["ik"]):
        q = arm_ik(np.array(p), 0.3)
        if q is None or jq is None:
            assert q is None and jq is None, p
            n_none += 1
            continue
        pq = [q["arm_yaw"], q["shoulder_pitch"], q["elbow_pitch"], q["wrist_pitch"]]
        ik_err = max(ik_err, max(abs(a - b) for a, b in zip(pq, jq)))
        n_ok += 1
    print(f"nav law: {len(nav_in)} inputs, largest |JS - Python| wheel command = {nav_err:.2e} rad/s")
    print(f"arm IK:  {n_ok} reachable + {n_none} out-of-reach points (both agree), largest |JS - Python| angle = {ik_err:.2e} rad")
    print("PASS" if nav_err < 1e-9 and ik_err < 1e-9 else "FAIL")


if __name__ == "__main__":
    main()
