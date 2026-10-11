"""Check that the page's kinematics (lesson 12 panel and sandbox) matches MuJoCo.

Runs the CORE block of _fragments/script.js in node, then compares with MuJoCo 3.15.0:
forward kinematics (site_xpos), the Jacobian (mj_jacSite rows x and z), analytic IK
(the script's analytic_ik, checked through site_xpos) and whole damped-least-squares runs
(the script's dls_ik). Needs `node`.
"""

import importlib.util
import json
import pathlib
import subprocess

import mujoco
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("l12", HERE / "12_kinematics_and_jacobians.py")
l12 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(l12)

POSES = [(0.0, 0.0), (0.5, 0.8), (1.0, -0.5), (-0.4, 1.2), (0.3, 2.0), (0.0, 0.01), (-2.5, -1.7), (3.0, 2.9)]
TARGETS = [(0.6, 0.6), (0.6, 1.0), (0.9, 1.0), (1.0, 1.0), (0.05, 1.0), (0.1, 1.0), (-0.3, 0.4), (0.2, 1.7)]
RUNS = [((0.6, 0.6), (0.0, 0.0), 0.05), ((0.6, 0.6), (0.0, 0.001), 0.01), ((0.6, 0.6), (0.0, 0.001), 0.2),
        ((0.6, 0.6), (0.0, 0.001), 0.0), ((1.2, 1.0), (0.3, 0.6), 0.2), ((-0.3, 0.4), (0.3, 0.6), 0.1)]

js = (HERE / "_fragments" / "script.js").read_text()
block = js.split("// CORE-BEGIN")[1].split("// CORE-END")[0]
prog = block + f"""
const runs = {json.dumps(RUNS)}.map(([t, q0, lam]) => {{ let q = q0.slice(); const out = [q.slice()];
  for (let k = 0; k < 60; k++) {{ const h = l12FK(q[0], q[1]).hand; if (Math.hypot(t[0] - h[0], t[1] - h[1]) < 1e-6) break;
    const r = l12DLS(q, t[0], t[1], lam); q = r.q; out.push(q.slice()); }} return out; }});
console.log(JSON.stringify({{
  fk: {json.dumps(POSES)}.map(([a, b]) => l12FK(a, b)),
  jac: {json.dumps(POSES)}.map(([a, b]) => l12Jac(a, b)),
  ik: {json.dumps(TARGETS)}.map(([x, z]) => l12IK(x, z)),
  runs }}));"""
out = json.loads(subprocess.run(["node", "-e", prog], capture_output=True, text=True, check=True).stdout)

model = mujoco.MjModel.from_xml_string(l12.XML)
data = mujoco.MjData(model)
sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "hand")
worst = 0.0

for (q1, q2), k, J in zip(POSES, out["fk"], out["jac"]):
    p = l12.hand_at(model, data, sid, (q1, q2))
    jacp = l12.jac(model, data, sid)[0]
    el = data.xpos[model.body("forearm").id]
    e = max(abs(k["hand"][0] - p[0]), abs(k["hand"][1] - p[2]), abs(k["elbow"][0] - el[0]), abs(k["elbow"][1] - el[2]),
            np.abs(np.array(J) - jacp[[0, 2]]).max())
    worst = max(worst, e)
    print(f"pose ({q1:+.2f}, {q2:+.2f}) rad: hand JS ({k['hand'][0]:.4f}, {k['hand'][1]:.4f}) MuJoCo ({p[0]:.4f}, {p[2]:.4f});"
          f" max |JS - MuJoCo| over elbow, hand, 2x2 Jacobian = {e:.1e}")

for (x, z), r in zip(TARGETS, out["ik"]):
    py = l12.analytic_ik(np.array([x, 0, z]))
    assert len(py) == len(r["sols"]), (x, z)
    e = 0.0
    for (a, b), s in zip(py, r["sols"]):
        p = l12.hand_at(model, data, sid, s)
        e = max(e, abs(a - s[0]), abs(b - s[1]), abs(p[0] - x), abs(p[2] - z))
    worst = max(worst, e)
    print(f"IK target ({x:.2f}, {z:.2f}) m: {len(py)} solution(s) in both; max |JS - Python| and |site_xpos - target| = {e:.1e}")

for (t, q0, lam), qs in zip(RUNS, out["runs"]):
    use = (0, 1, 2) if lam > 0 else (0, 2)
    rows = l12.dls_ik(model, data, sid, np.array([t[0], 0, t[1]]), q0, lam, iters=60, tol=1e-6, use=use)
    n = min(len(rows), len(qs))
    e = max(np.abs(np.array(qs[i]) - rows[i][1]).max() / max(1.0, np.abs(rows[i][1]).max()) for i in range(n))
    worst = max(worst, e)
    print(f"DLS target {t} start {q0} lambda {lam}: {len(qs) - 1} iterations in JS, {len(rows) - 1} in Python;"
          f" max relative |q_JS - q_MuJoCo| = {e:.1e}")
print(f"worst difference: {worst:.1e} -> {'PASS' if worst < 5e-5 else 'FAIL'} (4 decimals)")
