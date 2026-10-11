"""Lesson 14: check that the web page's kinematic model is the same maths as a Python copy.

The page cannot run MuJoCo's contact solver, so its sandbox moves the base with a calibrated
kinematic unicycle (v = k_v r (wL + wR)/2, w = r (wR - wL)/B_eff). This script:
  1. runs that unicycle in Python with the calibration numbers stored in 14_recorded_runs.json,
  2. runs the page's own JavaScript (between SIM-CORE-BEGIN and SIM-CORE-END) with node,
  3. compares every pose of every run, and prints the MuJoCo results next to it.

Run (needs node):
    cd /Users/mohwit/Desktop/mujoco && .venv/bin/python lessons/14-mobile-bases/14_check_browser_copy.py
"""

import json
import subprocess
from pathlib import Path

import numpy as np

HERE = Path(__file__).parent
REC = json.loads((HERE / "14_recorded_runs.json").read_text())
R, B, KV, DT = 0.06, REC["B_eff"], REC["k_v"], 0.01
ROUTE = [(1.5, 0.0), (1.5, 1.0)]


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def step(s, cl, cr, bias):
    crb = cr * (1 + bias)
    v = KV * R * (cl + crb) / 2
    w = R * (crb - cl) / B
    s["th"] += w * DT
    s["x"] += v * np.cos(s["th"]) * DT
    s["y"] += v * np.sin(s["th"]) * DT
    s["t"] += DT
    s["wz"] = w


def closed(wps, bias):
    s = dict(x=0.0, y=0.0, th=0.0, t=0.0, wz=0.0)
    path = [(s["t"], s["x"], s["y"], s["th"])]
    k = 0
    while s["t"] < 60 and k < len(wps):
        dx, dy = wps[k][0] - s["x"], wps[k][1] - s["y"]
        dist = np.hypot(dx, dy)
        if dist < 0.05:
            k += 1
            continue
        he = wrap(np.arctan2(dy, dx) - s["th"])
        turn = np.clip(6 * he - 2 * s["wz"], -6, 6)
        al = max(0.0, 1 - abs(he) / 0.8)
        creep = 1.5 if abs(he) < 0.4 else 0.0
        sp = np.clip(max(4 * dist * al, creep), 0, 6)
        step(s, np.clip(sp - turn, -6, 6), np.clip(sp + turn, -6, 6), bias)
        path.append((s["t"], s["x"], s["y"], s["th"]))
    return np.array(path)


def open_(wps, bias):
    s = dict(x=0.0, y=0.0, th=0.0, t=0.0, wz=0.0)
    path = [(s["t"], s["x"], s["y"], s["th"])]
    bx = by = bth = 0.0

    def run(cl, cr, T):
        for _ in range(int(round(T / DT))):
            step(s, cl, cr, bias)
            path.append((s["t"], s["x"], s["y"], s["th"]))
    for wx, wy in wps:
        d = wrap(np.arctan2(wy - by, wx - bx) - bth)
        if abs(d) > 1e-9:
            run(-5 if d > 0 else 5, 5 if d > 0 else -5, abs(d) / (R * 10 / B))
            run(0, 0, 0.5)
        run(5, 5, np.hypot(wx - bx, wy - by) / (KV * R * 5))
        run(0, 0, 0.5)
        bth += d
        bx, by = wx, wy
    return np.array(path)


def main():
    js = (HERE / "_fragments" / "script.js").read_text()
    core = js.split("// SIM-CORE-BEGIN")[1].split("// SIM-CORE-END")[0]
    cases = [("closed", ROUTE, 0.0), ("closed", ROUTE, -0.10), ("closed", [(-0.5, 1.2)], 0.0),
             ("open", ROUTE, 0.0), ("open", ROUTE, -0.10), ("open", [(-0.5, 1.2)], 0.2)]
    node_src = ("const REC = " + json.dumps({"B_eff": B, "k_v": KV}) + ";\n" + core +
                "\nconst cases = " + json.dumps([[c, [list(p) for p in w], b] for c, w, b in cases]) + ";\n"
                "console.log(JSON.stringify(cases.map(([k, w, b]) => (k === 'open' ? k14Open(w, b) : k14Closed(w, b)).path.map(p => p.slice(0, 4)))));")
    out = json.loads(subprocess.run(["node", "-e", node_src], capture_output=True, text=True, check=True).stdout)
    worst = 0.0
    for (kind, wps, bias), jpath in zip(cases, out):
        p = (open_ if kind == "open" else closed)(wps, bias)
        j = np.array(jpath)
        n = min(len(p), len(j))
        diff = np.abs(p[:n] - j[:n]).max()
        worst = max(worst, diff)
        end = np.hypot(p[-1, 1] - wps[-1][0], p[-1, 2] - wps[-1][1])
        print(f"{kind:6s} to {wps} bias {bias:+.2f}: steps python {len(p)} / js {len(j)},"
              f" largest pose difference {diff:.2e}, final error {end:.4f} m, time {p[-1, 0]:.2f} s")
    print(f"largest difference over all runs: {worst:.2e} (m or rad)")
    nom = REC["runs"]["nominal"]
    print(f"MuJoCo on the same route (recorded): closed loop error {nom['closed']['err']} m in {nom['closed']['t']} s;"
          f" open loop error {nom['open']['err']} m")


if __name__ == "__main__":
    main()
