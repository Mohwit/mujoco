"""Helper for Lesson 07's sandbox: record box-on-ramp trajectories from MuJoCo.

Prints a JavaScript-ready JSON object (pasted into _fragments/script.js). For every ramp
angle 0, 2.5, ..., 45 deg and friction mu 0.1, 0.2, ..., 1.0 (condim 3), plus condim 1 for
each angle, it stores the box's downhill distance every 0.1 s for 1.0 s, in units of 0.1 mm.

Run:
  uv run --python .venv/bin/python lessons/07-contact-and-constraints/record_ramp_grid.py
"""
import importlib.util
import json
import pathlib

spec = importlib.util.spec_from_file_location(
    "l7", pathlib.Path(__file__).with_name("07_contact_and_constraints.py"))
l7 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(l7)

ANGLES = [i * 2.5 for i in range(19)]
MUS = [round(0.1 * i, 1) for i in range(1, 11)]


def rec(deg, mu, condim):
    _, traj = l7.ramp_run(deg, mu, condim=condim, samples=50)  # every 50 steps = 0.1 s
    return [round(x * 10000) for x in traj]


grid = {"angles": ANGLES, "mus": MUS,
        "c3": [[rec(a, mu, 3) for mu in MUS] for a in ANGLES],
        "c1": [rec(a, 0.5, 1) for a in ANGLES]}
print(json.dumps(grid, separators=(",", ":")))
