"""Record the hold-or-slip grid that Sandbox 13 replays.

For every (squeeze setting, box mass, finger friction) it runs the lesson's grasp:
close 0.5 s, lift 10 cm in 1.0 s, hold 0.5 s, and records the box's slip relative
to the hand every 0.1 s (15 samples, in units of 0.1 mm).
It also records the measured squeeze per finger for each setting (touch sensor while
a light 0.05 kg box is held with mu 2.0).

Run:
  uv run --python .venv/bin/python lessons/13-grippers-and-grasping/record_grip_grid.py
Writes grip_grid.json next to this file (pasted into _fragments/script.js).
"""
import importlib.util
import json
import pathlib

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("l13", HERE / "13_grippers_and_grasping.py")
L = importlib.util.module_from_spec(spec)
spec.loader.exec_module(L)

SETS = [0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0]
MASSES = [0.1, 0.25, 0.5, 0.75, 1.0]
MUS = [0.6, 0.8, 1.0, 1.5, 2.0]

rig = L.Rig()
meas = [round(float(np.mean(L.grasp_and_lift(rig, s, 0.05, 2.0)["touch"])), 4) for s in SETS]
slip = []          # slip[setIndex][massIndex][muIndex] = 15 samples, 0.1 mm
agree = 0
for i, s in enumerate(SETS):
    rows = []
    for mass in MASSES:
        row = []
        for mu in MUS:
            rec = []
            r = L.grasp_and_lift(rig, s, mass, mu, record=rec)
            row.append([int(round(v * 1e4)) for v in rec])
            agree += L.rule_holds(mu, meas[i], mass) == r["held"]
        rows.append(row)
    slip.append(rows)
total = len(SETS) * len(MASSES) * len(MUS)
out = {"sets": SETS, "masses": MASSES, "mus": MUS, "meas": meas, "slip": slip}
(HERE / "grip_grid.json").write_text(json.dumps(out, separators=(",", ":")))
print("measured squeeze per finger (N):", dict(zip(SETS, meas)))
print(f"rule 2 mu N >= m g agrees with MuJoCo in {agree} of {total} runs")
for i, s in enumerate(SETS):
    for j, m in enumerate(MASSES):
        for k, mu in enumerate(MUS):
            held = slip[i][j][k][-1] < 100
            if L.rule_holds(mu, meas[i], m) != held:
                print(f"  disagree: set {s} N, mass {m} kg, mu {mu}: rule {'hold' if not held else 'slip'}, "
                      f"MuJoCo slip {slip[i][j][k][-1]/10:.1f} mm, 2 mu N = {2*mu*meas[i]:.3f}, m g = {m*L.G:.3f}")
A_PEAK = L.LIFT / 2 * (np.pi / L.T_LIFT) ** 2      # m/s^2, peak upward acceleration of the smooth lift
agree_dyn = sum((2 * mu * meas[i] >= m * (L.G + A_PEAK)) == (slip[i][j][k][-1] < 100)
                for i in range(len(SETS)) for j, m in enumerate(MASSES) for k, mu in enumerate(MUS))
print(f"with the lift's peak acceleration ({A_PEAK:.3f} m/s^2): 2 mu N >= m (g + a) agrees in {agree_dyn} of {total} runs")
