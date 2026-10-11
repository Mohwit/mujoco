"""Lesson 06 helper: check that the page's in-browser pendulum + sensors match MuJoCo.

The page (sandbox 6 and the lesson pictures) uses this exact update, copied from script.js:
    a  = (-K * sin(q) + 100 * u) * INV_I        # qacc, rad/s^2 (gear 100)
    sensordata = [q, v, u, -sin q, 0, 1 - cos q, -a - g sin q, 0, g cos q + v^2]
    v += h * a;  q += h * v                     # MuJoCo's Euler (semi-implicit)
Sensors are computed BEFORE the integration, exactly like mj_step.
"""

import importlib.util
import pathlib

import mujoco
import numpy as np

spec = importlib.util.spec_from_file_location("l6", pathlib.Path(__file__).with_name("06_sensors_and_observation.py"))
l6 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(l6)

model = mujoco.MjModel.from_xml_string(l6.XML)
data = mujoco.MjData(model)

# Constants measured from MuJoCo (printed so they can be pasted into script.js).
mujoco.mj_forward(model, data)
INERTIA = data.M[0]                      # kg m^2 about the hinge
data.qpos[0] = np.pi / 2
mujoco.mj_forward(model, data)
MGD = data.qfrc_bias[0]                  # m g d, N m (gravity torque at 90 degrees)
K, INV_I = MGD, 1.0 / INERTIA
print(f"I about hinge = {INERTIA:.15g} kg m^2, m g d = {MGD:.15g} N m")
print(f"mgd / I = {MGD / INERTIA:.15g} 1/s^2, 1 / I = {INV_I:.15g} 1/(kg m^2)")

G, H = 9.81, 0.002


def js_step(q, v, u):
    a = (-K * np.sin(q) + 100.0 * u) * INV_I
    sens = np.array([q, v, u, -np.sin(q), 0.0, 1.0 - np.cos(q), -a - G * np.sin(q), 0.0, G * np.cos(q) + v * v])
    v = v + H * a
    q = q + H * v
    return q, v, sens


mujoco.mj_resetData(model, data)
q, v = 0.7, 0.0
data.qpos[0] = q
worst = np.zeros(model.nsensordata)
worst_state = 0.0
for i in range(5000):  # 10 s with a ctrl that changes every 0.5 s
    u = [0.0, 1.0, -1.0, 0.5, -0.3][(i // 250) % 5]
    data.ctrl[0] = u
    mujoco.mj_step(model, data)
    q, v, sens = js_step(q, v, u)
    worst = np.maximum(worst, np.abs(sens - data.sensordata))
    worst_state = max(worst_state, abs(q - data.qpos[0]), abs(v - data.qvel[0]))
print("max |browser - MuJoCo| per sensordata element over 5000 steps:")
print("  " + " ".join(f"{w:.1e}" for w in worst))
print(f"max |browser - MuJoCo| for qpos, qvel: {worst_state:.1e}")
print("PASS: matches to 4 decimals" if max(worst.max(), worst_state) < 5e-5 else "FAIL")
