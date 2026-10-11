"""Lesson 05 helper: check that the page's in-browser stepper reproduces MuJoCo.

The page steps:  force = gain*clamp(ctrl) + bias(q, v);  tau = gear*force
                 a = (tau - m*g*d*sin q) / I;  v += h*a;  q += h*v
MuJoCo's default Euler integrator treats only JOINT damping implicitly, so the
kp/kv terms of position/velocity actuators are ordinary explicit forces.
"""

import importlib.util
import pathlib

import numpy as np
import mujoco

spec = importlib.util.spec_from_file_location(
    "l5", pathlib.Path(__file__).with_name("05_actuators_and_control.py"))
l5 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(l5)

H = 0.002
m0, d0 = l5.build(l5.MOTOR)
mujoco.mj_forward(m0, d0)
Mfull = np.zeros((1, 1))
mujoco.mj_fullM(m0, d0, Mfull)
I = Mfull[0, 0]
MGD = m0.body_mass[1] * 9.81 * 0.5
print(f"constants for the page: I={I!r}  MGD={MGD!r}")


def page_step(kind, par, ctrl, lo, hi, q, v, n):
    c = min(max(ctrl, lo), hi)
    out = []
    for _ in range(n):
        if kind == "motor":
            tau = par * c
        elif kind == "position":
            tau = par * (c - q)
        else:
            tau = par * (c - v)
        a = (tau - MGD * np.sin(q)) / I
        v += H * a
        q += H * v
        out.append((q, v))
    return np.array(out)


def xml_for(kind, par):
    base = {"motor": l5.MOTOR, "position": l5.POSITION, "velocity": l5.VELOCITY}[kind]
    return (base.replace('gear="100"', f'gear="{par}"').replace('kp="500"', f'kp="{par}"')
            .replace('kv="200"', f'kv="{par}"'))


cases = [("motor", 100, 0.5, -1, 1), ("motor", 200, 1.5, -1, 1), ("motor", 20, -0.7, -1, 1),
         ("position", 500, 0.5, -1.57, 1.57), ("position", 2000, 1.57, -1.57, 1.57),
         ("position", 50, -1.0, -1.57, 1.57),
         ("velocity", 200, 0.5, -2, 2), ("velocity", 1000, 2.0, -2, 2), ("velocity", 20, -2.0, -2, 2)]
worst = 0.0
for kind, par, ctrl, lo, hi in cases:
    mdl, dat = l5.build(xml_for(kind, par))
    dat.qpos[0], dat.ctrl[0] = 0.3, ctrl
    tr = []
    for _ in range(5000):  # 10 s
        mujoco.mj_step(mdl, dat)
        tr.append((dat.qpos[0], dat.qvel[0]))
    diff = np.abs(np.array(tr) - page_step(kind, par, ctrl, lo, hi, 0.3, 0.0, 5000)).max()
    worst = max(worst, diff)
    print(f"{kind:<8} par={par:<5} ctrl={ctrl:+.2f}  10 s, max |qpos,qvel diff| = {diff:.1e}")
print(f"worst difference over all cases: {worst:.1e}  (4-decimal match needs < 5e-5)")

# What if MuJoCo used implicitfast? The kv term becomes implicit, so numbers change slightly.
mdl, dat = l5.build(xml_for("velocity", 1000))
mdl.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
dat.ctrl[0] = 2.0
for _ in range(500):
    mujoco.mj_step(mdl, dat)
ref = page_step("velocity", 1000, 2.0, -2, 2, 0.0, 0.0, 500)[-1]
print(f"velocity kv=1000 ctrl=2, 1 s: Euler/page qvel={ref[1]:.5f}, implicitfast qvel={dat.qvel[0]:.5f}")

# Extra numbers used on the page.
mdl, dat = l5.build(l5.MOTOR)
dat.ctrl[0] = 0.5
mujoco.mj_forward(mdl, dat)
print(f"motor ctrl=0.5 at qpos=0: qacc = {dat.qacc[0]:.5f} rad/s^2  (50 / I = {50 / I:.5f})")
dat.qpos[0] = 0.5
mujoco.mj_forward(mdl, dat)
print(f"motor ctrl=0.5 at qpos=0.5: qacc = {dat.qacc[0]:.5f} rad/s^2  ((50 - MGD sin 0.5)/I = {(50 - MGD*np.sin(0.5)) / I:.5f})")
qs = np.arcsin(50 / MGD)
print(f"motor ctrl=0.5 balance angle asin(50/MGD) = {qs:.5f} rad")
# position: 500 (0.5 - q) = MGD sin q
q = 0.4
for _ in range(50):
    q -= (500 * (0.5 - q) - MGD * np.sin(q)) / (-500 - MGD * np.cos(q))
print(f"servo ctrl=0.5 balance angle = {q:.5f} rad, droop below target = {0.5 - q:.5f} rad")
print(f"velocity ctrl=0.5 stall angle asin(100/MGD) = {np.arcsin(100 / MGD):.5f} rad")
for kind, par in (("motor", 100), ("position", 500), ("velocity", 200)):
    mdl, dat = l5.build(xml_for(kind, par))
    dat.ctrl[0] = 0.5
    for i in range(1, 5001):
        mujoco.mj_step(mdl, dat)
        if i in (2500, 5000):
            print(f"{kind:<8} ctrl=0.5 t={dat.time:.1f} s qpos={dat.qpos[0]:.5f} qvel={dat.qvel[0]:.5f}")
