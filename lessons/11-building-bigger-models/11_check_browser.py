"""Check that the browser copy of lesson 11 (sandbox and page) matches MuJoCo.

Runs the CORE11 block from _fragments/script.js in node, then compares with real MuJoCo 3.15:
  1. the default-class resolver: every attribute of every geom, for all 64 on/off combinations
     of the six class toggles (each combination is compiled as its own XML);
  2. the levels (explicit / class / parent class / main / built-in) against the script's resolver;
  3. the arm forward kinematics for every keyframe and a few extra poses (site_xpos, forearm xpos, xmat);
  4. the keyframe data (qpos, qvel, time) against model.key_*.
Run:  cd /Users/mohwit/Desktop/mujoco && .venv/bin/python lessons/11-building-bigger-models/11_check_browser.py
"""

import importlib.util
import itertools
import json
import pathlib
import subprocess
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("l11", HERE / "11_building_bigger_models.py")
l11 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(l11)

js = (HERE / "_fragments" / "script.js").read_text()
core = js.split("// CORE11-BEGIN")[1].split("// CORE11-END")[0]
COMBOS = [list(c) for c in itertools.product([False, True], repeat=6)]
POSES = [(0.0, 0.0), (-1.0, 2.0), (0.5, 0.8), (-0.12, 1.0), (1.3, -2.1), (-2.8, 0.4)]
prog = core + f"""
const combos = {json.dumps(COMBOS)}, poses = {json.dumps(POSES)};
const res = combos.map(on => Object.fromEntries(L11.GEOMS.map(g => [g.name, L11.resolve(g.name, on)])));
console.log(JSON.stringify({{ res, fk: poses.map(([a, b]) => L11.fk(a, b)), keys: L11.KEYS, toggles: L11.TOGGLES }}));
"""
out = json.loads(subprocess.run(["node", "-e", prog], capture_output=True, text=True, check=True).stdout)
TYPES = {"plane": 0, "sphere": 2, "capsule": 3, "box": 6}

# 1. resolver vs compiled model, all 64 toggle combinations
worst, n_checked = 0.0, 0
for on, res in zip(COMBOS, out["res"]):
    root = ET.fromstring(l11.XML)
    defaults = {"main": root.find("default")}
    for d in root.find("default").iter("default"):
        if d.get("class"):
            defaults[d.get("class")] = d
    for flag, t in zip(on, out["toggles"]):
        if flag:
            geom = defaults[t["cls"]].find("geom")
            for a, v in t["set"].items():
                geom.set(a, " ".join(str(x) for x in v) if isinstance(v, list) else str(v))
    m = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    for name, attrs in res.items():
        g = m.geom(name)
        comp = {"size": g.size, "rgba": g.rgba, "friction": g.friction, "condim": g.condim, "contype": g.contype, "conaffinity": g.conaffinity}
        assert TYPES[attrs["type"]["v"]] == int(g.type[0]), (on, name, "type")
        for a, arr in comp.items():
            want = np.atleast_1d(np.array(attrs[a]["v"], float))
            err = np.abs(np.asarray(arr, float)[: len(want)] - want).max()
            worst = max(worst, err)
            n_checked += 1
            assert err < 1e-6, (on, name, a, attrs[a], arr)
print(f"1. resolver: {len(COMBOS)} toggle combinations x 12 geoms x 7 attributes, {n_checked} numeric arrays: worst |JS - MuJoCo| = {worst:.1e} -> PASS")

# 2. levels vs the script's own resolver (default XML)
root = ET.fromstring(l11.XML)
classes = l11.read_classes(root)
mism = 0
for name, cls, via, found in l11.resolve(root, classes, "geom"):
    for a in l11.GEOM_ATTRS:
        if found[a][1] != out["res"][0][name][a]["lvl"]:
            mism += 1
            print("level mismatch", name, a, found[a][1], out["res"][0][name][a]["lvl"])
print(f"2. levels (explicit / class / parent class / main / built-in): {12 * len(l11.GEOM_ATTRS)} checked, {mism} mismatches -> {'PASS' if mism == 0 else 'FAIL'}")

# 3. forward kinematics
model = mujoco.MjModel.from_xml_string(l11.XML)
data = mujoco.MjData(model)
worst = 0.0
for (q1, q2), k in zip(POSES, out["fk"]):
    data.qpos[0], data.qpos[1] = q1, q2
    mujoco.mj_forward(model, data)
    xm = data.geom("weight").xmat
    err = max(np.abs(np.array(k["hand"]) - data.site("hand").xpos).max(), np.abs(np.array(k["elbow"]) - data.body("forearm").xpos).max(),
              np.abs(np.array(k["hand"]) - data.geom("weight").xpos).max(), np.abs(np.array(k["R"]) - xm[[0, 2, 6, 8]]).max())
    worst = max(worst, err)
    print(f"   pose ({q1:+.2f}, {q2:+.2f}) rad: hand JS ({k['hand'][0]:.4f}, {k['hand'][2]:.4f}) MuJoCo ({data.site('hand').xpos[0]:.4f}, {data.site('hand').xpos[2]:.4f}) m  max diff {err:.1e}")
print(f"3. forward kinematics (site_xpos, forearm xpos, weight xpos and xmat): worst {worst:.1e} -> {'PASS' if worst < 1e-6 else 'FAIL'}")

# 4. keyframes
worst = 0.0
for name, k in out["keys"].items():
    if name == "qpos0":
        q, v, t = model.qpos0, np.zeros(model.nv), 0.0
    else:
        i = model.key(name).id
        q, v, t = model.key_qpos[i], model.key_qvel[i], model.key_time[i]
    worst = max(worst, np.abs(np.array(k["qpos"]) - q).max(), abs(k["qvel0"] - v[0]), np.abs(v[1:]).max(), abs(k["time"] - t))
print(f"4. keyframes (qpos, qvel, time) and qpos0: worst {worst:.1e} -> {'PASS' if worst < 1e-12 else 'FAIL'}")
