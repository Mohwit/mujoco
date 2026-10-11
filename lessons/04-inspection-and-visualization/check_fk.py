"""Check that the in-browser forward kinematics (sandbox 4) matches MuJoCo to 4 decimals.

Runs the exact FK block from _fragments/script.js in node, then compares every number with MuJoCo.
"""

import importlib.util
import json
import pathlib
import subprocess

import mujoco
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("l4", HERE / "04_inspection_and_visualization.py")
l4 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(l4)

POSES = [(0.0, 0.0), (0.5, 0.8), (0.0, 1.5708), (0.0, 1.57), (-1.2, 2.3), (3.0, -2.5), (2.5, 2.5), (-3.14, 1.0)]

js = (HERE / "_fragments" / "script.js").read_text()
block = js.split("// FK-BEGIN")[1].split("// FK-END")[0]
prog = block + f"\nconsole.log(JSON.stringify({json.dumps(POSES)}.map(([a, b]) => L4FK(a, b))));"
out = json.loads(subprocess.run(["node", "-e", prog], capture_output=True, text=True, check=True).stdout)

model = mujoco.MjModel.from_xml_string(l4.XML)
data = mujoco.MjData(model)
worst = 0.0
for (q1, q2), k in zip(POSES, out):
    data.qpos[0], data.qpos[1] = q1, q2
    mujoco.mj_forward(model, data)
    pairs = [
        (np.array(k["xpos"]), data.xpos), (np.array(k["xquat"]), data.xquat), (np.array(k["xipos"]), data.xipos),
        (np.array(k["hand"]), data.site("hand").xpos), (np.array(k["cam"]), data.cam_xpos),
    ]
    err = max(np.abs(a - b).max() for a, b in pairs)
    worst = max(worst, err)
    print(f"pose ({q1:+.4f}, {q2:+.4f}) rad: hand JS {l4.v3(k['hand'])} MuJoCo {l4.v3(data.site('hand').xpos)}  max |JS - MuJoCo| over xpos, xquat, xipos, site, cam = {err:.1e}")
print(f"worst difference over {len(POSES)} poses: {worst:.1e}  ->  {'PASS' if worst < 5e-5 else 'FAIL'} (4 decimals)")
