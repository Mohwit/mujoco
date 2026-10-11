"""Extra numbers for the Lesson 4 page: exercise answers and gotchas. Run after the main script."""

import importlib.util
import math
import pathlib

import mujoco

HERE = pathlib.Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("l4", HERE / "04_inspection_and_visualization.py")
l4 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(l4)

model = mujoco.MjModel.from_xml_string(l4.XML)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

print("== gotcha: a typo gives id -1, and Python reads row -1 = the LAST row ==")
bad = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "forarm")
print(f"id = {bad}; data.xpos[{bad}] = {l4.v3(data.xpos[bad])} m  (that is the box, body {model.nbody - 1})")
try:
    model.body("forarm")
except Exception as exc:
    print(f"model.body('forarm') raises {type(exc).__name__}: {exc}")

print("\n== check question: shoulder = 0, elbow = pi/2 ==")
data.qpos[0], data.qpos[1] = 0.0, math.pi / 2
mujoco.mj_forward(model, data)
print(f"hand site_xpos = {l4.v3(data.site('hand').xpos)} m; forearm xquat = {l4.v4(data.body('forearm').xquat)}")

print("\n== variant: a hinge added AFTER the free joint ==")
VAR = l4.XML.replace('<geom name="box_geom" type="box" size="0.1 0.1 0.1" rgba="0.3 0.5 0.9 1"/>',
                     '<geom name="box_geom" type="box" size="0.1 0.1 0.1" rgba="0.3 0.5 0.9 1"/>\n'
                     '      <body name="lid" pos="0 0 0.1"><joint name="lid_hinge" axis="0 1 0"/>'
                     '<geom type="box" size="0.1 0.1 0.01"/></body>')
mv = mujoco.MjModel.from_xml_string(VAR)
j = mv.joint("lid_hinge")
print(f"lid_hinge id = {j.id}, jnt_qposadr = {mv.jnt_qposadr[j.id]}, jnt_dofadr = {mv.jnt_dofadr[j.id]}, nq = {mv.nq}, nv = {mv.nv}")

print("\n== read a model from its XML alone ==")
EX = """
<mujoco>
  <worldbody>
    <body name="cart" pos="0 0 0.5">
      <joint name="slider" type="slide" axis="1 0 0"/>
      <geom type="box" size="0.2 0.1 0.05"/>
      <body name="pole" pos="0 0 0.05">
        <joint name="pivot" type="hinge" axis="0 1 0"/>
        <geom type="capsule" fromto="0 0 0 0 0 0.6" size="0.02"/>
        <site name="tip" pos="0 0 0.6"/>
      </body>
    </body>
    <body name="ball" pos="1 0 1">
      <freejoint/>
      <geom type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
"""
me = mujoco.MjModel.from_xml_string(EX)
de = mujoco.MjData(me)
mujoco.mj_forward(me, de)
print(f"nbody={me.nbody} njnt={me.njnt} ngeom={me.ngeom} nsite={me.nsite} nq={me.nq} nv={me.nv}")
print(f"jnt_qposadr={[int(x) for x in me.jnt_qposadr]} jnt_dofadr={[int(x) for x in me.jnt_dofadr]}")
print(f"pole xpos={l4.v3(de.body('pole').xpos)} m  tip site_xpos={l4.v3(de.site('tip').xpos)} m  ball xpos={l4.v3(de.body('ball').xpos)} m")
print(f"qpos0 = {[float(x) for x in me.qpos0]}")
de.qpos[0] = 0.3
mujoco.mj_forward(me, de)
print(f"after qpos[0] = 0.3 (slide): tip site_xpos={l4.v3(de.site('tip').xpos)} m")
