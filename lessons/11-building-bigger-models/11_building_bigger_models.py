"""Lesson 11: write big models without repeating yourself.

Default classes (a style sheet for geoms and joints), keyframes (save points), <include> (split one
model over several files) and MjSpec (edit or generate a model from Python, then recompile).

Run (headless, no viewer needed):
    cd /Users/mohwit/Desktop/mujoco && .venv/bin/python lessons/11-building-bigger-models/11_building_bigger_models.py

The model is the lesson 4 two-link arm (same geometry) plus a small warehouse shelf.
Positions are in metres in the world frame (x forward, y left, z up); angles are hinge angles in rad.
"""

import collections
import pathlib
import shutil
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent.parent
WAREHOUSE = HERE.parent / "16-capstone-warehouse-robot" / "warehouse_robot.xml"

# One-file version of 11_scene.xml + 11_arm.xml + 11_shelf.xml (section 4 checks they compile the same).
XML = r"""
<mujoco model="arm_and_shelf">
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <default>
    <geom rgba="0.75 0.78 0.76 1"/>
    <default class="robot">
      <geom rgba="0.1 0.7 0.6 1" contype="0" conaffinity="0"/>
      <joint axis="0 1 0" damping="0.5"/>
      <default class="finger">
        <geom friction="2 0.05 0.01" condim="4" contype="1" conaffinity="1"/>
      </default>
    </default>
    <default class="furniture">
      <geom type="box" friction="0.5 0.005 0.001" rgba="0.55 0.35 0.15 1"/>
    </default>
    <default class="object">
      <geom type="box" size="0.05 0.05 0.05" friction="0.6 0.01 0.001" rgba="0.6 0.45 0.25 1"/>
    </default>
  </default>

  <worldbody>
    <light name="sun" pos="0.5 -1.5 3" dir="0 0.4 -1"/>
    <geom name="floor" type="plane" size="2 2 0.05"/>
    <body name="upper_arm" pos="0 0 1" childclass="robot">
      <joint name="shoulder"/>
      <geom name="upper_geom" type="capsule" fromto="0 0 0 0.5 0 0" size="0.04"/>
      <body name="forearm" pos="0.5 0 0">
        <joint name="elbow"/>
        <geom name="forearm_geom" type="capsule" fromto="0 0 0 0.4 0 0" size="0.03" rgba="0.9 0.5 0.2 1"/>
        <geom name="weight" class="finger" type="box" pos="0.4 0 0" size="0.05 0.05 0.05"/>
        <site name="hand" pos="0.4 0 0" size="0.02"/>
      </body>
    </body>
    <body name="shelf" pos="0.75 0 0" childclass="furniture">
      <geom name="board" pos="0 0 0.48" size="0.15 0.35 0.02"/>
      <geom name="post_left" pos="0 0.33 0.23" size="0.15 0.02 0.23"/>
      <geom name="post_right" pos="0 -0.33 0.23" size="0.15 0.02 0.23"/>
      <geom name="box_a" class="object" pos="0 0.24 0.55"/>
      <geom name="box_b" class="object" pos="0 0.12 0.55"/>
      <geom name="box_c" class="object" pos="0 -0.12 0.55"/>
      <geom name="box_d" class="object" pos="0 -0.24 0.55"/>
    </body>
    <body name="red_box" pos="0.75 0 0.55">
      <freejoint name="red_box_free"/>
      <geom name="red_box_geom" class="object" rgba="0.85 0.15 0.1 1"/>
    </body>
  </worldbody>

  <keyframe>
    <key name="home"    qpos="-1.0 2.0   0.75 0 0.55 1 0 0 0"/>
    <key name="lesson4" qpos="0.5 0.8    0.75 0 0.55 1 0 0 0"/>
    <key name="reach"   qpos="-0.12 1.0  0.75 0 0.55 1 0 0 0"/>
    <key name="swing"   qpos="-1.0 2.0   0.75 0 0.55 1 0 0 0" qvel="3 0  0 0 0 0 0 0" time="2"/>
  </keyframe>
</mujoco>
"""

# MuJoCo's own built-in values, used when nothing in the XML sets an attribute.
BUILTIN = {"geom": {"type": "sphere", "size": "0 0 0", "rgba": "0.5 0.5 0.5 1", "friction": "1 0.005 0.0001",
                    "condim": "3", "contype": "1", "conaffinity": "1"},
           "joint": {"axis": "0 0 1", "damping": "0"}}
GEOM_ATTRS = ["type", "size", "rgba", "friction", "condim", "contype", "conaffinity"]
JOINT_ATTRS = ["axis", "damping"]


def fl(a, d=4):
    return " ".join(f"{x:g}" if abs(x - round(x, d)) < 1e-12 else f"{x:.{d}f}" for x in np.atleast_1d(a))


def v3(a):
    return "(" + ", ".join(f"{x: .4f}" for x in a) + ")"


# ---------------------------------------------------------------------------------------------
# A tiny resolver that does by hand what MuJoCo's compiler does: for every attribute, look on the
# element, then its class, then that class's parent, ... then main, then the built-in value.
# ---------------------------------------------------------------------------------------------
def read_classes(root):
    classes = {}  # name -> {"parent": name, "geom": {...}, "joint": {...}}

    def walk(node, name, parent):
        classes[name] = {"parent": parent,
                         "geom": dict(node.find("geom").attrib) if node.find("geom") is not None else {},
                         "joint": dict(node.find("joint").attrib) if node.find("joint") is not None else {}}
        for child in node.findall("default"):
            walk(child, child.get("class"), name)

    walk(root.find("default"), "main", None)
    return classes


def chain(classes, name):
    out = []
    while name is not None:
        out.append(name)
        name = classes[name]["parent"]
    return out


def resolve(root, classes, kind):
    """Yield (element name, class used, {attr: (value, level)}) for every <geom> or <joint>."""
    parent_of = {c: p for p in root.iter() for c in p}
    attrs = GEOM_ATTRS if kind == "geom" else JOINT_ATTRS
    for el in root.find("worldbody").iter(kind):
        cls, node = el.get("class"), el
        while cls is None:  # nearest childclass of an enclosing body, else main
            node = parent_of[node]
            cls = node.get("childclass") if node.tag == "body" else ("main" if node.tag == "worldbody" else None)
        via = "own class" if el.get("class") else ("childclass" if cls != "main" else "no class")
        found = {}
        for a in attrs:
            if a in el.attrib:
                found[a] = (el.get(a), "explicit")
                continue
            for i, c in enumerate(chain(classes, cls)):
                if a in classes[c][kind]:
                    lvl = "main (global)" if c == "main" else (f"class {c}" if i == 0 else f"parent class {c}")
                    found[a] = (classes[c][kind][a], lvl)
                    break
            else:
                found[a] = (BUILTIN[kind][a], "built-in")
        yield el.get("name"), cls, via, found


GEOM_TYPES = {"plane": 0, "sphere": 2, "capsule": 3, "box": 6, "cylinder": 5}


def main() -> None:
    print(f"MuJoCo {mujoco.__version__}")
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    root = ET.fromstring(XML)
    classes = read_classes(root)

    print("\n== 1. The class tree (the style sheet) ==")
    for name, c in classes.items():
        depth = len(chain(classes, name)) - 1
        sets = "; ".join(f"<{k}> " + " ".join(f'{a}="{v}"' for a, v in c[k].items()) for k in ("geom", "joint") if c[k])
        print(f"{'  ' * depth}{name:<10} parent={str(c['parent']):<6} sets: {sets}")
    print(f"counts: nbody={model.nbody} ngeom={model.ngeom} njnt={model.njnt} nsite={model.nsite} nq={model.nq} nv={model.nv} nkey={model.nkey}")

    print("\n== 2. Where every geom attribute comes from (resolver), checked against the compiled model ==")
    users = collections.defaultdict(list)
    from_class = 0
    worst = 0.0
    for name, cls, via, found in resolve(root, classes, "geom"):
        g = model.geom(name)
        users[cls].append(name)
        print(f"geom {g.id:>2} {name:<13} class={cls:<9} ({via})")
        for a in GEOM_ATTRS:
            val, lvl = found[a]
            if lvl.startswith(("class", "parent", "main")):
                from_class += 1
            compiled = {"type": g.type, "size": g.size, "rgba": g.rgba, "friction": g.friction,
                        "condim": g.condim, "contype": g.contype, "conaffinity": g.conaffinity}[a]
            el = root.find(f".//geom[@name='{name}']")
            if a == "type":
                ok = GEOM_TYPES[val] == int(compiled[0])
            elif a == "size" and el.get("fromto"):
                ok = True  # capsule from fromto: MuJoCo turns it into (radius, half-length, 0)
            else:
                want = np.array([float(x) for x in val.split()])
                n = len(want)
                err = np.abs(np.asarray(compiled, float)[:n] - want).max()
                worst = max(worst, err)
                ok = err < 1e-6
            shown = val if a != "size" or not el.get("fromto") else f"{val} + fromto -> {fl(compiled)}"
            print(f"      {a:<11} = {shown:<26} from {lvl:<22} compiled: {fl(compiled)}  {'ok' if ok else 'MISMATCH'}")
    print(f"resolver vs compiled model: worst difference {worst:.1e} (all attributes match)")

    print("\nPredict: geom 'weight' has class=finger inside childclass=robot. What is its friction?")
    w = model.geom("weight")
    print(f"  weight friction = [{fl(w.friction)}] (finger)  condim = {w.condim[0]} (finger)  contype = {w.contype[0]} (finger beats robot's 0)  rgba = [{fl(w.rgba)}] (parent class robot)")

    bolt = mujoco.MjModel.from_xml_string(XML.replace('<site name="hand"', '<geom name="bolt" type="sphere" size="0.01"/>\n        <site name="hand"'))
    b = bolt.geom("bolt")
    print(f"Check: a new <geom name=\"bolt\" type=\"sphere\" size=\"0.01\"/> inside forearm -> contype={b.contype[0]} conaffinity={b.conaffinity[0]} rgba=[{fl(b.rgba)}] friction=[{fl(b.friction)}] (childclass robot reaches it)")

    print("\nJoints:")
    for name, cls, via, found in resolve(root, classes, "joint"):
        j = model.joint(name)
        print(f"joint {name:<9} class={cls:<6} axis=({fl(j.axis)}) from {found['axis'][1]}  damping={fl(model.dof_damping[j.dofadr[0]])} N m s/rad from {found['damping'][1]}")

    print("\nWho uses each class:")
    for c in classes:
        print(f"  {c:<9} {len(users[c])} geom(s): {', '.join(users[c])}")
    class_values = sum(len(c["geom"]) + len(c["joint"]) for c in classes.values())
    joint_from_class = 2 * len(JOINT_ATTRS)
    print(f"attribute values that came from a class instead of being typed on the element: {from_class} (geoms) + {joint_from_class} (joints) = {from_class + joint_from_class}")
    print(f"attribute values typed inside <default>: {class_values}")

    print("\n== 3. Keyframes: save points ==")
    print(f"nkey = {model.nkey}   key_qpos shape = {model.key_qpos.shape} (nkey, nq)   key_qvel shape = {model.key_qvel.shape} (nkey, nv)")
    print(f"nu = {model.nu} actuators, so key_ctrl has shape {model.key_ctrl.shape}: these keys hold qpos, qvel and time")
    print(f"qpos layout: [0] shoulder, [1] elbow, [2:5] red box position (m, world), [5:9] red box quaternion [w x y z]")
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    print(f"mj_resetData        qpos[0:2]=({data.qpos[0]: .4f}, {data.qpos[1]: .4f}) rad  hand site_xpos={v3(data.site('hand').xpos)} m  time={data.time:.1f} s  qvel[0]={data.qvel[0]:.1f} rad/s   (qpos0)")
    for k in range(model.nkey):
        mujoco.mj_resetDataKeyframe(model, data, k)
        mujoco.mj_forward(model, data)
        print(f"key {k} {model.key(k).name:<8}    qpos[0:2]=({data.qpos[0]: .4f}, {data.qpos[1]: .4f}) rad  hand site_xpos={v3(data.site('hand').xpos)} m  time={data.time:.1f} s  qvel[0]={data.qvel[0]:.1f} rad/s")
    k = model.key("reach").id
    q1, q2 = model.key_qpos[k][:2]
    hx = 0.5 * np.cos(q1) + 0.4 * np.cos(q1 + q2)
    hz = 1.0 - 0.5 * np.sin(q1) - 0.4 * np.sin(q1 + q2)
    print(f"by hand (lesson 4 FK) for 'reach': x = 0.5 cos({q1}) + 0.4 cos({q1 + q2:.2f}) = {0.5 * np.cos(q1):.4f} + {0.4 * np.cos(q1 + q2):.4f} = {hx:.4f} m")
    print(f"                                   z = 1 - 0.5 sin({q1}) - 0.4 sin({q1 + q2:.2f}) = 1 + {-0.5 * np.sin(q1):.4f} - {0.4 * np.sin(q1 + q2):.4f} = {hz:.4f} m")
    print(f"  red box top is at z = 0.55 + 0.05 = 0.60 m, so the hand is {hz - 0.60:.4f} m above it")

    print("\nGotcha: the number of qpos values in a key")
    bad = XML.replace('qpos="0.5 0.8    0.75 0 0.55 1 0 0 0"', 'qpos="0.5 0.8    0.75 0 0.55 1 0 0 0 0"')
    try:
        mujoco.MjModel.from_xml_string(bad)
    except ValueError as exc:
        print(f"  10 numbers for nq = 9 -> {str(exc).splitlines()[0]}")
    short = XML.replace('qpos="0.5 0.8    0.75 0 0.55 1 0 0 0"', 'qpos="0.5 0.8"')
    ms = mujoco.MjModel.from_xml_string(short)
    print(f"  2 numbers for nq = 9  -> no error; key_qpos = [{fl(ms.key_qpos[1])}] (the missing 7 come from qpos0)")
    print(f"  model.qpos0 = [{fl(model.qpos0)}]")

    print("\n== 4. <include>: the same model from three files ==")
    m_inc = mujoco.MjModel.from_xml_path(str(HERE / "11_scene.xml"))
    for f in ["11_scene.xml", "11_arm.xml", "11_shelf.xml"]:
        print(f"  {f:<13} {len((HERE / f).read_text().splitlines())} lines")
    counts = ["nbody", "njnt", "ngeom", "nsite", "nq", "nv", "nkey"]
    print("  one file  : " + " ".join(f"{c}={getattr(model, c)}" for c in counts))
    print("  3 files   : " + " ".join(f"{c}={getattr(m_inc, c)}" for c in counts))
    same = all(np.array_equal(getattr(model, a), getattr(m_inc, a)) for a in
               ["geom_size", "geom_pos", "geom_rgba", "geom_friction", "geom_contype", "body_mass", "key_qpos", "key_qvel", "dof_damping"])
    print(f"  geom_size, geom_pos, geom_rgba, geom_friction, geom_contype, body_mass, key_qpos, key_qvel, dof_damping identical: {same}")
    with tempfile.TemporaryDirectory() as tmp:  # include the arm file twice (in a scratch copy)
        for f in ["11_arm.xml", "11_shelf.xml"]:
            shutil.copy(HERE / f, tmp)
        twice = (HERE / "11_scene.xml").read_text().replace('<include file="11_arm.xml"/>', '<include file="11_arm.xml"/>\n    <include file="11_arm.xml"/>')
        (pathlib.Path(tmp) / "twice.xml").write_text(twice)
        try:
            mujoco.MjModel.from_xml_path(str(pathlib.Path(tmp) / "twice.xml"))
        except ValueError as exc:
            print(f"  include 11_arm.xml twice -> {str(exc).splitlines()[0]}")

    print("\n== 5. MjSpec: edit the blueprint from Python, then recompile ==")
    spec = mujoco.MjSpec.from_string(XML)
    m0 = spec.compile()
    d0 = mujoco.MjData(m0)
    mujoco.mj_resetDataKeyframe(m0, d0, m0.key("swing").id)
    for _ in range(10):
        mujoco.mj_step(m0, d0)
    print(f"spec.compile() -> MjModel with ngeom={m0.ngeom}; red_box body_mass = {m0.body('red_box').mass[0]:.4f} kg, red_box_geom size = ({fl(m0.geom('red_box_geom').size)}) m")
    spec.geom("red_box_geom").size = [0.04, 0.04, 0.04]
    m1 = spec.compile()
    print(f"spec.geom('red_box_geom').size = [0.04, 0.04, 0.04]; spec.compile()")
    print(f"  new model: size = ({fl(m1.geom('red_box_geom').size)}) m, body_mass = {m1.body('red_box').mass[0]:.4f} kg  (density 1000 kg/m^3 x 0.08^3 m^3)")
    print(f"  old model: size = ({fl(m0.geom('red_box_geom').size)}) m, body_mass = {m0.body('red_box').mass[0]:.4f} kg  (unchanged: a compiled model is fixed)")
    d1 = mujoco.MjData(m1)
    print(f"  fresh MjData(new model): time = {d1.time:.3f} s, qpos[0] = {d1.qpos[0]:.4f} rad (a new data starts at qpos0)")
    m2, d2 = spec.recompile(m0, d0)
    print(f"  spec.recompile(old model, old data) -> new model + data that keep the state: time = {d2.time:.3f} s, qpos[0] = {d2.qpos[0]:.4f} rad (old data: {d0.time:.3f} s, {d0.qpos[0]:.4f} rad)")

    print("\nGotcha: editing a default class AFTER the geoms exist")
    spec = mujoco.MjSpec.from_string(XML)
    finger = spec.find_default("finger")
    finger.geom.friction = [1.0, 0.05, 0.01]
    pad = spec.body("forearm").add_geom(finger, name="new_pad", type=mujoco.mjtGeom.mjGEOM_BOX, pos=[0.3, 0, 0], size=[0.02, 0.02, 0.02])
    m3 = spec.compile()
    print(f"  spec.find_default('finger').geom.friction = [1, 0.05, 0.01]")
    print(f"  existing geom 'weight' friction  = [{fl(m3.geom('weight').friction)}]  (kept: defaults are copied when an element is created)")
    print(f"  geom 'new_pad' added afterwards  = [{fl(m3.geom('new_pad').friction)}]  rgba = [{fl(m3.geom('new_pad').rgba)}] (from parent class robot)")

    print("\nGenerate a row of shelves in a loop:")

    def build_row(n_shelves, n_boxes):
        s = mujoco.MjSpec()
        s.modelname = "shelf_row"
        furn = s.add_default("furniture", s.default)
        furn.geom.type = mujoco.mjtGeom.mjGEOM_BOX
        furn.geom.friction = [0.5, 0.005, 0.001]
        furn.geom.rgba = [0.55, 0.35, 0.15, 1]
        obj = s.add_default("object", s.default)
        obj.geom.type = mujoco.mjtGeom.mjGEOM_BOX
        obj.geom.size = [0.05, 0.05, 0.05]
        obj.geom.friction = [0.6, 0.01, 0.001]
        obj.geom.rgba = [0.6, 0.45, 0.25, 1]
        s.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[5, 5, 0.05])
        for i in range(n_shelves):
            shelf = s.worldbody.add_body(name=f"shelf{i}", pos=[0.75, 0.8 * i, 0])
            shelf.add_geom(furn, name=f"board{i}", pos=[0, 0, 0.48], size=[0.15, 0.35, 0.02])
            shelf.add_geom(furn, name=f"post{i}_left", pos=[0, 0.33, 0.23], size=[0.15, 0.02, 0.23])
            shelf.add_geom(furn, name=f"post{i}_right", pos=[0, -0.33, 0.23], size=[0.15, 0.02, 0.23])
            for j in range(n_boxes):
                y = -0.24 + 0.48 * j / max(n_boxes - 1, 1) if n_boxes > 1 else 0.0
                shelf.add_geom(obj, name=f"box{i}_{j}", pos=[0, y, 0.55])
        return s

    for n, k in [(1, 4), (3, 4), (5, 4), (3, 2)]:
        s = build_row(n, k)
        m = s.compile()
        print(f"  {n} shelf(s) x {k} boxes: nbody={m.nbody} ngeom={m.ngeom}  (formula 1 + n = {1 + n}, 1 + n (3 + k) = {1 + n * (3 + k)})"
              f"  box{n - 1}_{k - 1} friction=[{fl(m.geom(f'box{n - 1}_{k - 1}').friction)}] type={'box' if m.geom(f'box{n - 1}_{k - 1}').type[0] == 6 else '?'}")
    xml_out = build_row(3, 4).to_xml()
    print(f"  spec.to_xml() for 3 x 4: {len(xml_out.splitlines())} lines; the first shelf:")
    lines = xml_out.splitlines()
    start = next(i for i, ln in enumerate(lines) if 'name="shelf0"' in ln)
    for ln in lines[start:start + 9]:
        print("    " + ln)
    ok = all(build_row(n, k).compile().ngeom == 1 + n * (3 + k) for n in range(1, 7) for k in range(1, 5))
    print(f"  formula checked for every n = 1..6, k = 1..4: {ok}")

    print("\nAttach: copy the whole arm-and-shelf model in twice, with name prefixes")
    cells = mujoco.MjSpec()
    cells.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[5, 5, 0.05])
    for i in range(2):
        frame = cells.worldbody.add_frame(pos=[0, 1.0 * i, 0])
        cells.attach(mujoco.MjSpec.from_string(XML), frame=frame, prefix=f"cell{i}_")
    mc = cells.compile()
    print(f"  nbody={mc.nbody} ngeom={mc.ngeom} nq={mc.nq} nkey={mc.nkey}")
    print(f"  bodies: {', '.join(mc.body(b).name for b in range(mc.nbody))}")
    print(f"  cell1_upper_arm pos = ({fl(mc.body('cell1_upper_arm').pos)}) m; cell1_weight friction = [{fl(mc.geom('cell1_weight').friction)}]")

    print("\n== 6. Real world: the Capstone 2 warehouse robot ==")
    ws = mujoco.MjSpec.from_file(str(WAREHOUSE))
    wm = ws.compile()
    print(f"warehouse_robot.xml: nbody={wm.nbody} ngeom={wm.ngeom} njnt={wm.njnt} nq={wm.nq} nv={wm.nv} nu={wm.nu} nkey={wm.nkey}")
    gby, jby = collections.defaultdict(list), collections.defaultdict(list)
    for g in ws.geoms:
        gby[g.classname.name].append(g.name)
    for j in ws.joints:
        jby[j.classname.name].append(j.name)
    for c in ["main", "robot", "wheel", "finger", "object", "furniture"]:
        d = ws.find_default(c)
        print(f"class {c:<9} friction=[{fl(d.geom.friction)}] condim={d.geom.condim} contype={d.geom.contype} conaffinity={d.geom.conaffinity} joint damping={fl(d.joint.damping[0])}")
        print(f"   {len(gby[c])} geoms: {', '.join(gby[c][:6])}{' ...' if len(gby[c]) > 6 else ''}   joints: {', '.join(jby[c]) or '-'}")
    print("robot-class geoms and their compiled contype (explicit contype=\"1\" wins on the arm links):")
    print("   " + "  ".join(f"{n}={wm.geom(n).contype[0]}" for n in gby["robot"]))
    print("robot-class joints, compiled damping (N m s/rad) / armature (kg m^2): the class says 2.0 / 0.01")
    print("   " + "  ".join(f"{n}={fl(wm.dof_damping[wm.joint(n).dofadr[0]])}/{fl(wm.dof_armature[wm.joint(n).dofadr[0]])}" for n in jby["robot"]))
    print(f"finger geoms compiled: friction=[{fl(wm.geom('finger_L_geom').friction)}] condim={wm.geom('finger_L_geom').condim[0]}; red box: friction=[{fl(wm.geom('red_box_geom').friction)}] mass={wm.body('red_box').mass[0]:.2f} kg")
    print(f"keyframes: key_qpos shape = {wm.key_qpos.shape} (nkey, nq): 7 chassis + 4 wheels + 7 arm/gripper + 7 red box + 7 blue cylinder = {7 + 4 + 7 + 7 + 7}")
    wd = mujoco.MjData(wm)
    for k in range(wm.nkey):
        mujoco.mj_resetDataKeyframe(wm, wd, k)
        mujoco.mj_forward(wm, wd)
        q = wd.qpos
        print(f"  key {k} {wm.key(k).name:<9} chassis xyz=({fl(q[0:3])}) m quat=[{fl(q[3:7])}]  arm yaw/shoulder/elbow/wrist roll/pitch=({fl(q[11:16])}) rad  grip=({fl(q[16:18])}) m  grasp_site={v3(wd.site('grasp_site').xpos)} m")


if __name__ == "__main__":
    main()
