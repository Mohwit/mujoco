"""Lesson 04: find parts of a model by name, read where they are, and take pictures headlessly."""

import math
import pathlib
import struct
import zlib

import mujoco
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent

# A two-link arm in the x-z plane (seen from the side: x right, z up), plus a loose box.
# Physical reasons for the numbers:
#   base at z = 1.0 m        about shoulder height on a table-top stand, so the arm can swing down
#   upper arm 0.50 m, r 0.04 roughly a human upper arm, a broomstick-thick capsule
#   forearm   0.40 m, r 0.03 a bit shorter and thinner than the upper arm, like ours
#   weight: 0.1 m cube       a 1 kg "tool" held at the tip, so the forearm's centre of mass
#                            is NOT at its middle (and not at its frame origin)
#   box: 0.2 m cube at x 1.2 out of the arm's reach (0.9 m), resting on the floor (half size 0.1)
#                            turned 30 degrees about the vertical, so its quaternion is not trivial
XML = r"""
<mujoco model="two_link_arm">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <light name="sun" pos="0.5 -1.5 3" dir="0 0.4 -1"/>
    <geom name="floor" type="plane" size="2 2 0.05" rgba="0.75 0.78 0.76 1"/>
    <camera name="side" pos="0.6 -3.0 0.7" xyaxes="1 0 0 0 0 1"/>
    <body name="upper_arm" pos="0 0 1">
      <joint name="shoulder" type="hinge" axis="0 1 0"/>
      <geom name="upper_geom" type="capsule" fromto="0 0 0 0.5 0 0" size="0.04" rgba="0.1 0.7 0.6 1"/>
      <body name="forearm" pos="0.5 0 0">
        <joint name="elbow" type="hinge" axis="0 1 0"/>
        <geom name="forearm_geom" type="capsule" fromto="0 0 0 0.4 0 0" size="0.03" rgba="0.9 0.5 0.2 1"/>
        <geom name="weight" type="box" pos="0.4 0 0" size="0.05 0.05 0.05" rgba="0.5 0.4 0.8 1"/>
        <site name="hand" pos="0.4 0 0" size="0.02"/>
        <camera name="track" mode="trackcom" pos="0.29 -1.2 0.2" xyaxes="1 0 0 0 0 1"/>
      </body>
    </body>
    <body name="box" pos="1.2 0 0.1" euler="0 0 30">
      <freejoint name="box_free"/>
      <geom name="box_geom" type="box" size="0.1 0.1 0.1" rgba="0.3 0.5 0.9 1"/>
    </body>
  </worldbody>
</mujoco>
"""

KINDS = [  # (label, mjtObj, count attribute)
    ("body", mujoco.mjtObj.mjOBJ_BODY, "nbody"),
    ("joint", mujoco.mjtObj.mjOBJ_JOINT, "njnt"),
    ("geom", mujoco.mjtObj.mjOBJ_GEOM, "ngeom"),
    ("site", mujoco.mjtObj.mjOBJ_SITE, "nsite"),
    ("camera", mujoco.mjtObj.mjOBJ_CAMERA, "ncam"),
]
JNT_TYPE = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}


def v3(a) -> str:
    return "(" + ", ".join(f"{x: .4f}" for x in a) + ")"


def v4(a) -> str:
    return "[" + ", ".join(f"{x: .4f}" for x in a) + "]"


def save_png(path: pathlib.Path, rgb: np.ndarray) -> None:
    """Write an RGB uint8 image as PNG with the standard library only."""
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))
    def chunk(tag, body):
        return struct.pack(">I", len(body)) + tag + body + struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")
    path.write_bytes(png)


def main() -> None:
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)

    print("== 1. Names are for people, ids are for arrays ==")
    for label, obj, count in KINDS:
        n = getattr(model, count)
        names = [f"{i}:{mujoco.mj_id2name(model, obj, i) or '(no name)'}" for i in range(n)]
        print(f"{label:<7} {count:<6}= {n}   " + "  ".join(names))
    fid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "forearm")
    print(f"mj_name2id(model, mjOBJ_BODY, 'forearm') = {fid}")
    print(f"model.body('forearm').id                  = {model.body('forearm').id}")
    print(f"mj_name2id(model, mjOBJ_BODY, 'forarm')  = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'forarm')}   (typo: -1 means not found)")

    print("\n== 2. Zero pose (all joint angles 0), after mj_forward ==")
    mujoco.mj_forward(model, data)
    for b in range(model.nbody):
        name = model.body(b).name or "world"
        print(f"body {b} {name:<9} xpos={v3(data.xpos[b])} m  xquat={v4(data.xquat[b])}  xipos={v3(data.xipos[b])} m")
    print(f"site hand site_xpos={v3(data.site('hand').xpos)} m (world)")
    print(f"forearm mass: capsule {model.geom('forearm_geom').size[0]:.2f} m radius + 1 kg cube -> body_mass = {model.body_mass[fid]:.4f} kg")
    print(f"forearm body_ipos (centre of mass in its own frame) = {v3(model.body_ipos[fid])} m")
    print(f"upper_arm body_ipos = {v3(model.body_ipos[model.body('upper_arm').id])} m")

    print("\n== 3. Stale values: set qpos, read BEFORE mj_forward ==")
    sh, el = 0.5, 0.8
    data.joint("shoulder").qpos[0] = sh
    data.joint("elbow").qpos[0] = el
    print(f"qpos[0..1] = {sh}, {el} rad set; site_xpos(hand) before mj_forward = {v3(data.site('hand').xpos)} m  (stale: still the zero pose)")
    mujoco.mj_forward(model, data)
    print(f"site_xpos(hand) after  mj_forward = {v3(data.site('hand').xpos)} m (world)")

    print("\n== 4. Pose shoulder = 0.5 rad, elbow = 0.8 rad ==")
    for name in ["upper_arm", "forearm"]:
        b = data.body(name)
        print(f"{name:<9} xpos={v3(b.xpos)} m  xquat={v4(b.xquat)}  xipos={v3(b.xipos)} m")
    print("forearm xmat (3x3, rows; columns are the body's x, y, z axes in world):")
    for row in data.body("forearm").xmat.reshape(3, 3):
        print("   " + v3(row))
    # forward kinematics by hand
    c1, s1, c12, s12 = math.cos(sh), math.sin(sh), math.cos(sh + el), math.sin(sh + el)
    elbow = np.array([0.5 * c1, 0.0, 1.0 - 0.5 * s1])
    hand = elbow + np.array([0.4 * c12, 0.0, -0.4 * s12])
    print(f"by hand: cos(0.5)={c1:.5f} sin(0.5)={s1:.5f} cos(1.3)={c12:.5f} sin(1.3)={s12:.5f}")
    print(f"by hand: elbow = {v3(elbow)} m   hand = {v3(hand)} m")
    print(f"max |by hand - MuJoCo| for hand = {np.abs(hand - data.site('hand').xpos).max():.1e} m")
    print(f"by hand: rotation about y by 1.3 rad -> [cos(0.65), 0, sin(0.65), 0] = {v4([math.cos(0.65), 0, math.sin(0.65), 0])}")

    print("\n== 5. Where each joint lives in qpos and qvel ==")
    print(f"nq = {model.nq} (numbers in qpos)   nv = {model.nv} (numbers in qvel)")
    for j in range(model.njnt):
        jt = JNT_TYPE[int(model.jnt_type[j])]
        nqj = {"free": 7, "ball": 4}.get(jt, 1)
        nvj = {"free": 6, "ball": 3}.get(jt, 1)
        a, d = model.jnt_qposadr[j], model.jnt_dofadr[j]
        print(f"joint {j} {model.joint(j).name:<8} type={jt:<5} qposadr={a} ({nqj} number{'s' if nqj > 1 else ''}: qpos[{a}:{a + nqj}])  dofadr={d} ({nvj} number{'s' if nvj > 1 else ''}: qvel[{d}:{d + nvj}])")
    a = model.jnt_qposadr[model.joint("box_free").id]
    print(f"box free joint qpos: position {v3(data.qpos[a:a + 3])} m, quaternion {v4(data.qpos[a + 3:a + 7])}")
    print(f"box xquat (world)   = {v4(data.body('box').xquat)}   by hand [cos(15deg), 0, 0, sin(15deg)] = {v4([math.cos(math.radians(15)), 0, 0, math.sin(math.radians(15))])}")
    print(f"full qpos (shape {data.qpos.shape}) = {np.array2string(data.qpos, precision=4, suppress_small=True)}")

    print("\n== 6. Cameras ==")
    modes = {0: "fixed", 1: "track", 2: "trackcom", 3: "targetbody", 4: "targetbodycom"}
    for c in range(model.ncam):
        print(f"camera {c} {model.camera(c).name:<6} mode={modes[int(model.cam_mode[c])]:<9} cam_xpos at pose (0.5, 0.8) = {v3(data.cam_xpos[c])} m")
    poses = {"zero": (0.0, 0.0), "bent": (sh, el)}
    try:
        renderer = mujoco.Renderer(model, 240, 320)
        for pname, (q0, q1) in poses.items():
            data.qpos[0], data.qpos[1] = q0, q1
            mujoco.mj_forward(model, data)
            for cam in ["side", "track"]:
                renderer.update_scene(data, camera=cam)
                path = HERE / f"cam_{cam}_{pname}.png"
                save_png(path, renderer.render())
                print(f"saved {path.name}  (320 x 240 px, camera '{cam}', pose {pname}: track cam_xpos = {v3(data.cam('track').xpos)} m)")
        renderer.close()
    except Exception as exc:  # no OpenGL context available (for example over SSH)
        print(f"rendering skipped: {type(exc).__name__}: {exc}")
        print("Everything above is still valid; pictures need an OpenGL context.")


if __name__ == "__main__":
    main()
