"""Lesson 07: contact and constraints.

Four small experiments, one idea each:
  A. Collision filtering: which geom pairs are allowed to touch (contype / conaffinity).
  B. Resting contact: read data.contact and mj_contactForce for a box on the floor.
  C. Friction: a box on a tilted ramp sticks below atan(mu) and slides above it.
  D. Margin: contacts that appear before two geoms touch, and a thin plate that a fast ball skips.

Run headless:
  uv run --python .venv/bin/python lessons/07-contact-and-constraints/07_contact_and_constraints.py

Units: metres, seconds, kilograms, newtons. All positions are in the world frame.
"""

import math

import mujoco
import numpy as np

G = 9.81  # m/s^2, MuJoCo's default gravity is (0, 0, -9.81)


# ---------------------------------------------------------------------------
# A. Collision filtering
# ---------------------------------------------------------------------------
FILTER_XML = r"""
<mujoco model="filtering">
  <option timestep="0.002"/>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1" contype="1" conaffinity="1"/>
    <body name="red" pos="-0.4 0 0.5">
      <freejoint/>
      <geom name="red" type="sphere" size="0.1" mass="1" contype="1" conaffinity="1"/>
    </body>
    <body name="green" pos="0 0 0.5">
      <freejoint/>
      <geom name="green" type="sphere" size="0.1" mass="1" contype="2" conaffinity="2"/>
    </body>
    <body name="blue" pos="0.4 0 0.5">
      <freejoint/>
      <geom name="blue" type="sphere" size="0.1" mass="1" contype="0" conaffinity="1"/>
    </body>
  </worldbody>
</mujoco>
"""

# A parent body and its child, joined by a hinge, with geoms that overlap at the joint.
PARENT_CHILD_XML = r"""
<mujoco model="parent_child">
  <option>
    <flag filterparent="{flag}"/>
  </option>
  <worldbody>
    <body name="upper" pos="0 0 1">
      <freejoint/>
      <geom name="upper" type="capsule" fromto="0 0 0 0.3 0 0" size="0.05"/>
      <body name="lower" pos="0.3 0 0">
        <joint name="elbow" type="hinge" axis="0 1 0"/>
        <geom name="lower" type="capsule" fromto="0 0 0 0.3 0 0" size="0.05"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def can_collide(t1: int, a1: int, t2: int, a2: int) -> bool:
    """MuJoCo's filtering rule: (contype1 & conaffinity2) or (contype2 & conaffinity1)."""
    return bool((t1 & a2) or (t2 & a1))


def part_a() -> None:
    print("=" * 72)
    print("A. Collision filtering: contype / conaffinity")
    print("=" * 72)
    model = mujoco.MjModel.from_xml_string(FILTER_XML)
    data = mujoco.MjData(model)
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) for i in range(model.ngeom)]
    print("geom   id  contype  conaffinity   (bits, read from model.geom_contype / geom_conaffinity)")
    for i, n in enumerate(names):
        print(f"{n:<6} {i:>2}  {model.geom_contype[i]:>7}  {model.geom_conaffinity[i]:>11}"
              f"   type={model.geom_contype[i]:02b}  affinity={model.geom_conaffinity[i]:02b}")

    print("\nPredicted pairs with the floor (rule: t1 & a2  or  t2 & a1):")
    f = 0
    for i in range(1, model.ngeom):
        t1, a1 = model.geom_contype[f], model.geom_conaffinity[f]
        t2, a2 = model.geom_contype[i], model.geom_conaffinity[i]
        ok = can_collide(t1, a1, t2, a2)
        print(f"  floor-{names[i]:<6} {t1}&{a2}={t1 & a2}  {t2}&{a1}={t2 & a1}  -> "
              f"{'can collide' if ok else 'ignored (passes through)'}")

    for _ in range(500):  # 1.0 s
        mujoco.mj_step(model, data)
    print(f"\nAfter {data.time:.1f} s of falling (each sphere radius 0.1 m, started at z = 0.5 m):")
    for name in ["red", "green", "blue"]:
        b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        z = data.xpos[b][2]
        print(f"  {name:<6} centre z (world) = {z: .4f} m   -> "
              f"{'resting on the floor' if z > 0 else 'fell through the floor'}")
    pairs = [f"{names[data.contact[i].geom1]}-{names[data.contact[i].geom2]}" for i in range(data.ncon)]
    print(f"  data.ncon = {data.ncon}   pairs in data.contact: {pairs}")

    print("\nParent-child filter: two capsules that overlap at a hinge")
    for flag in ["enable", "disable"]:
        m2 = mujoco.MjModel.from_xml_string(PARENT_CHILD_XML.format(flag=flag))
        d2 = mujoco.MjData(m2)
        mujoco.mj_forward(m2, d2)
        label = "default (parent-child pairs skipped)" if flag == "enable" else "filterparent disabled"
        print(f"  {label:<37} ncon = {d2.ncon}")


# ---------------------------------------------------------------------------
# B. Resting contact: box on the floor
# ---------------------------------------------------------------------------
REST_XML = r"""
<mujoco model="resting_box">
  <option timestep="0.002"/>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1" {solref}/>
    <body name="box" pos="0 0 0.1">
      <freejoint/>
      <geom name="box" type="box" size="0.1 0.1 0.1" mass="1" {solref}/>
    </body>
  </worldbody>
</mujoco>
"""


def part_b() -> None:
    print("\n" + "=" * 72)
    print("B. Resting contact: a 1 kg box (0.2 m cube) on the floor")
    print("=" * 72)
    model = mujoco.MjModel.from_xml_string(REST_XML.format(solref=""))
    data = mujoco.MjData(model)
    for _ in range(500):  # settle for 1.0 s
        mujoco.mj_step(model, data)
    m = model.body_mass[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "box")]
    print(f"time = {data.time:.1f} s   box mass = {m:.1f} kg   weight m*g = {m * G:.2f} N")
    print(f"box centre z (world) = {data.qpos[2]:.6f} m   (0.1 m if there were no overlap)")
    print(f"data.ncon = {data.ncon}   (one contact per box corner touching the floor)")
    f6 = np.zeros(6)
    total = 0.0
    for i in range(data.ncon):
        c = data.contact[i]
        g1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1)
        g2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)
        mujoco.mj_contactForce(model, data, i, f6)  # in the contact frame: [normal, tangent1, tangent2, ...]
        total += f6[0]
        print(f"  contact {i}: geom1={g1} geom2={g2}  dist={c.dist: .6f} m  "
              f"pos=({c.pos[0]: .3f}, {c.pos[1]: .3f}, {c.pos[2]: .6f}) m  "
              f"normal=frame[0:3]=({c.frame[0]:.0f}, {c.frame[1]:.0f}, {c.frame[2]:.0f})  "
              f"normal force={f6[0]:.4f} N")
    print(f"sum of normal forces = {total:.4f} N   vs m*g = {m * G:.4f} N")

    print("\nSoft contact: penetration depends on solref timeconst (default 0.02 s)")
    for sr in ["0.005 1", "0.02 1", "0.05 1"]:
        md = mujoco.MjModel.from_xml_string(REST_XML.format(solref=f'solref="{sr}"'))
        dd = mujoco.MjData(md)
        for _ in range(500):
            mujoco.mj_step(md, dd)
        print(f"  solref=\"{sr}\"  dist = {dd.contact[0].dist * 1000: .3f} mm")


# ---------------------------------------------------------------------------
# C. Friction: box on a ramp
# ---------------------------------------------------------------------------
RAMP_XML = r"""
<mujoco model="ramp">
  <option timestep="0.002" {opt}/>
  <worldbody>
    <geom name="ramp" type="plane" size="5 5 0.1" euler="0 {deg} 0"
          friction="{mu} 0.005 0.0001" condim="{condim}"/>
    <body name="box">
      <freejoint/>
      <geom name="box" type="box" size="0.1 0.1 0.1" mass="1"
            friction="{mu} 0.005 0.0001" condim="{condim}"/>
    </body>
  </worldbody>
</mujoco>
"""


def make_ramp(deg: float, mu: float, condim: int = 3, opt: str = ""):
    """Ramp tilted deg degrees about world y; box placed flat on it, at rest."""
    model = mujoco.MjModel.from_xml_string(RAMP_XML.format(deg=deg, mu=mu, condim=condim, opt=opt))
    data = mujoco.MjData(model)
    th = math.radians(deg)
    normal = np.array([math.sin(th), 0.0, math.cos(th)])     # ramp's up direction (world)
    downhill = np.array([math.cos(th), 0.0, -math.sin(th)])  # along the ramp, going down (world)
    data.qpos[0:3] = 0.1 * normal                            # box centre 0.1 m above the ramp surface
    q = np.zeros(4)
    mujoco.mju_euler2Quat(q, np.array([0.0, th, 0.0]), "xyz")  # same tilt as the ramp
    data.qpos[3:7] = q
    return model, data, downhill


def ramp_run(deg: float, mu: float, condim: int = 3, opt: str = "", t_end: float = 1.0, samples=None):
    """Return downhill displacement (m) after t_end seconds, and optional samples every 0.1 s."""
    model, data, downhill = make_ramp(deg, mu, condim, opt)
    p0 = data.qpos[0:3].copy()
    traj = [0.0]
    n = round(t_end / model.opt.timestep)
    for k in range(1, n + 1):
        mujoco.mj_step(model, data)
        if samples and k % samples == 0:
            traj.append(float((data.qpos[0:3] - p0) @ downhill))
    return float((data.qpos[0:3] - p0) @ downhill), traj


def ideal_slide(deg: float, mu: float, t: float = 1.0) -> float:
    """Coulomb's rule with ideal rigid contact: distance slid from rest in t seconds."""
    th = math.radians(deg)
    a = G * (math.sin(th) - mu * math.cos(th))
    return 0.5 * max(a, 0.0) * t * t


def part_c() -> None:
    print("\n" + "=" * 72)
    print("C. Friction: 1 kg box on a ramp, released from rest, 1.0 s")
    print("=" * 72)
    for mu in [0.2, 0.5, 0.8]:
        print(f"  mu = {mu}: threshold angle atan(mu) = {math.degrees(math.atan(mu)):.2f} deg")

    print("\nForces at 20 deg, mu = 0.5 (from mj_contactForce, summed over the 4 corners):")
    model, data, _ = make_ramp(20, 0.5)
    for _ in range(250):
        mujoco.mj_step(model, data)
    f6 = np.zeros(6)
    nsum, tsum = 0.0, np.zeros(2)
    for i in range(data.ncon):
        mujoco.mj_contactForce(model, data, i, f6)
        nsum += f6[0]
        tsum += f6[1:3]
    th = math.radians(20)
    print(f"  ncon = {data.ncon}")
    print(f"  normal force   N   = {nsum:.4f} N   (m g cos20 = {G * math.cos(th):.4f} N)")
    print(f"  friction force |F| = {np.linalg.norm(tsum):.4f} N   (m g sin20 = {G * math.sin(th):.4f} N)")
    print(f"  friction limit mu N = {0.5 * nsum:.4f} N   -> needed < limit, so it sticks")

    angles = [10, 15, 20, 25, 26, 27, 30, 35, 40]
    print("\nDownhill distance after 1.0 s, MuJoCo (mm).  S = slides (> 10 mm), . = stays put")
    print("angle  tan(angle) |   mu=0.2        mu=0.5        mu=0.8   | ideal mu=0.5 (mm)")
    for a in angles:
        row = []
        for mu in [0.2, 0.5, 0.8]:
            d, _ = ramp_run(a, mu)
            row.append(f"{d * 1000:9.1f} {'S' if d > 0.01 else '.'}  ")
        print(f"{a:>4}   {math.tan(math.radians(a)):.4f}     | " + " ".join(row) +
              f"| {ideal_slide(a, 0.5) * 1000:8.1f}")

    print("\nClose to the threshold, mu = 0.5 (atan(0.5) = 26.57 deg):")
    for a in [26.0, 26.5, 27.0]:
        d, _ = ramp_run(a, 0.5)
        print(f"  {a:>5} deg: MuJoCo {d * 1000:7.2f} mm   ideal {ideal_slide(a, 0.5) * 1000:7.2f} mm")

    print("\nCreep fix: same ramps with option cone=\"elliptic\" impratio=\"10\"")
    for a in [20, 25, 26, 26.5, 27]:
        d0, _ = ramp_run(a, 0.5)
        d1, _ = ramp_run(a, 0.5, opt='cone="elliptic" impratio="10"')
        print(f"  {a:>5} deg: default {d0 * 1000:7.2f} mm   elliptic+impratio {d1 * 1000:7.2f} mm")

    print("\ncondim: same ramp, 10 deg, mu = 0.5")
    for cd in [1, 3]:
        d, _ = ramp_run(10, 0.5, condim=cd)
        print(f"  condim={cd}: {d * 1000:7.1f} mm   ({'no friction at all' if cd == 1 else 'sliding friction on'})")
    print(f"  frictionless ideal 0.5 g sin10 t^2 = {0.5 * G * math.sin(math.radians(10)) * 1000:7.1f} mm")


# ---------------------------------------------------------------------------
# D. Margin
# ---------------------------------------------------------------------------
MARGIN_XML = r"""
<mujoco model="margin">
  <option timestep="0.002"/>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1" margin="{margin}"/>
    <body name="ball" pos="0 0 0.13">
      <freejoint/>
      <geom name="ball" type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"""

PLATE_XML = r"""
<mujoco model="thin_plate">
  <option timestep="0.002"/>
  <worldbody>
    <geom name="plate" type="box" size="0.5 0.5 0.005" margin="{margin}"/>
    <body name="ball" pos="0 0 0.3">
      <freejoint/>
      <geom name="ball" type="sphere" size="0.02" mass="0.1"/>
    </body>
  </worldbody>
</mujoco>
"""


def part_d() -> None:
    print("\n" + "=" * 72)
    print("D. Margin: contacts that start before touching")
    print("=" * 72)
    print("Ball radius 0.1 m, centre at z = 0.13 m: a 0.03 m air gap under it.")
    for mg in [0.0, 0.05]:
        model = mujoco.MjModel.from_xml_string(MARGIN_XML.format(margin=mg))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        ncon0 = data.ncon
        info = f"dist = {data.contact[0].dist:+.3f} m" if ncon0 else "no contact listed"
        for _ in range(1000):  # 2 s: let it drop and settle
            mujoco.mj_step(model, data)
        print(f"  floor margin={mg:.2f}: ncon = {ncon0} at start ({info});"
              f"  after 2 s the ball centre rests at z = {data.qpos[2]:.4f} m")

    print("\nThin plate (1 cm thick), small ball (radius 0.02 m) thrown down at 10 m/s, timestep 0.002 s:")
    for mg in [0.0, 0.05]:
        model = mujoco.MjModel.from_xml_string(PLATE_XML.format(margin=mg))
        data = mujoco.MjData(model)
        data.qvel[2] = -10.0
        for _ in range(250):  # 0.5 s
            mujoco.mj_step(model, data)
        z = data.qpos[2]
        print(f"  plate margin={mg:.2f}: ball z after 0.5 s = {z: .4f} m  -> "
              f"{'went through the plate' if z < 0 else 'stopped on the plate'}")


def main() -> None:
    part_a()
    part_b()
    part_c()
    part_d()


if __name__ == "__main__":
    main()
