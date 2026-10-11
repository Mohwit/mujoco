"""Lesson 13: grippers and grasping.

A small parallel-jaw gripper, built from the warehouse robot's gripper
(lessons/16-capstone-warehouse-robot/warehouse_robot.xml), picks up a box.

Five small experiments, one idea each:
  A. The model: which XML element becomes which array.
  B. Coupling two fingers: a fixed tendon plus an equality constraint (and the joint-equality form).
  C. A position actuator as a squeeze: kp x (finger position - target), shared by two fingers.
  D. Hold or slip: the rule 2 mu N >= m g against MuJoCo, plus impratio and condim.
  E. Touch sensors versus contact pairs: what you feel versus what you touch.

Run headless:
  uv run --python .venv/bin/python lessons/13-grippers-and-grasping/13_grippers_and_grasping.py

Units: metres, seconds, kilograms, newtons. Positions are in the world frame
(x forward, y to the left, z up) unless a line says otherwise.
"""

import mujoco
import numpy as np

G = 9.81          # m/s^2, the model's gravity is (0, 0, -9.81)
KP_GRIP = 600.0   # N/m, gripper position actuator gain (from the warehouse robot)
Q_TOUCH = 0.030   # m, finger opening at which the inner faces just touch the 7 cm box
LIFT = 0.10       # m, how high the hand lifts
T_CLOSE, T_LIFT, T_HOLD = 0.5, 1.0, 0.5   # s: close the fingers, lift, then hold still
HELD_MM = 10.0    # a run counts as "held" if the box slid less than this relative to the hand

# Values marked (warehouse) are copied from warehouse_robot.xml.
GRIPPER_XML = r"""
<mujoco model="gripper">
  <compiler angle="radian" autolimits="true"/>
  <!-- (warehouse) implicit integrator, Newton solver, elliptic friction cone, impratio 10 -->
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicit"
          solver="Newton" iterations="30" cone="elliptic" impratio="10"/>
  <default>
    <!-- (warehouse) finger class: very grippy -->
    <default class="finger">
      <geom rgba="0.7 0.25 0.1 1" friction="2.0 0.05 0.01" condim="4"
            contype="1" conaffinity="1"/>
    </default>
    <!-- (warehouse) object class: ordinary cardboard-like grip -->
    <default class="object">
      <geom friction="0.6 0.01 0.001" condim="3" contype="1" conaffinity="1"/>
    </default>
  </default>

  <worldbody>
    <geom name="floor" type="plane" size="0.5 0.5 0.05" friction="0.5 0.005 0.001"/>

    <!-- The hand: one slide joint straight up (the lift). Its palm never collides. -->
    <body name="hand" pos="0 0 0.12">
      <joint name="lift" type="slide" axis="0 0 1" range="-0.1 0.2" damping="100"/>
      <geom name="palm" type="box" size="0.02 0.06 0.012" mass="0.5"
            contype="0" conaffinity="0" rgba="0.35 0.38 0.42 1"/>

      <!-- (warehouse) left finger: slides along -y to open, 0 = closed, 0.05 = wide open -->
      <body name="finger_L" pos="0 -0.01 -0.07" childclass="finger">
        <joint name="grip_L" type="slide" axis="0 -1 0" range="0 0.05" damping="10"/>
        <geom name="finger_L_geom" type="box" size="0.008 0.005 0.035" mass="0.05"/>
        <site name="touch_L_site" type="box" pos="0 0.005 0" size="0.009 0.003 0.036"/>
      </body>
      <!-- (warehouse) right finger: slides along +y to open -->
      <body name="finger_R" pos="0 0.01 -0.07" childclass="finger">
        <joint name="grip_R" type="slide" axis="0 1 0" range="0 0.05" damping="10"/>
        <geom name="finger_R_geom" type="box" size="0.008 0.005 0.035" mass="0.05"/>
        <site name="touch_R_site" type="box" pos="0 -0.005 0" size="0.009 0.003 0.036"/>
      </body>
    </body>

    <!-- (warehouse) the red box: 7 cm cube, 0.25 kg -->
    <body name="box" pos="0 0 0.035">
      <freejoint name="box_free"/>
      <geom name="box_geom" class="object" type="box" size="0.035 0.035 0.035"
            mass="0.25" rgba="0.85 0.15 0.1 1"/>
    </body>
  </worldbody>

  <!-- (warehouse) tendon length = grip_L - grip_R ... -->
  <tendon>
    <fixed name="gripper_couple">
      <joint joint="grip_L" coef="1"/>
      <joint joint="grip_R" coef="-1"/>
    </fixed>
  </tendon>
  <!-- (warehouse) ... and the equality holds that length at 0, so grip_R follows grip_L -->
  <equality>
    @@EQUALITY@@
  </equality>

  <actuator>
    <position name="lift_pos" joint="lift" kp="3000" ctrlrange="-0.1 0.2"/>
    <!-- (warehouse) one gripper actuator, on grip_L only -->
    <position name="gripper_pos" joint="grip_L" kp="600" ctrlrange="0 0.05"/>
  </actuator>

  <sensor>
    <touch name="touch_L" site="touch_L_site"/>
    <touch name="touch_R" site="touch_R_site"/>
  </sensor>

  <keyframe>
    <!-- lift, grip_L, grip_R, box (x y z, quaternion w x y z); fingers open 4 cm -->
    <key name="open" qpos="0  0.04 0.04  0 0 0.035 1 0 0 0" ctrl="0 0.04"/>
  </keyframe>
</mujoco>
"""

EQ_TENDON = '<tendon tendon1="gripper_couple" polycoef="0 0 0 0 0"/>'
EQ_JOINT = '<joint joint1="grip_R" joint2="grip_L" polycoef="0 1 0 0 0"/>'


def build(equality=EQ_TENDON):
    xml = GRIPPER_XML.replace("@@EQUALITY@@", equality)
    if not equality:
        xml = xml.replace("<equality>\n    \n  </equality>", "")
    model = mujoco.MjModel.from_xml_string(xml)
    return model, mujoco.MjData(model)


class Rig:
    """The gripper model plus the ids we need, and helpers to read contacts."""

    def __init__(self, equality=EQ_TENDON):
        self.model, self.data = build(equality)
        m = self.model
        gid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
        bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
        self.FL, self.FR, self.BOX, self.FLOOR = gid("finger_L_geom"), gid("finger_R_geom"), gid("box_geom"), gid("floor")
        self.box, self.hand = bid("box"), bid("hand")
        self.mass0 = float(m.body_mass[self.box])
        self.inertia0 = m.body_inertia[self.box].copy()

    def set_box_mass(self, mass):
        """Change the box mass in the MODEL (a design change), scaling its inertia with it."""
        self.model.body_mass[self.box] = mass
        self.model.body_inertia[self.box] = self.inertia0 * mass / self.mass0

    def set_finger(self, mu=2.0, condim=4):
        for g in (self.FL, self.FR):
            self.model.geom_friction[g, 0] = mu
            self.model.geom_condim[g] = condim

    def reset(self):
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)

    def name(self, g):
        return self.model.geom(g).name

    def pairs(self):
        """Sum mj_contactForce per geom pair: normal N, world-frame force on geom2, count, mu, condim."""
        m, d, f6, out = self.model, self.data, np.zeros(6), {}
        for i in range(d.ncon):
            c = d.contact[i]
            mujoco.mj_contactForce(m, d, i, f6)
            world = c.frame.reshape(3, 3).T @ f6[:3]      # contact frame -> world frame
            key = (self.name(c.geom1), self.name(c.geom2))
            e = out.setdefault(key, {"N": 0.0, "F": np.zeros(3), "n": 0, "mu": c.friction[0], "dim": c.dim})
            e["N"] += f6[0]; e["F"] += world; e["n"] += 1
        return out

    def finger_box(self, side):
        f = self.FL if side == "L" else self.FR
        for (a, b), e in self.pairs().items():
            if {a, b} == {self.name(f), "box_geom"}:
                return e
        return None


def ctrl_for(n_set):
    """Gripper target that would give n_set newtons per finger if the faces stopped at Q_TOUCH.
    The actuator pushes kp (q - ctrl) on grip_L; the equality hands half of it to grip_R."""
    return max(0.0, Q_TOUCH - 2.0 * n_set / KP_GRIP)


def lift_target(t):
    """Smooth lift: 0 -> LIFT metres over T_LIFT seconds (half a cosine), then hold."""
    s = min(max(t / T_LIFT, 0.0), 1.0)
    return LIFT * 0.5 * (1.0 - np.cos(np.pi * s))


def grasp_and_lift(rig, n_set, mass=0.25, mu=2.0, impratio=10.0, condim=4, record=None):
    """Close for T_CLOSE s, lift 10 cm in T_LIFT s, hold T_HOLD s. Returns a dict of measurements."""
    m, d = rig.model, rig.data
    m.opt.impratio = impratio
    rig.set_box_mass(mass); rig.set_finger(mu, condim)
    rig.reset()
    d.ctrl[1] = ctrl_for(n_set)
    zh_start = d.xpos[rig.hand][2]
    for _ in range(round(T_CLOSE / m.opt.timestep)):
        mujoco.mj_step(m, d)
    out = {"ctrl": d.ctrl[1], "settle": d.xpos[rig.hand][2] - zh_start, "pre_touch": d.sensordata.copy(), "pre_q": d.qpos[1:3].copy(),
           "pre_act": d.actuator_force[1], "pre_pairs": rig.pairs()}
    zb0, zh0 = d.xpos[rig.box][2], d.xpos[rig.hand][2]
    n = round((T_LIFT + T_HOLD) / m.opt.timestep)
    touch_hold = []
    for k in range(1, n + 1):
        d.ctrl[0] = lift_target(k * m.opt.timestep)
        mujoco.mj_step(m, d)
        if k * m.opt.timestep > T_LIFT + T_HOLD - 0.25:
            touch_hold.append(d.sensordata.copy())
        if record is not None and k % 50 == 0:                     # every 0.1 s
            record.append((d.xpos[rig.hand][2] - zh0) - (d.xpos[rig.box][2] - zb0))
    out["slip"] = (d.xpos[rig.hand][2] - zh0) - (d.xpos[rig.box][2] - zb0)
    out["hand_rise"] = d.xpos[rig.hand][2] - zh0
    out["touch"] = np.mean(touch_hold, axis=0)
    out["q"] = d.qpos[1:3].copy()
    out["act"] = d.actuator_force[1]
    out["pairs"] = rig.pairs()
    out["held"] = out["slip"] * 1000 < HELD_MM
    return out


def rule_holds(mu, n, mass):
    return 2 * mu * n >= mass * G


def part_a():
    print("=" * 78)
    print("A. The model: XML elements and the arrays they become")
    print("=" * 78)
    rig = Rig(); m = rig.model
    print(f"nq={m.nq}  nv={m.nv}  nu={m.nu}  ntendon={m.ntendon}  neq={m.neq}  nsensor={m.nsensor}")
    for j in range(m.njnt):
        print(f"  joint {j}: {m.joint(j).name:9s} type={['free','ball','slide','hinge'][m.jnt_type[j]]:5s} "
              f"qposadr={m.jnt_qposadr[j]}  range={m.jnt_range[j]}")
    print(f"  tendon 0: {m.tendon(0).name}  joints {list(m.wrap_objid[:2])} (grip_L, grip_R)  coef {list(m.wrap_prm[:2])}")
    print(f"  equality 0: type={mujoco.mjtEq(m.eq_type[0]).name}  obj1id={m.eq_obj1id[0]}  data[:5]={m.eq_data[0,:5]}")
    for a in range(m.nu):
        print(f"  actuator {a}: {m.actuator(a).name:11s} joint={m.joint(m.actuator_trnid[a,0]).name:6s} "
              f"gainprm[0]={m.actuator_gainprm[a,0]:.0f}  biasprm[0:3]={m.actuator_biasprm[a,:3]}  "
              f"ctrlrange={m.actuator_ctrlrange[a]}")
    for s in range(m.nsensor):
        print(f"  sensor {s}: {m.sensor(s).name}  adr={m.sensor_adr[s]}  dim={m.sensor_dim[s]}  site={m.site(m.sensor_objid[s]).name}")
    for g in (rig.FL, rig.BOX, rig.FLOOR):
        print(f"  geom {g}: {rig.name(g):13s} friction={m.geom_friction[g]}  condim={m.geom_condim[g]}")
    print(f"  box mass = {m.body_mass[rig.box]:.3f} kg   weight m g = {m.body_mass[rig.box]*G:.4f} N")
    print(f"  opt: integrator={mujoco.mjtIntegrator(m.opt.integrator).name}  cone={mujoco.mjtCone(m.opt.cone).name}  "
          f"impratio={m.opt.impratio:.0f}  timestep={m.opt.timestep}")


def part_b():
    print()
    print("=" * 78)
    print("B. Coupling: close the empty gripper from 0.040 m to target 0.020 m (box moved away)")
    print("=" * 78)
    for label, eq in [("tendon + equality (warehouse)", EQ_TENDON),
                      ("joint equality", EQ_JOINT),
                      ("no coupling", "")]:
        rig = Rig(eq); m, d = rig.model, rig.data
        rig.reset()
        d.qpos[3] = 0.5                      # box x = 0.5 m: out of the way
        mujoco.mj_forward(m, d)
        d.ctrl[1] = 0.02
        mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)       # ten_length is computed at the START of a step: refresh it (lesson 3)
        q1, t1 = d.qpos[1:3].copy(), d.ten_length[0]
        for _ in range(249):
            mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)
        print(f"{label:30s} after 1 step: grip_L={q1[0]:.6f} grip_R={q1[1]:.6f} ten_length={t1:+.2e}   "
              f"after 0.5 s: grip_L={d.qpos[1]:.6f} grip_R={d.qpos[2]:.6f} m   "
              f"ten_length={d.ten_length[0]:+.2e} m")


def part_c():
    print()
    print("=" * 78)
    print("C. The squeeze: set 1.0 N per finger, box 0.25 kg, finger mu 2.0")
    print("=" * 78)
    rig = Rig()
    n_set = 1.0
    r = grasp_and_lift(rig, n_set)
    print(f"gripper ctrl (target opening) = {Q_TOUCH} - 2 x {n_set} / {KP_GRIP:.0f} = {r['ctrl']:.6f} m")
    print(f"hand settled {r['settle']*1000:+.2f} mm on its lift spring during the 0.5 s close")
    print(f"naive squeeze per finger = kp (0.030 - ctrl) / 2 = {KP_GRIP*(Q_TOUCH-r['ctrl'])/2:.4f} N")
    for when, q, act, touch, pairs in [("on the floor (t=0.5 s)", r["pre_q"], r["pre_act"], r["pre_touch"], r["pre_pairs"]),
                                       ("held in the air (t=2.0 s)", r["q"], r["act"], r["touch"], r["pairs"])]:
        print(f"-- {when}")
        print(f"   qpos grip_L={q[0]:.6f} grip_R={q[1]:.6f} m   equality violation grip_L-grip_R={q[0]-q[1]:+.2e} m")
        print(f"   actuator_force = kp (ctrl - grip_L) = {act:+.4f} N  -> half of it per finger {-act/2:.4f} N")
        note = "sensordata, mean of the last 0.25 s" if "air" in when else "sensordata"
        print(f"   touch_L={touch[0]:.4f} N  touch_R={touch[1]:.4f} N  ({note})")
        for (a, b), e in pairs.items():
            print(f"   pair {a:13s}<->{b:13s} contacts={e['n']}  normal sum={e['N']:.4f} N  "
                  f"vertical force={e['F'][2]:+.4f} N  mu={e['mu']:.1f} condim={e['dim']}")
    up = sum(abs(e["F"][2]) for (a, b), e in r["pairs"].items() if "box_geom" in (a, b))
    nL = r["touch"][0]
    print(f"friction carried by both fingers = {up:.4f} N   weight m g = {0.25*G:.4f} N")
    print(f"friction needed per finger = m g / 2 = {0.25*G/2:.4f} N   available = mu N = 2.0 x {nL:.4f} = {2*nL:.4f} N")
    print(f"lift: hand rose {r['hand_rise']*1000:.2f} mm, box slipped {r['slip']*1000:.2f} mm relative to the hand")

    print("-- which mu does MuJoCo use? (finger friction set to 0.3, box friction 0.6)")
    rig.set_finger(0.3, 4); rig.reset(); rig.data.ctrl[1] = 0.025
    for _ in range(250):
        mujoco.mj_step(rig.model, rig.data)
    for (a, b), e in rig.pairs().items():
        print(f"   pair {a:13s}<->{b:13s} contact.friction[0]={e['mu']:.1f}  contact.dim={e['dim']}")


def part_d():
    print()
    print("=" * 78)
    print(f"D. Hold or slip: close {T_CLOSE} s, lift {LIFT*100:.0f} cm in {T_LIFT} s, hold {T_HOLD} s")
    print(f"   rule: 2 mu N >= m g, with N = measured squeeze per finger. 'held' = slip < {HELD_MM:.0f} mm")
    print("=" * 78)
    rig = Rig()
    # Measured squeeze for each setting (touch sensor while a light 0.05 kg box is held with mu 2.0).
    sets = [0.5, 1.0, 2.0, 4.0, 8.0]
    meas, viol = {}, {}
    for s in sets + [0.70, 0.75, 0.80]:
        r = grasp_and_lift(rig, s, mass=0.05, mu=2.0)
        meas[s], viol[s] = float(np.mean(r["touch"])), r["q"][0] - r["q"][1]
    print("setting (N)  measured squeeze per finger (N, touch while held)  equality violation grip_L-grip_R (m)")
    for s in sets:
        print(f"  {s:4.1f}        {meas[s]:.4f}                                       {viol[s]:+.2e}")
    stiff = grasp_and_lift(Rig(EQ_TENDON.replace("/>", ' solref="0.005 1"/>')), 8.0, mass=0.05, mu=2.0)
    print(f"  8.0 with a stiffer equality (solref 0.005 1): squeeze {np.mean(stiff['touch']):.4f} N, "
          f"violation {stiff['q'][0]-stiff['q'][1]:+.2e} m")
    print(f"{'set N':>6} {'meas N':>7} {'mass kg':>8} {'mu':>4} {'2 mu N':>7} {'m g':>6} {'rule':>5} {'MuJoCo slip mm':>15} {'MuJoCo':>7}")
    agree = total = 0
    for mass in (0.25, 0.5, 1.0):
        for mu in (0.6, 2.0):
            for s in sets:
                r = grasp_and_lift(rig, s, mass, mu)
                rule = rule_holds(mu, meas[s], mass)
                total += 1; agree += rule == r["held"]
                print(f"{s:6.1f} {meas[s]:7.3f} {mass:8.2f} {mu:4.1f} {2*mu*meas[s]:7.3f} {mass*G:6.3f} "
                      f"{'hold' if rule else 'slip':>5} {r['slip']*1000:15.2f} {'held' if r['held'] else 'slipped':>7}")
    print(f"rule and MuJoCo agree in {agree} of {total} runs")

    print("-- near the edge: 0.25 kg box, mu 2.0. Static rule needs N >= m g / (2 mu) = "
          f"{0.25*G/4:.4f} N; with the lift's peak acceleration {LIFT/2*(np.pi/T_LIFT)**2:.3f} m/s^2: "
          f"{0.25*(G+LIFT/2*(np.pi/T_LIFT)**2)/4:.4f} N")
    for s in (0.70, 0.75, 0.80):
        r = grasp_and_lift(rig, s, 0.25, 2.0)
        print(f"   set {s:.2f} N: measured squeeze {meas[s]:.4f} N, slip {r['slip']*1000:7.2f} mm "
              f"({'held' if r['held'] else 'slipped'})")

    print("-- one variable at a time: set 1.0 N, 0.25 kg, mu 2.0 (slip relative to the hand, mm)")
    for imp, cd in [(10, 4), (1, 4), (10, 3)]:
        r = grasp_and_lift(rig, 1.0, 0.25, 2.0, impratio=imp, condim=cd)
        print(f"   impratio={imp:2d} finger condim={cd}: slip {r['slip']*1000:.2f} mm")
    for imp, cd in [(10, 4), (1, 4), (10, 3)]:
        r = grasp_and_lift(rig, 4.0, 1.0, 2.0, impratio=imp, condim=cd)
        print(f"   1.0 kg, set 4.0 N, impratio={imp:2d} condim={cd}: slip {r['slip']*1000:.2f} mm")


def part_e():
    print()
    print("=" * 78)
    print("E. Touch sensor vs contact pairs")
    print("=" * 78)
    rig = Rig(); m, d = rig.model, rig.data

    def report(label):
        print(f"-- {label}")
        print(f"   touch_L={d.sensordata[0]:.3f} N  touch_R={d.sensordata[1]:.3f} N")
        for (a, b), e in rig.pairs().items():
            print(f"   pair {a:13s}<->{b:13s} contacts={e['n']}  normal sum={e['N']:.3f} N")
        held = rig.finger_box("L") is not None and rig.finger_box("R") is not None
        print(f"   both fingers touch the box (from data.contact)? {held}")

    # 1. Fingers open around the box, hand lowered 3 cm: fingertips press the floor.
    rig.reset(); d.ctrl[:] = [-0.03, 0.04]
    for _ in range(500):
        mujoco.mj_step(m, d)
    report("fingers open (0.04 m) around the box, lift target 3 cm down: fingertips pressed on the floor")
    print(f"   lift qpos[0]={d.qpos[0]*1000:.2f} mm (the floor stops the hand)")
    # 2. A real grasp.
    rig.reset(); d.ctrl[1] = ctrl_for(4.0)
    for _ in range(250):
        mujoco.mj_step(m, d)
    for k in range(1, 751):
        d.ctrl[0] = lift_target(k * m.opt.timestep); mujoco.mj_step(m, d)
    report("closed on the box (set 4 N) and lifted")


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    part_a(); part_b(); part_c(); part_d(); part_e()
