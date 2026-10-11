"""Capstone 1: build a cart-pole from scratch and balance it.

Nothing here is new MuJoCo: every milestone uses skills from lessons 1-9.
  M1  build the model, predict the counts             (lessons 1, 2, 4, 6, 7)
  M2  let go: how fast does it fall, which way does the cart go   (lesson 3)
  M3  mass matrix and the equations of motion by hand  (lessons 3, 8)
  M4  controllers: lean-only PD, then full-state feedback (lessons 5, 8)
  M5  wrap it as a Gymnasium environment               (lesson 9)

Run (headless, no viewer needed):
    cd /Users/mohwit/Desktop/mujoco && .venv/bin/python lessons/10-capstone-cart-pole/10_capstone_cart_pole.py
"""

import re
import warnings

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

# Every number has a reason:
#   timestep 0.002 s, gravity -9.81      the course defaults (lesson 3)
#   rail half-length 1.2 m, cart travel  slider range -1..1 m: a 2 m table-top track; the rail is drawn a bit longer
#   cart at z = 0.5 m                    the rail sits at table height (only for the picture: it does not change the physics)
#   cart box 0.2 x 0.15 x 0.1 m, 1 kg    a small trolley; mass given directly so it is a round number
#   pole capsule 0.6 m, radius 0.02 m    a 4 cm thick rod, long enough to see, short enough to fall in about 1 s
#   pole density 1000 kg/m^3             MuJoCo's default (water), as for every capsule in this course -> 0.7875 kg
#   contype 0 / conaffinity 0            no contacts at all (lesson 7): cart and pole only move through their joints
#   motor gear 20, ctrlrange -1..1       at most 20 N on the cart; the balance tests below use under 12 N
XML = r"""
<mujoco model="cart_pole">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="rail" type="box" size="1.2 0.01 0.01" pos="0 0 0.5" contype="0" conaffinity="0"/>
    <body name="cart" pos="0 0 0.5">
      <joint name="slider" type="slide" axis="1 0 0" range="-1 1"/>
      <geom name="cart_box" type="box" size="0.1 0.075 0.05" mass="1" contype="0" conaffinity="0"/>
      <body name="pole" pos="0 0 0">
        <joint name="hinge" type="hinge" axis="0 1 0"/>
        <geom name="pole_rod" type="capsule" fromto="0 0 0 0 0 0.6" size="0.02" contype="0" conaffinity="0"/>
        <site name="tip" pos="0 0 0.6"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="push" joint="slider" gear="20" ctrlrange="-1 1"/>
  </actuator>
  <sensor>
    <jointpos name="cart_x" joint="slider"/>
    <jointpos name="pole_angle" joint="hinge"/>
    <jointvel name="cart_v" joint="slider"/>
    <jointvel name="pole_spin" joint="hinge"/>
    <framepos name="tip_pos" objtype="site" objname="tip"/>
  </sensor>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(XML)
data = mujoco.MjData(model)
H = model.opt.timestep
GEAR = model.actuator_gear[0, 0]

# Physical constants read back from MuJoCo (used by the hand equations, the LQR design and the web page).
mujoco.mj_forward(model, data)
_M = np.zeros((2, 2))
mujoco.mj_fullM(model, data, _M)              # at theta = 0
M_TOTAL = _M[0, 0]                            # cart + pole mass, kg
MP_L = _M[0, 1]                               # pole mass x CoM distance, kg m (times cos theta in general)
I_HINGE = _M[1, 1]                            # pole inertia about the hinge, kg m^2
MP = model.body_mass[2]
L_COM = model.body_ipos[2][2]
G = -model.opt.gravity[2]
MGL = MP * G * L_COM                          # gravity torque on a horizontal pole, N m


def state():
    """[x, theta, xdot, thetadot]: m, rad, m/s, rad/s (joint frames)."""
    return np.array([data.qpos[0], data.qpos[1], data.qvel[0], data.qvel[1]])


def hand_qacc(x, th, xd, thd, ctrl, tau_push=0.0):
    """The cart-pole equations of motion written by hand (milestone 3)."""
    c, s = np.cos(th), np.sin(th)
    M = np.array([[M_TOTAL, MP_L * c], [MP_L * c, I_HINGE]])
    rhs = np.array([GEAR * ctrl + MP_L * s * thd**2, MGL * s + tau_push])
    return np.linalg.solve(M, rhs)


def run(knobs, th0=0.1, T=6.0, push=None, stop_at_rail=False, x0=0.0):
    """Simulate with ctrl = clip(k_pos x + k_lean th + k_vel xd + k_spin thd, -1, 1), computed before each step.

    knobs = (k_pos, k_lean, k_vel, k_spin) in ctrl per (m, rad, m/s, rad/s).
    push  = (torque N m, duration s): data.qfrc_applied[1] on the hinge from t = 0.
    """
    k = np.array([knobs[0], knobs[1], knobs[2], knobs[3]])
    mujoco.mj_resetData(model, data)
    data.qpos[:] = [x0, th0]
    mujoco.mj_forward(model, data)
    n = int(round(T / H))
    rec = np.zeros((n + 1, 6))     # t, x, th, xd, thd, ctrl asked (unclipped)
    rec[0, :5] = [0, *state()]  # time 0
    rail = None
    for i in range(n):
        s = state()
        u = float(k @ s)
        rec[i, 5] = u
        data.ctrl[0] = np.clip(u, -1, 1)
        data.qfrc_applied[1] = push[0] if (push and data.time < push[1] - 1e-9) else 0.0
        mujoco.mj_step(model, data)
        rec[i + 1, :5] = [data.time, *state()]
        if rail is None and abs(data.qpos[0]) >= 1.0:
            rail = data.time
            if stop_at_rail:
                return rec[: i + 2], rail
    return rec, rail


def metrics(rec, band_x=0.01, band_th=0.01):
    t, x, th, u = rec[:, 0], rec[:, 1], rec[:, 2], rec[:-1, 5]
    out = np.where((np.abs(x) > band_x) | (np.abs(th) > band_th))[0]
    settle = 0.0 if len(out) == 0 else (None if out[-1] == len(t) - 1 else t[out[-1] + 1])
    return dict(settle=settle, xmax=x[np.argmax(np.abs(x))], thpeak=th[np.argmax(np.abs(th))],
                umax=np.max(np.abs(u)), xend=x[-1], thend=th[-1])


def lqr_gains(Q, R):
    """Linearise at upright, discretise exactly like MuJoCo's Euler step, iterate the discrete Riccati equation."""
    Kq = np.linalg.solve(_M, np.array([[0.0, 0.0], [0.0, MGL]]))   # qacc per unit (x, th) near upright
    Ku = np.linalg.solve(_M, np.array([GEAR, 0.0]))                 # qacc per unit ctrl
    A = np.eye(4)
    A[2:, :2] = H * Kq                       # v' = v + h (Kq q + Ku u)
    A[:2, :2] += H * H * Kq                  # q' = q + h v'
    A[:2, 2:] = H * np.eye(2)
    B = np.zeros((4, 1))
    B[2:, 0] = H * Ku
    B[:2, 0] = H * H * Ku
    P = Q.copy()
    for it in range(1, 100001):
        K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        Pn = Q + A.T @ P @ (A - B @ K)
        if np.max(np.abs(Pn - P)) < 1e-10 * np.max(np.abs(Pn)):
            return K[0], A, B, it, Kq, Ku
        P = Pn
    raise RuntimeError("Riccati iteration did not converge")


def f(v, d=4):
    return f"{v:.{d}f}"


# ------------------------------------------------------------------ gymnasium env (milestone 5)
class CartPoleBalanceEnv(gym.Env):
    """Keep the pole upright and the cart on the rail.

    observation (4,) float32: [x, theta, xdot, thetadot] from sensordata[0:4] (m, rad, m/s, rad/s)
    action      (1,) float32: ctrl in [-1, 1] (force on the cart = 20 * ctrl N), held for frame_skip = 10 ticks
    reward      +1 for every env step that ends with the pole up and the cart on the rail
    terminated  |theta| > 0.2 rad (the pole has fallen too far) or |x| > 0.9 m (about to hit the rail stop at 1 m)
    truncated   after max_steps env steps (500 x 0.02 s = 10 s)
    """

    metadata = {"render_modes": []}
    THETA_LIMIT = 0.2
    X_LIMIT = 0.9

    def __init__(self, frame_skip=10, max_steps=500):
        self.model = mujoco.MjModel.from_xml_string(XML)
        self.data = mujoco.MjData(self.model)
        self.frame_skip = frame_skip
        self.max_steps = max_steps
        lo, hi = self.model.actuator_ctrlrange[0]
        self.action_space = spaces.Box(low=lo, high=hi, shape=(1,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)
        self.steps = 0

    def _obs(self):
        return self.data.sensordata[:4].astype(np.float32)      # x, theta, xdot, thetadot

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self.np_random.uniform(-0.05, 0.05, size=2)
        self.data.qvel[:] = self.np_random.uniform(-0.05, 0.05, size=2)
        mujoco.mj_forward(self.model, self.data)               # fills sensordata for the new state
        self.steps = 0
        return self._obs(), {}

    def step(self, action):
        a = float(np.clip(action[0], -1.0, 1.0))
        self.data.ctrl[0] = a
        for _ in range(self.frame_skip):
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)               # refresh sensors (they lag one tick after mj_step)
        self.steps += 1
        x, th = self.data.qpos
        terminated = bool(abs(th) > self.THETA_LIMIT or abs(x) > self.X_LIMIT)
        reward = 0.0 if terminated else 1.0
        truncated = self.steps >= self.max_steps
        return self._obs(), reward, terminated, truncated, {"time": self.data.time}


def main():
    np.set_printoptions(suppress=True)
    print("== Milestone 1: build the model (lessons 1, 2, 4, 6, 7) ==")
    print(f"nbody={model.nbody}  njnt={model.njnt}  nq={model.nq}  nv={model.nv}  nu={model.nu}  "
          f"ngeom={model.ngeom}  nsite={model.nsite}  nsensor={model.nsensor}  nsensordata={model.nsensordata}")
    print("body tree:")
    for b in range(model.nbody):
        depth, p = 0, b
        while p != 0:
            p = model.body_parentid[p]
            depth += 1
        jn = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(model.njnt) if model.jnt_bodyid[j] == b]
        gn = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) for g in range(model.ngeom) if model.geom_bodyid[g] == b]
        print(f"  {'  ' * depth}body {b} '{model.body(b).name or 'world'}'  joints={jn}  geoms={gn}  mass={model.body_mass[b]:.4f} kg")
    types = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}
    for j in range(model.njnt):
        print(f"joint {j} '{model.joint(j).name}': type={types[model.jnt_type[j]]}  qposadr={model.jnt_qposadr[j]}  dofadr={model.jnt_dofadr[j]}"
              f"  axis={model.jnt_axis[j]}  limited={bool(model.jnt_limited[j])}  range={model.jnt_range[j]}")
    print("sensors (name, adr, dim):", [(model.sensor(i).name, int(model.sensor_adr[i]), int(model.sensor_dim[i])) for i in range(model.nsensor)])
    print(f"pole mass {MP:.4f} kg (capsule r 0.02 m, length 0.6 m, density 1000 kg/m^3), CoM {L_COM:.4f} m above the hinge,"
          f" inertia about its CoM (y) {model.body_inertia[2][1]:.6f} kg m^2")
    print(f"actuator 'push': gear={GEAR:.0f} N per unit ctrl, ctrlrange={model.actuator_ctrlrange[0]}, max force {GEAR:.0f} N")
    data.qpos[:] = [0.0, 0.1]
    mujoco.mj_forward(model, data)
    print(f"at x=0, theta=0.1: sensordata={np.round(data.sensordata, 5).tolist()}  (tip world pos = (0.6 sin 0.1, 0, 0.5 + 0.6 cos 0.1))")
    print(f"ncon (contacts) = {data.ncon}")

    print("\n== Milestone 2: let go, zero control (lesson 3) ==")
    for th0 in (0.05, 0.1):
        rec, _ = run((0, 0, 0, 0), th0=th0, T=3.0)
        i = np.argmax(np.abs(rec[:, 2]) > 0.5)
        print(f"start theta={th0} rad: passes 0.5 rad at t={rec[i, 0]:.3f} s; then cart x={rec[i, 1]:+.4f} m, xdot={rec[i, 3]:+.4f} m/s,"
              f" thetadot={rec[i, 4]:+.4f} rad/s")
    rec, _ = run((0, 0, 0, 0), th0=0.1, T=3.0)
    for t in (0.0, 0.25, 0.5, 0.75):
        r = rec[int(round(t / H))]
        print(f"   t={t:.2f} s  x={r[1]:+.5f} m  theta={r[2]:+.5f} rad")
    # centre of mass of the whole machine does not move sideways (no horizontal outside force)
    i = np.argmax(np.abs(rec[:, 2]) > 0.5)
    com = (1.0 * rec[i, 1] + MP * (rec[i, 1] + L_COM * np.sin(rec[i, 2]))) / M_TOTAL
    print(f"   whole-machine centre of mass x at that moment: {com:+.6f} m (it started at {MP * L_COM * np.sin(0.1) / M_TOTAL:+.6f} m)")
    for t in (0.5, 0.6):
        r = rec[int(round(t / H))]
        print(f"   centre of mass x at t={t} s: {(1.0 * r[1] + MP * (r[1] + L_COM * np.sin(r[2]))) / M_TOTAL:+.6f} m  (theta {r[2]:+.4f} rad, x {r[1]:+.4f} m)")

    print("\n== Milestone 3: mass matrix and equations of motion (lessons 3, 8) ==")
    for th in (0.0, 0.3):
        data.qpos[:] = [0.0, th]
        data.qvel[:] = 0
        mujoco.mj_forward(model, data)
        M = np.zeros((2, 2))
        mujoco.mj_fullM(model, data, M)
        print(f"mj_fullM at theta={th}: [[{M[0,0]:.6f}, {M[0,1]:.6f}], [{M[1,0]:.6f}, {M[1,1]:.6f}]]")
    print(f"M00 = m_cart + m_pole = 1 + {MP:.6f} = {M_TOTAL:.6f} kg")
    print(f"M01 = m_pole * l * cos(theta),  m_pole * l = {MP:.6f} * {L_COM:.1f} = {MP_L:.6f} kg m;  at 0.3: {MP_L * np.cos(0.3):.6f}")
    print(f"M11 = I_com + m_pole l^2 = {model.body_inertia[2][1]:.6f} + {MP * L_COM**2:.6f} = {I_HINGE:.6f} kg m^2")
    print(f"m_pole g l = {MGL:.6f} N m")
    tests = [(0.0, 0.1, 0.0, 0.0, 0.0), (0.2, 0.3, -0.5, 1.2, 0.4), (0.0, 0.0, 0.0, 0.0, 0.5)]
    for x, th, xd, thd, u in tests:
        mujoco.mj_resetData(model, data)
        data.qpos[:] = [x, th]
        data.qvel[:] = [xd, thd]
        data.ctrl[0] = u
        mujoco.mj_forward(model, data)
        mine = hand_qacc(x, th, xd, thd, u)
        c, sn = np.cos(th), np.sin(th)
        a, b, dd = M_TOTAL, MP_L * c, I_HINGE
        r0, r1 = GEAR * u + MP_L * sn * thd**2, MGL * sn
        det = a * dd - b * b
        print(f"   by hand: M=[[{a:.6f}, {b:.6f}], [{b:.6f}, {dd:.6f}]]  rhs=[{GEAR:.0f}*{u} + {MP_L:.6f}*sin({th})*{thd}^2, {MGL:.6f}*sin({th})]"
              f" = [{r0:.6f}, {r1:.6f}]  det={det:.6f}")
        print(f"   qacc[0] = ({r0:.6f}*{dd:.6f} - {b:.6f}*{r1:.6f}) / {det:.6f} = {(r0 * dd - b * r1) / det:+.6f}"
              f"   qacc[1] = ({a:.6f}*{r1:.6f} - {b:.6f}*{r0:.6f}) / {det:.6f} = {(a * r1 - b * r0) / det:+.6f}")
        print(f"state x={x} th={th} xd={xd} thd={thd} ctrl={u}: MuJoCo qacc=[{data.qacc[0]:+.6f}, {data.qacc[1]:+.6f}]"
              f"  hand=[{mine[0]:+.6f}, {mine[1]:+.6f}]  qfrc_bias=[{data.qfrc_bias[0]:+.6f}, {data.qfrc_bias[1]:+.6f}]"
              f"  max diff {np.max(np.abs(mine - data.qacc)):.1e}")

    print("\n== Milestone 4a: PD on the lean only (lesson 8) ==")
    Q, R = np.diag([1.0, 1.0, 0.0, 0.0]), np.array([[1.0]])
    K, A, B, iters, Kq, Ku = lqr_gains(Q, R)
    knobs = -K
    lean_only = (0.0, knobs[1], 0.0, knobs[3])
    rec, rail = run(lean_only, th0=0.1, T=10.0)
    i2 = int(round(2.0 / H))
    m4 = metrics(rec[:i2 + 1])
    print(f"knobs: lean {lean_only[1]:.4f}, lean speed {lean_only[3]:.4f}, cart offset 0, cart speed 0 (ctrl per rad, per rad/s)")
    for t in (0.5, 1.0, 2.0, 3.0, 4.0):
        r = rec[int(round(t / H))]
        print(f"   t={t:.1f} s  x={r[1]:+.4f} m  theta={r[2]:+.5f} rad  xdot={r[3]:+.4f} m/s")
    print(f"lean-only: pole upright within 0.01 rad from t={metrics(rec[:i2+1], band_x=1e9)['settle']:.3f} s, peak ctrl {m4['umax']:.4f};"
          f" cart reaches the 1 m rail stop at t={rail:.3f} s")
    rec_after = rec[int(round(rail / H)):]
    print(f"   after the rail stop: |theta| reaches {np.max(np.abs(rec_after[:, 2])):.3f} rad by t=10 s (pole knocked over)")

    print("\n== Milestone 4b: full-state feedback, gains from LQR (lesson 8 + one new idea) ==")
    print(f"linear model near upright: qacc = Kq [x, theta] + Ku ctrl, Kq={np.round(Kq, 6).tolist()}, Ku={np.round(Ku, 6).tolist()}")
    print(f"   uncontrolled lean grows like exp({np.sqrt(Kq[1, 1]):.4f} t): multiplied by e every {1 / np.sqrt(Kq[1, 1]):.3f} s")
    print(f"LQR cost per step: x^2 + theta^2 + ctrl^2 (Q=diag(1,1,0,0), R=1); Riccati iterations: {iters}")
    print(f"K = {np.round(K, 4).tolist()}  ->  ctrl = -K s")
    print(f"knobs (= -K): cart offset {knobs[0]:.4f} /m, lean {knobs[1]:.4f} /rad, cart speed {knobs[2]:.4f} /(m/s), lean speed {knobs[3]:.4f} /(rad/s)")
    print(f"   in newtons (x 20): {knobs[0]*GEAR:.3f} N/m, {knobs[1]*GEAR:.3f} N/rad, {knobs[2]*GEAR:.3f} N/(m/s), {knobs[3]*GEAR:.3f} N/(rad/s)")
    eig = np.abs(np.linalg.eigvals(A - B @ K.reshape(1, 4)))
    print(f"closed-loop step growth factors |eig(A - B K)|: max {eig.max():.6f} (< 1: every wobble shrinks)")
    print(f"   slowest wobble shrinks by half every {np.log(2) / -np.log(eig.max()) * H:.3f} s")
    # cross-check the hand linearisation with MuJoCo's own finite-difference linearisation
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    Afd, Bfd = np.zeros((4, 4)), np.zeros((4, 1))
    mujoco.mjd_transitionFD(model, data, 1e-6, True, Afd, Bfd, None, None)
    print(f"check vs mujoco.mjd_transitionFD: max |A - A_fd| = {np.max(np.abs(A - Afd)):.1e}, max |B - B_fd| = {np.max(np.abs(B - Bfd)):.1e}")
    rec, rail = run(tuple(knobs), th0=0.1, T=6.0)
    mb = metrics(rec)
    print(f"balance from 0.1 rad: settle (|x|<0.01 m and |theta|<0.01 rad for good) {mb['settle']:.3f} s,"
          f" max cart travel {mb['xmax']:+.4f} m, peak ctrl {mb['umax']:.4f} ({mb['umax']*GEAR:.2f} N), clipped: {mb['umax'] > 1},"
          f" end x={mb['xend']:+.2e} m theta={mb['thend']:+.2e} rad, lean range {rec[:, 2].min():+.4f} .. {rec[:, 2].max():+.4f} rad")
    first = rec[0]
    print(f"   step 0: s=[{first[1]}, {first[2]}, {first[3]}, {first[4]}] -> ctrl = {knobs[1]:.4f} x 0.1 = {first[5]:.5f} (force {first[5]*GEAR:.4f} N, towards +x)")
    for t in (0.1, 0.25, 0.5, 1.0, 2.0):
        r = rec[int(round(t / H))]
        print(f"   t={t:.2f} s  x={r[1]:+.4f} m  theta={r[2]:+.5f} rad  xdot={r[3]:+.4f} m/s  thetadot={r[4]:+.4f} rad/s  ctrl={r[5]:+.4f}")
    PUSH = (1.0, 0.1)
    rec, rail = run(tuple(knobs), th0=0.0, T=6.0, push=PUSH)
    mp_ = metrics(rec)
    for t in (0.05, 0.1, 0.2, 0.3, 0.5, 1.0):
        r = rec[int(round(t / H))]
        print(f"   push run t={t:.2f} s  x={r[1]:+.4f} m  theta={r[2]:+.5f} rad  xdot={r[3]:+.4f} m/s  thetadot={r[4]:+.4f} rad/s  ctrl={r[5]:+.4f}")
    print(f"push: data.qfrc_applied[1] = +{PUSH[0]} N m on the hinge for {PUSH[1]} s (a flick that tips the pole towards +x)")
    print(f"   lean range {rec[:,2].min():+.4f} .. {rec[:,2].max():+.4f} rad, max cart travel {mp_['xmax']:+.4f} m, peak ctrl {mp_['umax']:.4f} ({mp_['umax']*GEAR:.2f} N),"
          f" settle {mp_['settle']:.3f} s, rail hit: {rail}")
    rec, _ = run(tuple(knobs), th0=0.2, T=6.0)
    m2 = metrics(rec)
    print(f"harder start 0.2 rad: peak ctrl {m2['umax']:.4f} ({'clipped' if m2['umax'] > 1 else 'not clipped'}), max travel {m2['xmax']:+.4f} m, settle {m2['settle']:.3f} s, lean range {rec[:, 2].min():+.4f} .. {rec[:, 2].max():+.4f} rad")
    rec, _ = run(tuple(knobs), th0=0.0, T=6.0, x0=0.5)
    i = np.argmax(rec[:, 1])
    print(f"start x=+0.5 m, upright: step-0 ctrl {rec[0, 5]:+.5f}; cart first moves AWAY to x={rec[i, 1]:+.4f} m at t={rec[i, 0]:.3f} s"
          f" (lean then {rec[i, 2]:+.4f} rad), lean min {rec[:, 2].min():+.4f} rad; x at 3 s {rec[int(3 / H), 1]:+.4f} m, settle {metrics(rec)['settle']:.3f} s")
    print(f"sign check: with all four knobs negated the pole falls: |theta| at 1 s = "
          f"{abs(run(tuple(-knobs), th0=0.1, T=1.0)[0][-1, 2]):.3f} rad")

    print("\n== Milestone 5: a Gymnasium environment (lesson 9) ==")
    env = CartPoleBalanceEnv()
    print(f"frame_skip {env.frame_skip} -> env step {H * env.frame_skip:.3f} s; max_steps {env.max_steps} -> {H * env.frame_skip * env.max_steps:.1f} s")
    print(f"observation_space {env.observation_space}; action_space {env.action_space}")
    from gymnasium.utils.env_checker import check_env
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        check_env(CartPoleBalanceEnv(), skip_render_check=True)
    print(f"check_env: passed, {len(w)} warning(s)")
    for x in w:
        print("   warning:", re.sub(r"\x1b\[[0-9;]*m", "", str(x.message)).splitlines()[0][:110])
    kn = np.array(knobs)
    lo = np.array(lean_only)
    policies = {
        "zero": lambda o, sp: np.array([0.0], np.float32),
        "random": lambda o, sp: sp.sample(),
        "lean-only": lambda o, sp: np.array([np.clip(lo @ o, -1, 1)], np.float32),
        "full-state": lambda o, sp: np.array([np.clip(kn @ o, -1, 1)], np.float32),
    }
    print("seed   start [x, th, xd, thd]                 " + "".join(f"{k:>11}" for k in policies) + "   (return = steps survived, max 500)")
    tot = {k: [] for k in policies}
    why = {}
    for seed in range(5):
        row = []
        for name, pol in policies.items():
            obs, _ = env.reset(seed=seed)
            env.action_space.seed(seed)
            ret = 0.0
            while True:
                obs, r, term, trunc, info = env.step(pol(obs, env.action_space))
                ret += r
                if term or trunc:
                    break
            if term:
                why.setdefault(name, []).append(("theta" if abs(env.data.qpos[1]) > env.THETA_LIMIT else "rail") + f"@{info['time']:.2f}s")
            row.append(ret)
            tot[name].append(ret)
        o0, _ = env.reset(seed=seed)
        print(f"{seed:>4}   {np.array2string(o0, precision=4, floatmode='fixed', sign='+')}  " + "".join(f"{v:>11.0f}" for v in row))
    print("mean" + " " * 43 + "".join(f"{np.mean(tot[k]):>11.1f}" for k in policies))
    for k, v in why.items():
        print(f"   {k} ended by: {', '.join(v)}")


if __name__ == "__main__":
    main()
