"""Lesson 08: write a PD controller for the course pendulum, tune it by experiment,
see why a big timestep breaks it, and log a run to CSV so anyone can repeat it.

Run (headless, no viewer, no matplotlib needed):
    uv run --python .venv/bin/python lessons/08-controllers-and-experiments/08_controllers_and_experiments.py

Every experiment starts with the pendulum hanging straight down at rest (qpos = 0, qvel = 0)
and asks it to move to a target angle of 1.0 rad, then runs for 3 s.
Angles are hinge angles in rad, measured from hanging straight down, positive about +y (joint frame).
"""

import pathlib

import mujoco
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent

# The course pendulum from lessons 3, 5 and 6: the lesson 5 motor (gear 100, ctrl in [-1, 1])
# plus two sensors.
XML = r"""
<mujoco model="pd_pendulum">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0 0 -1" size="0.08"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor" joint="hinge" gear="100" ctrlrange="-1 1" ctrllimited="true"/>
  </actuator>
  <sensor>
    <jointpos name="angle" joint="hinge"/>
    <jointvel name="spin" joint="hinge"/>
  </sensor>
</mujoco>
"""

TARGET = 1.0      # rad, the step we ask for
T_END = 3.0       # s, length of every experiment run
BAND = 0.02       # settling band: 2 % of the 1.0 rad step, in rad

model = mujoco.MjModel.from_xml_string(XML)
data = mujoco.MjData(model)

# ---------- physical constants read from the model (never typed by hand) ----------
mujoco.mj_forward(model, data)
M = np.zeros((model.nv, model.nv))
mujoco.mj_fullM(model, data, M)          # joint-space inertia (mass matrix), kg m^2
I = M[0, 0]                                  # turning inertia about the hinge
mass = model.body_mass[1]                    # kg
d = -model.body_ipos[1][2]                   # m, hinge to centre of mass (ipos z = -0.5)
g = -model.opt.gravity[2]                    # m/s^2
MGD = mass * g * d                           # N m, gravity torque when the rod is horizontal
GEAR = model.actuator_gear[0][0]             # N m of torque per unit of ctrl
U_MAX = GEAR * model.actuator_ctrlrange[0][1] # N m, the strongest torque the motor can give


def q_balance(kp, target, gravity_comp):
    """Angle where the motor's pull kp*(target - q) exactly balances gravity MGD*sin(q)."""
    if gravity_comp:
        return target
    lo, hi = 0.0, target                     # f(lo) > 0, f(hi) < 0 for 0 < target < pi
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if kp * (target - mid) - MGD * np.sin(mid) > 0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def pd_torque(q, qvel, kp, kd, target, gravity_comp):
    """The whole controller, in N m: spring towards the target, brake on speed, optional gravity help."""
    tau = kp * (target - q) - kd * qvel
    if gravity_comp:
        tau += MGD * np.sin(q)
    return tau


def tau_to_ctrl(tau, limited=True):
    """The motor takes ctrl in [-1, 1]; torque = gear * ctrl, so divide by the gear and clip."""
    return float(np.clip(tau / GEAR, -1.0, 1.0)) if limited else tau / GEAR


def run(kp, kd, h=0.002, t_end=T_END, target=TARGET, gravity_comp=False, log=None, limited=True):
    """Step response in real MuJoCo. Returns arrays t, q, qvel, tau (asked, N m), torque (applied, N m)."""
    model.opt.timestep = h
    model.actuator_ctrllimited[0] = limited   # False only for the "no motor limit" comparison
    mujoco.mj_resetData(model, data)          # qpos = 0, qvel = 0, time = 0
    n = int(round(t_end / h))
    t, q, v, u, f = [0.0], [0.0], [0.0], [], []
    for _ in range(n):
        # 1. read state   2. compute tau, then ctrl   3. step
        tau = pd_torque(data.qpos[0], data.qvel[0], kp, kd, target, gravity_comp)
        ctrl = tau_to_ctrl(tau, limited)
        if log is not None:
            log.append((data.time, data.qpos[0], data.qvel[0], tau, ctrl, target))
        data.ctrl[0] = ctrl
        mujoco.mj_step(model, data)
        u.append(tau)
        f.append(data.qfrc_actuator[0])       # joint torque the motor really applied, N m
        t.append(data.time); q.append(data.qpos[0]); v.append(data.qvel[0])
        if not np.isfinite(data.qpos[0]) or abs(data.qpos[0]) > 100:
            break                              # exploded: stop before MuJoCo's own blow-up guard
    model.opt.timestep = 0.002
    model.actuator_ctrllimited[0] = True
    return tuple(np.array(a) for a in (t, q, v, u, f))


def metrics(t, q, f, kp, target=TARGET, gravity_comp=False):
    """Rise time, overshoot, settling time, steady-state error, peak torque."""
    qs = q_balance(kp, target, gravity_comp)
    above = np.nonzero(q >= 0.9 * qs)[0]
    rise = t[above[0]] if len(above) else None
    overshoot = max(0.0, (q.max() - qs) / qs * 100.0)
    out = np.nonzero(np.abs(q - qs) > BAND * target)[0]
    if len(out) == 0:
        settle = 0.0
    elif out[-1] == len(q) - 1:
        settle = None                          # still outside the band at the end
    else:
        settle = t[out[-1] + 1]
    return dict(q_star=qs, rise=rise, overshoot=overshoot, settle=settle,
                ss_err=target - q[-1], peak=np.abs(f).max())


def z(x):
    """Print a tiny negative number like -0.0000001 as 0, not -0.000000."""
    return 0.0 if abs(x) < 5e-7 else x


def s(x, fmt="{:.3f}", none="not settled"):
    return none if x is None else fmt.format(x)


def main() -> None:
    print("=" * 78)
    print("MODEL FACTS (read from MjModel / MjData, SI units)")
    print("=" * 78)
    print(f"mujoco version                         {mujoco.__version__}")
    print(f"timestep model.opt.timestep            {model.opt.timestep} s")
    print(f"body_mass[1] pendulum mass m           {mass:.4f} kg")
    print(f"body_ipos[1] centre of mass (body)     {model.body_ipos[1]} m  -> d = {d:.3f} m")
    print(f"turning inertia I  (mj_fullM, nv x nv) {I:.4f} kg m^2   (I = {I!r})")
    print(f"m g d  (gravity torque at 90 deg)      {MGD:.4f} N m    (m g d = {MGD!r})")
    print(f"m g d / I                              {MGD / I:.5f} 1/s^2")
    hold = MGD * np.sin(TARGET)
    print(f"actuator 0 'motor': nu = {model.nu}, gear = {GEAR:.0f} N m per unit ctrl, "
          f"ctrlrange = {model.actuator_ctrlrange[0]}, ctrllimited = {bool(model.actuator_ctrllimited[0])}")
    print(f"  strongest torque = gear x 1 = {U_MAX:.0f} N m")
    print(f"torque to HOLD 1.0 rad = m g d sin(1)  {hold:.3f} N m  -> ctrl = {hold / GEAR:.5f} "
          f"({hold / U_MAX * 100:.0f} % of the limit, {U_MAX - hold:.3f} N m to spare)")
    print(f"  highest angle the motor can hold: asin({U_MAX:.0f} / {MGD:.2f}) = {np.arcsin(U_MAX / MGD):.4f} rad")

    print()
    print("=" * 78)
    print("THE CONTROL LOOP, first 3 turns (kp = 400 N m/rad, kd = 40 N m s/rad)")
    print("=" * 78)
    model.opt.timestep = 0.002
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    for k in range(3):
        q, v = data.qpos[0], data.qvel[0]
        tau = pd_torque(q, v, 400, 40, TARGET, False)
        data.ctrl[0] = tau_to_ctrl(tau)
        print(f"turn {k}: read time={data.time:.3f} s qpos[0]={q:.6f} rad qvel[0]={v:.6f} rad/s"
              f" -> tau={tau:.4f} N m -> ctrl[0]=clip(tau/100)={data.ctrl[0]:.4f}")
        mujoco.mj_step(model, data)
        print(f"        after mj_step: qpos[0]={data.qpos[0]:.6f} rad qvel[0]={data.qvel[0]:.6f} rad/s"
              f" | sensordata angle={data.sensordata[0]:.6f} spin={data.sensordata[1]:.6f}"
              f" | actuator_force[0]={data.actuator_force[0]:.4f} qfrc_actuator[0]={data.qfrc_actuator[0]:.4f} N m")
    print("  note: after mj_step, sensordata still holds the state from the START of that step")
    print("        (sensors are computed before integration); qpos/qvel hold the new state.")
    mujoco.mj_resetData(model, data)
    data.ctrl[0] = 0.5
    mujoco.mj_step(model, data)
    print(f"  in range:  data.ctrl[0] = 0.5 -> actuator_force[0] = {data.actuator_force[0]:.1f}, "
          f"qfrc_actuator[0] = {data.qfrc_actuator[0]:.1f} N m")
    data.ctrl[0] = 8.0
    mujoco.mj_step(model, data)
    print(f"  clipping:  data.ctrl[0] = 8.0 (tau 800 / 100, unclipped) -> after mj_step data.ctrl[0] = {data.ctrl[0]:.1f}, "
          f"actuator_force[0] = {data.actuator_force[0]:.1f}, qfrc_actuator[0] = {data.qfrc_actuator[0]:.1f} N m")

    print()
    print("=" * 78)
    print("EXPERIMENT 1: P only (kd = 0). Step 0 -> 1.0 rad, 3 s")
    print("=" * 78)
    print(" kp     balance q*  lowest q   highest q  swing 2-3 s  peak tau asked  peak torque  settled?")
    for kp in (100, 400, 1600):
        t, q, v, u, f = run(kp, 0)
        m = metrics(t, q, f, kp)
        late = q[t >= 2.0]
        print(f"{kp:5d}  {m['q_star']:9.4f}  {z(q[1:].min()):8.4f}  {q.max():9.4f}  "
              f"{late.max() - late.min():10.4f}   {np.abs(u).max():12.1f}  {m['peak']:10.1f}   "
              f"{'yes' if m['settle'] is not None else 'no'}")
    print(f"  (rad, rad, rad, rad peak-to-peak, N m, N m) tau above {U_MAX:.0f} N m is clipped: ctrl stops at 1")

    print()
    print("=" * 78)
    print("EXPERIMENT 2: fixed kp = 400 N m/rad, sweep kd. Step 0 -> 1.0 rad, 3 s, no gravity help")
    print("=" * 78)
    kp = 400
    kd_crit = 2 * np.sqrt(kp * I)
    print(f"natural frequency sqrt(kp/I) = {np.sqrt(kp / I):.4f} rad/s (one wobble every 2 pi/omega = "
          f"{2 * np.pi / np.sqrt(kp / I):.3f} s), critical kd = 2 sqrt(kp I) = {kd_crit:.2f}")
    print("  kd  damping ratio  rise (s)  overshoot (%)  settle (s)  ss error (rad)  time at motor limit (s)")
    for kd in (10, 40, 112, 400):
        t, q, v, u, f = run(kp, kd)
        m = metrics(t, q, f, kp)
        sat = np.sum(np.abs(u) >= U_MAX) * 0.002
        print(f"{kd:4d}  {kd / kd_crit:12.3f}  {s(m['rise'])+'   ':>9} {m['overshoot']:12.1f}  "
              f"{s(m['settle']):>10}  {m['ss_err']:13.4f}  {sat:14.3f}")
    print(f"  q* (balance angle with gravity) = {metrics(t, q, f, kp)['q_star']:.4f} rad; settle band = "
          f"+-{BAND * TARGET:.2f} rad around q*")

    print()
    print("=" * 78)
    print("EXPERIMENT 3: steady-state error from gravity. kd = 112 fixed, sweep kp")
    print("=" * 78)
    print("  kp   guess m g d sin(1)/kp  balance-equation error  measured at 3 s  with gravity comp")
    for kp in (200, 400, 800):
        t, q, v, u, f = run(kp, 112)
        tg, qg, vg, ug, fg = run(kp, 112, gravity_comp=True)
        print(f"{kp:5d}  {hold / kp:20.4f}  {TARGET - q_balance(kp, TARGET, False):21.4f}  "
              f"{TARGET - q[-1]:15.4f}  {z(TARGET - qg[-1]):16.6f}")
    print("  (all errors in rad = target - angle). Gravity comp adds m g d sin(q) to tau.")
    for gc in (False, True):
        t, q, v, u, f = run(400, 112, gravity_comp=gc)
        m = metrics(t, q, f, 400, gravity_comp=gc)
        print(f"  kp=400 kd=112 gravity comp {'on ' if gc else 'off'}: rise {s(m['rise'])} s, overshoot {m['overshoot']:.1f} %, "
              f"settle {s(m['settle'])} s, ss error {z(m['ss_err']):.6f} rad, peak torque {m['peak']:.1f} N m, "
              f"final torque {f[-1]:.3f} N m")

    print()
    print("=" * 78)
    print("EXPERIMENT 4: timestep vs stability. kp = 400, kd = 400, gravity comp ON, 10 s runs")
    print("=" * 78)
    kp, kd = 400, 400

    def growth(h):
        # one semi-implicit Euler step of I*qacc = -kp*e - kd*v as a 2x2 matrix on (e, v)
        A = np.array([[1 - h * h * kp / I, h * (1 - h * kd / I)],
                      [-h * kp / I, 1 - h * kd / I]])
        return np.abs(np.linalg.eigvals(A)).max()

    a, b, c = kp / I, 2 * kd / I, -4.0
    h_formula = (-b + np.sqrt(b * b - 4 * a * c)) / (2 * a)
    def buzz(t, f, t_end):
        late = f[t[1:] > t_end - 2.0 + 1e-9]       # applied torque during the last 2 s
        return late.max() - late.min()

    print("  verdict: 'buzzing' if the applied torque still swings by more than 1 N m in the last 2 s")
    print("  h (s)   steps  kd h/I   kp h^2/I  test<4  growth/step  torque swing 8-10 s (N m)  angle at 10 s"
          "  verdict    | no motor limit (ctrllimited off)")
    for h in (0.002, 0.01, 0.02, 0.03, 0.035, 0.038, 0.039, 0.04, 0.05):
        t, q, v, u, f = run(kp, kd, h=h, t_end=10.0, gravity_comp=True)
        tn, qn, vn, un, fn = run(kp, kd, h=h, t_end=10.0, gravity_comp=True, limited=False)
        sw = buzz(t, f, 10.0)
        test = kp * h * h / I + 2 * kd * h / I
        off = np.nonzero(np.abs(qn - TARGET) > 1.0)[0]
        free = "stays at target" if len(off) == 0 else f"wobble passes 1 rad at t = {tn[off[0]]:.2f} s"
        print(f"{h:6.3f}  {len(t) - 1:5d}  {kd * h / I:7.4f}  {kp * h * h / I:8.5f}  {test:6.3f}  "
              f"{growth(h):11.4f}  {sw:24.2f}  {q[-1]:13.4f}  {'stable ' if sw < 1.0 else 'BUZZING'}    | {free}")
    lo, hi = 0.002, 0.08
    for _ in range(30):
        mid = 0.5 * (lo + hi)
        t, q, v, u, f = run(kp, kd, h=mid, t_end=30.0, gravity_comp=True)
        if buzz(t, f, 30.0) < 1.0:
            lo = mid
        else:
            hi = mid
    print(f"  formula threshold: kp h^2/I + 2 kd h/I = 4  ->  h = {h_formula:.5f} s")
    print(f"  MuJoCo threshold (bisection, 30 s runs)    ->  h = {0.5 * (lo + hi):.5f} s")
    for kd2 in (112, 40):
        b2 = 2 * kd2 / I
        print(f"  same formula for kd = {kd2}: h = {(-b2 + np.sqrt(b2 * b2 + 16 * a)) / (2 * a):.4f} s")

    print()
    print("=" * 78)
    print("EXPERIMENT 5: reproducible log -> 08_pd_log.csv (kp = 400, kd = 40, no gravity comp)")
    print("=" * 78)
    rows = []
    t, q, v, u, f = run(400, 40, log=rows)
    tau = pd_torque(q[-1], v[-1], 400, 40, TARGET, False)
    rows.append((t[-1], q[-1], v[-1], tau, tau_to_ctrl(tau), TARGET))
    rows = np.array(rows)[:, [0, 1, 2, 4, 5]]           # time, qpos, qvel, ctrl, target
    path = HERE / "08_pd_log.csv"
    header = (f"lesson 08 PD step response, mujoco {mujoco.__version__}\n"
              f"model: course pendulum + motor 'motor' gear 100 ctrlrange -1 1 (torque = 100 * ctrl N m)\n"
              f"timestep 0.002 s, integrator Euler, gravity -9.81, start qpos 0 qvel 0\n"
              f"kp 400 N m/rad, kd 40 N m s/rad, target 1.0 rad, gravity_comp no, duration 3.0 s\n"
              f"ctrl = clip((kp*(target-qpos) - kd*qvel) / 100, -1, 1), written before each mj_step\n"
              f"time_s,qpos_rad,qvel_rad_s,ctrl,target_rad")
    np.savetxt(path, rows, delimiter=",", fmt="%.6f", header=header, comments="# ")
    back = np.loadtxt(path, delimiter=",")
    print(f"wrote {path.name}: {back.shape[0]} rows x {back.shape[1]} columns (+ 6 header lines starting with #)")
    for r in back[[0, 1, 2, 250, 500, 1500]]:
        print(f"  time={r[0]:.3f} s  qpos={r[1]:.6f} rad  qvel={r[2]:.6f} rad/s  ctrl={r[3]:.6f} "
              f"(torque {GEAR * r[3]:.4f} N m)  target={r[4]:.1f} rad")
    m = metrics(t, q, f, 400)
    print(f"  metrics: rise {s(m['rise'])} s, overshoot {m['overshoot']:.1f} %, settle {s(m['settle'])} s, "
          f"ss error {m['ss_err']:.4f} rad, peak torque {m['peak']:.1f} N m, "
          f"ctrl at the limit for {np.sum(back[:-1, 3] >= 1.0) * 0.002:.3f} s")

    print()
    print("=" * 78)
    print("REFERENCE ANGLES for checking the browser copy (rad, at the given time)")
    print("=" * 78)
    for kp_, kd_, h_, gc in ((400, 40, 0.002, False), (400, 112, 0.002, True), (60, 25, 0.002, False),
                             (400, 400, 0.04, True)):
        t, q, v, u, f = run(kp_, kd_, h=h_, gravity_comp=gc)
        picks = [0.5, 1.0, 2.0, 3.0]
        idx = [int(round(p / h_)) for p in picks]
        print(f"kp={kp_} kd={kd_} h={h_} gc={'on' if gc else 'off'}: " +
              "  ".join(f"q({p:.1f})={q[i]:.6f}" for p, i in zip(picks, idx)))


if __name__ == "__main__":
    main()
