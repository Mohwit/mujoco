"""Lesson 14: Mobile bases.

Drive a four-wheeled skid-steer base on a floor:
  1. read its pose from the free joint (position + quaternion -> yaw),
  2. command the wheels with velocity actuators,
  3. measure skid-steer kinematics (forward speed v, turn rate w, effective track width),
  4. read a gyro and a rangefinder,
  5. compare open-loop and closed-loop driving to two waypoints, also when one thing changes.

The model is a SMALL copy of the Capstone 2 warehouse robot base
(lessons/16-capstone-warehouse-robot/warehouse_robot.xml): same chassis box, same four
wheels (radius 0.06 m, at x = +-0.18 m, y = +-0.17 m), same wheel class (friction 1.5, condim 6,
damping 0.1, armature 0.005), same velocity actuators (kv 5, forcerange +-10), same option
line, same gyro / accelerometer / rangefinder. The arm, shelves and objects are left out, and
the robot starts at the origin facing +x (the warehouse robot starts facing +y).

Run:
    cd /Users/mohwit/Desktop/mujoco && .venv/bin/python lessons/14-mobile-bases/14_mobile_bases.py
"""

import json
from pathlib import Path

import mujoco
import numpy as np

XML = """
<mujoco model="mobile_base">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicit"
          solver="Newton" iterations="30" cone="elliptic" impratio="10"/>
  <default>
    <default class="robot">
      <geom friction="0.8 0.005 0.001" condim="3" contype="0" conaffinity="0"/>
    </default>
    <default class="wheel">
      <geom friction="1.5 0.005 0.001" condim="6" contype="1" conaffinity="1"/>
      <joint damping="0.1" armature="0.005"/>
    </default>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.01"/>
    <geom name="wall" type="box" pos="3 0 0.4" size="0.05 3 0.4"/>
    <body name="chassis" pos="0 0 0.11" childclass="robot">
      <freejoint name="chassis_free"/>
      <geom name="chassis_body" type="box" size="0.25 0.15 0.04" mass="8.0"/>
      <geom name="top_plate" type="box" size="0.20 0.12 0.005" pos="0 0 0.045" mass="0.5"/>
      <site name="imu_site" pos="0 0 0" size="0.01"/>
      <site name="rangefinder_site" pos="0.26 0 0" zaxis="1 0 0" size="0.01"/>
      <body name="wheel_FL" pos="0.18 0.17 -0.05" childclass="wheel">
        <joint name="wheel_FL_spin" type="hinge" axis="0 1 0"/>
        <geom name="wheel_FL_geom" type="cylinder" size="0.06 0.025" euler="1.5708 0 0" mass="0.3"/>
      </body>
      <body name="wheel_FR" pos="0.18 -0.17 -0.05" childclass="wheel">
        <joint name="wheel_FR_spin" type="hinge" axis="0 1 0"/>
        <geom name="wheel_FR_geom" type="cylinder" size="0.06 0.025" euler="1.5708 0 0" mass="0.3"/>
      </body>
      <body name="wheel_RL" pos="-0.18 0.17 -0.05" childclass="wheel">
        <joint name="wheel_RL_spin" type="hinge" axis="0 1 0"/>
        <geom name="wheel_RL_geom" type="cylinder" size="0.06 0.025" euler="1.5708 0 0" mass="0.3"/>
      </body>
      <body name="wheel_RR" pos="-0.18 -0.17 -0.05" childclass="wheel">
        <joint name="wheel_RR_spin" type="hinge" axis="0 1 0"/>
        <geom name="wheel_RR_geom" type="cylinder" size="0.06 0.025" euler="1.5708 0 0" mass="0.3"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <velocity name="drive_FL" joint="wheel_FL_spin" kv="5" forcerange="-10 10"/>
    <velocity name="drive_FR" joint="wheel_FR_spin" kv="5" forcerange="-10 10"/>
    <velocity name="drive_RL" joint="wheel_RL_spin" kv="5" forcerange="-10 10"/>
    <velocity name="drive_RR" joint="wheel_RR_spin" kv="5" forcerange="-10 10"/>
  </actuator>
  <sensor>
    <gyro name="imu_gyro" site="imu_site"/>
    <accelerometer name="imu_accel" site="imu_site"/>
    <rangefinder name="forward_range" site="rangefinder_site"/>
  </sensor>
</mujoco>
"""

H = 0.002            # timestep (s), from <option timestep>
R_WHEEL = 0.06       # wheel radius (m), cylinder size[0]
B_GEOM = 0.34        # geometric track width (m): wheels at y = +0.17 and -0.17
WALL_FACE_X = 2.95   # wall centre x 3.0 minus half thickness 0.05 (m, world)
WP = [(1.5, 0.0), (1.5, 1.0)]   # the two waypoints of the route (m, world)

# Closed-loop gains: the same numbers as Capstone 2 (16_capstone_closed_loop.py, navigate_step)
KP_HEADING, KD_HEADING, KP_SPEED, MAX_WHEEL = 6.0, 2.0, 4.0, 6.0
ARRIVE = 0.05        # arrival radius (m). Capstone 2 uses 0.10 m; we tighten it to 0.05 m.


def make_model(mu=None, extra_mass=0.0):
    """Build the model. Optionally change the sliding friction of floor AND wheels (one thing:
    'the floor is slippier'), or add a payload mass to the chassis."""
    m = mujoco.MjModel.from_xml_string(XML)
    if mu is not None:
        for name in ["floor", "wheel_FL_geom", "wheel_FR_geom", "wheel_RL_geom", "wheel_RR_geom"]:
            m.geom_friction[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name), 0] = mu
    if extra_mass:
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        scale = (m.body_mass[b] + extra_mass) / m.body_mass[b]
        m.body_mass[b] += extra_mass
        m.body_inertia[b] *= scale
    return m


def yaw_from_quat(q):
    """q = [w, x, y, z] (MuJoCo order). Rotation about world z, rad."""
    w, x, y, z = q
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def yaw_from_xmat(d, bid):
    """First column of the body rotation matrix = the body's +x (forward) axis in world."""
    R = d.xmat[bid].reshape(3, 3)
    return np.arctan2(R[1, 0], R[0, 0])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def settle(m, d):
    """Drop the robot onto its wheels for 0.5 s with the wheels off, then restart the clock."""
    mujoco.mj_resetData(m, d)
    mujoco.mj_forward(m, d)
    for _ in range(250):
        mujoco.mj_step(m, d)
    d.time = 0.0


def set_wheels(d, left, right, bias=0.0):
    """ctrl order: FL, FR, RL, RR. Left wheels get `left`, right wheels `right`.
    bias: a fault on the right motors, they spin (1 + bias) times what we ask."""
    d.ctrl[:] = [left, right * (1 + bias), left, right * (1 + bias)]


def open_loop(m, B_plan, bias=0.0, record=False):
    """Eyes closed: drive 1.5 m, stop, turn left 90 deg in place, stop, drive 1.0 m, stop.
    Every duration is computed in advance from v = r w and w_turn = r (wR - wL) / B_plan."""
    d = mujoco.MjData(m)
    settle(m, d)
    w_turn = R_WHEEL * (5 - (-5)) / B_plan
    plan = [(5, 5, 1.5 / (R_WHEEL * 5)), (0, 0, 0.5), (-5, 5, (np.pi / 2) / w_turn),
            (0, 0, 0.5), (5, 5, 1.0 / (R_WHEEL * 5)), (0, 0, 1.0)]
    path = [(0.0, d.qpos[0], d.qpos[1], yaw_from_quat(d.qpos[3:7]))]
    for left, right, T in plan:
        set_wheels(d, left, right, bias)
        for _ in range(int(round(T / H))):
            mujoco.mj_step(m, d)
            if record and round(d.time / H) % 100 == 0:
                path.append((d.time, d.qpos[0], d.qpos[1], yaw_from_quat(d.qpos[3:7])))
    err = np.hypot(d.qpos[0] - WP[1][0], d.qpos[1] - WP[1][1])
    # the plan ends with a 1.0 s stop, so it "arrives" 1.0 s before the end
    return dict(x=d.qpos[0], y=d.qpos[1], yaw=yaw_from_quat(d.qpos[3:7]), err=err,
                t_arrive=d.time - 1.0, plan=plan, path=path)


def closed_loop(m, bias=0.0, record=False, log=False, t_max=60.0):
    """Eyes open: every step, look (pose + gyro), correct (heading PD, speed from distance), repeat."""
    d = mujoco.MjData(m)
    settle(m, d)
    k, t_arr = 0, None
    path = [(0.0, d.qpos[0], d.qpos[1], yaw_from_quat(d.qpos[3:7]))]
    rows = []
    while d.time < t_max:
        if k < len(WP):
            x, y, th = d.qpos[0], d.qpos[1], yaw_from_quat(d.qpos[3:7])
            dx, dy = WP[k][0] - x, WP[k][1] - y
            dist = np.hypot(dx, dy)
            if dist < ARRIVE:
                k += 1
                if log:
                    rows.append(("reached waypoint", k, round(d.time, 3)))
                if k == len(WP):
                    t_arr = d.time
                    set_wheels(d, 0, 0, bias)
                continue
            heading_err = wrap(np.arctan2(dy, dx) - th)
            yaw_rate = d.sensordata[2]                         # gyro z, rad/s, chassis frame
            turn = np.clip(KP_HEADING * heading_err - KD_HEADING * yaw_rate, -MAX_WHEEL, MAX_WHEEL)
            alignment = max(0.0, 1.0 - abs(heading_err) / 0.8)  # 0 when more than 0.8 rad off
            creep = 1.5 if abs(heading_err) < 0.4 else 0.0
            speed = np.clip(max(KP_SPEED * dist * alignment, creep), 0.0, MAX_WHEEL)
            left = np.clip(speed - turn, -MAX_WHEEL, MAX_WHEEL)
            right = np.clip(speed + turn, -MAX_WHEEL, MAX_WHEEL)
            set_wheels(d, left, right, bias)
            if log and round(d.time / H) % 1000 == 0:
                rows.append((d.time, x, y, th, dist, heading_err, yaw_rate, turn, speed, left, right))
        elif d.time > t_arr + 1.0:
            break
        mujoco.mj_step(m, d)
        if record and round(d.time / H) % 100 == 0:
            path.append((d.time, d.qpos[0], d.qpos[1], yaw_from_quat(d.qpos[3:7])))
    err = np.hypot(d.qpos[0] - WP[1][0], d.qpos[1] - WP[1][1])
    return dict(x=d.qpos[0], y=d.qpos[1], yaw=yaw_from_quat(d.qpos[3:7]), err=err,
                t_arrive=t_arr, path=path, rows=rows)


def drive_steady(m, left, right, T=6.0, avg_from=2.0):
    """Hold one wheel command, average the motion from avg_from to T seconds."""
    d = mujoco.MjData(m)
    settle(m, d)
    set_wheels(d, left, right)
    cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    yaws, xs, ys, wheels, ts, forces = [], [], [], [], [], []
    while d.time < T - 1e-9:
        mujoco.mj_step(m, d)
        if d.time >= avg_from - 1e-9:
            ts.append(d.time); xs.append(d.qpos[0]); ys.append(d.qpos[1])
            yaws.append(yaw_from_quat(d.qpos[3:7])); wheels.append(d.qvel[6:10].copy())
            forces.append(d.actuator_force.copy())
    yw = np.unwrap(yaws)
    span = ts[-1] - ts[0]
    w = (yw[-1] - yw[0]) / span
    # forward speed: distance travelled along the heading
    dxy = np.diff(np.c_[xs, ys], axis=0)
    hd = np.c_[np.cos(yw[:-1]), np.sin(yw[:-1])]
    v = float(np.sum(dxy * hd)) / span
    wh = np.mean(wheels, axis=0)
    mean_force = np.mean(forces, axis=0)   # actuator force averaged over the same window (N m)
    return v, w, wh, mean_force


def main():
    np.set_printoptions(precision=4, suppress=True)
    m = make_model()
    d = mujoco.MjData(m)
    cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis")

    print("=== 1. The model (small copy of the Capstone 2 base) ===")
    print(f"nq = {m.nq}  (free joint 7 + four wheel hinges 4)")
    print(f"nv = {m.nv}  (free joint 6 + four wheel hinges 4)")
    print(f"nu = {m.nu}  (velocity actuators drive_FL, drive_FR, drive_RL, drive_RR)")
    print(f"nsensor = {m.nsensor}  sensordata size = {m.nsensordata}  (gyro 3 + accelerometer 3 + rangefinder 1)")
    print(f"total mass (kg): {m.body_subtreemass[cid]:.3f}   (chassis 8.0 + plate 0.5 + 4 wheels x 0.3)")
    print(f"wheel radius r (m): {R_WHEEL}   geometric track width B (m): {B_GEOM}   wheelbase (m): 0.36")
    gyro_adr = m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "imu_gyro")]
    rf_adr = m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "forward_range")]
    print(f"sensor_adr: imu_gyro -> sensordata[{gyro_adr}:{gyro_adr + 3}], forward_range -> sensordata[{rf_adr}]")
    print(f"actuator_gainprm[0][0] (kv) = {m.actuator_gainprm[0][0]}, biasprm[0][2] = {m.actuator_biasprm[0][2]}"
          f", forcerange = {m.actuator_forcerange[0]}")

    settle(m, d)
    mujoco.mj_forward(m, d)
    print("\n=== 2. Pose from the free joint, after settling 0.5 s ===")
    print(f"qpos[0:3] position (m, world): {d.qpos[0:3]}")
    print(f"qpos[3:7] quaternion [w x y z]: {d.qpos[3:7]}")
    print(f"qpos[7:11] wheel angles (rad): {d.qpos[7:11]}")
    print(f"ncon (wheel-floor contacts) = {d.ncon}")

    # turn left in place for 3 s, then read the yaw both ways
    set_wheels(d, -5, 5)
    for _ in range(int(3.0 / H)):
        mujoco.mj_step(m, d)
    set_wheels(d, 0, 0)
    for _ in range(int(0.5 / H)):
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)
    q = d.qpos[3:7].copy()
    R = d.xmat[cid].reshape(3, 3)
    print("After 3.0 s of left -5, right +5 rad/s, then 0.5 s stop:")
    print(f"  quaternion [w x y z]: {q}")
    w_, x_, y_, z_ = q
    print(f"  2(wz+xy) = {2*(w_*z_ + x_*y_):.6f}   1-2(y^2+z^2) = {1 - 2*(y_*y_ + z_*z_):.6f}")
    print(f"  yaw from quaternion  atan2(2(wz+xy), 1-2(y^2+z^2)) = {yaw_from_quat(q):.6f} rad = {np.degrees(yaw_from_quat(q)):.3f} deg")
    print(f"  xmat first column (body +x in world): [{R[0,0]:.6f} {R[1,0]:.6f} {R[2,0]:.6f}]")
    print(f"  yaw from xmat        atan2(R[1,0], R[0,0])        = {yaw_from_xmat(d, cid):.6f} rad")
    print(f"  difference (rad): {abs(yaw_from_quat(q) - yaw_from_xmat(d, cid)):.2e}")
    print(f"  half-angle check: 2*atan2(z, w) = {2*np.arctan2(q[3], q[0]):.6f} rad")
    print(f"  position (m, world): x={d.qpos[0]:.4f} y={d.qpos[1]:.4f}  (turned on the spot)")

    print("\n=== 3. Velocity actuators: wheel speed vs ctrl (steady, averaged 2-6 s) ===")
    for cmd in [(5, 5), (-5, 5)]:
        v, w, wh, dd = drive_steady(m, *cmd)
        print(f"ctrl L={cmd[0]:+d} R={cmd[1]:+d} rad/s -> wheel qvel[6:10] FL FR RL RR = {wh} rad/s"
              f"; mean actuator_force (N m) = {dd}"
              f"; kv*(ctrl - qvel) = {5*(cmd[1] - wh[1]):.4f}")

    print("\n=== 4. Skid-steer kinematics: measured v and w (averaged 2-6 s) ===")
    print("cmd L, R (rad/s) | wheel L, R measured | v meas (m/s) | r(wL+wR)/2 | w meas (rad/s) | r(wR-wL)/B_geom | B_eff = r(wR-wL)/w (m)")
    table = []
    for cmd in [(5, 5), (-5, 5), (-2, 2), (0, 5), (2, 5), (4, 6)]:
        v, w, wh, _ = drive_steady(m, *cmd)
        wL, wR = (wh[0] + wh[2]) / 2, (wh[1] + wh[3]) / 2
        v_f, w_f = R_WHEEL * (wL + wR) / 2, R_WHEEL * (wR - wL) / B_GEOM
        beff = R_WHEEL * (wR - wL) / w if abs(w) > 1e-4 else float("nan")
        table.append((cmd, wL, wR, v, w, beff))
        bs = f"{beff:.3f}" if np.isfinite(beff) else "  -  "
        print(f"  {cmd[0]:+d}, {cmd[1]:+d}          | {wL:6.3f}, {wR:6.3f}      | {v:7.4f}      | {v_f:7.4f}    | {w:7.4f}        | {w_f:7.4f}         | {bs}")
    beffs = [t[5] for t in table if np.isfinite(t[5])]
    print(f"B_eff range: {min(beffs):.3f} to {max(beffs):.3f} m, geometric B = {B_GEOM} m"
          f" -> {min(beffs)/B_GEOM:.2f}x to {max(beffs)/B_GEOM:.2f}x wider")
    v55, w55 = table[0][3], table[1][4]
    print(f"Prediction 'both wheels 5 rad/s': r*5 = {R_WHEEL*5:.3f} m/s, measured {v55:.4f} m/s")
    print(f"Prediction 'left -5, right +5' with B_geom: {R_WHEEL*10/B_GEOM:.4f} rad/s, measured {w55:.4f} rad/s")

    # turn-rate wobble once per wheel revolution while spinning in place
    d2 = mujoco.MjData(m)
    settle(m, d2)
    set_wheels(d2, -5, 5)
    rates = []
    prev = yaw_from_quat(d2.qpos[3:7])
    dips, low = [], False
    while d2.time < 8.0 - 1e-9:
        mujoco.mj_step(m, d2)
        cur = yaw_from_quat(d2.qpos[3:7])
        if d2.time > 2.0:
            rates.append(wrap(cur - prev) / H)
            if rates[-1] < 0.40 and not low:
                dips.append(d2.time)
            low = rates[-1] < 0.40
        prev = cur
    print(f"Spin in place, 2-8 s: turn rate min {min(rates):.4f}, max {max(rates):.4f} rad/s")
    print(f"  {len(dips)} dips below 0.40 rad/s in 6 s, mean spacing {np.mean(np.diff(dips)):.4f} s"
          f" (a fast stick-slip chatter of the scrubbing wheels, about {1/np.mean(np.diff(dips)):.1f} per second)")

    print("\n=== 5. Sensors: gyro vs yaw differences, rangefinder vs geometry ===")
    d3 = mujoco.MjData(m)
    settle(m, d3)
    mujoco.mj_forward(m, d3)
    print(f"start: rangefinder = {d3.sensordata[rf_adr]:.4f} m, geometric {WALL_FACE_X} - (x + 0.26) = "
          f"{WALL_FACE_X - (d3.qpos[0] + 0.26):.4f} m")
    set_wheels(d3, 2, 5)    # a gentle left curve
    prev = yaw_from_quat(d3.qpos[3:7])
    print("time (s) | gyro z (rad/s, chassis) | (yaw_now - yaw_prev)/h (rad/s) | rangefinder (m) | geometric (m)")
    for _ in range(int(3.0 / H)):
        mujoco.mj_step(m, d3)
        cur = yaw_from_quat(d3.qpos[3:7])     # qpos is already the NEW state after mj_step
        if round(d3.time / H) % 250 == 0:
            mujoco.mj_forward(m, d3)       # refresh sensors to the current state
            # after mj_forward the gyro reports the NEW angular velocity, the one that moved
            # qpos during this step (velocity first, then position)
            sx = d3.site_xpos[1]
            th = yaw_from_xmat(d3, cid)
            geo = (WALL_FACE_X - sx[0]) / np.cos(th)
            print(f"  {d3.time:5.2f}  | {d3.sensordata[gyro_adr+2]: .6f}               | {wrap(cur - prev)/H: .6f}                      | {d3.sensordata[rf_adr]:.4f}          | {geo:.4f}")
        prev = cur
    # face +y: no wall ahead
    d4 = mujoco.MjData(m)
    settle(m, d4)
    d4.qpos[3:7] = [np.cos(np.pi / 4), 0, 0, np.sin(np.pi / 4)]
    mujoco.mj_forward(m, d4)
    print(f"facing +y (yaw 90 deg, nothing ahead): rangefinder = {d4.sensordata[rf_adr]:.1f}  (-1 means no hit)")
    d5 = mujoco.MjData(m)
    settle(m, d5)
    mujoco.mj_forward(m, d5)
    print(f"accelerometer at rest (m/s^2, chassis frame): {d5.sensordata[3:6]}  (gravity reaction, +z up)")

    print("\n=== 6. Open loop vs closed loop: route (0,0) -> (1.5, 0) -> (1.5, 1.0) m, world ===")
    b_spin = table[1][5]
    # open-loop plans use the commanded wheel speeds, so calibrate B from the commands too
    B_cmd = R_WHEEL * 10 / w55
    print(f"Open-loop turn time with B_geom = {B_GEOM}: {(np.pi/2)/(R_WHEEL*10/B_GEOM):.3f} s;"
          f" with B_cal = r*10/w_measured = {B_cmd:.3f} m: {(np.pi/2)/(R_WHEEL*10/B_cmd):.3f} s")
    ol_geo = open_loop(m, B_GEOM)
    ol = open_loop(m, B_cmd, record=True)
    cl = closed_loop(m, record=True, log=True)
    print("Plan (left, right rad/s, seconds) with B_cal:", [(l, r_, round(float(T), 3)) for l, r_, T in ol["plan"]])
    print(f"open loop, B_geom : end x={ol_geo['x']:.4f} y={ol_geo['y']:.4f} yaw={np.degrees(ol_geo['yaw']):.2f} deg  error {ol_geo['err']:.4f} m  time {ol_geo['t_arrive']:.2f} s")
    print(f"open loop, B_cal  : end x={ol['x']:.4f} y={ol['y']:.4f} yaw={np.degrees(ol['yaw']):.2f} deg  error {ol['err']:.4f} m  time {ol['t_arrive']:.2f} s")
    print(f"closed loop       : end x={cl['x']:.4f} y={cl['y']:.4f} yaw={np.degrees(cl['yaw']):.2f} deg  error {cl['err']:.4f} m  time {cl['t_arrive']:.2f} s")
    print("Closed-loop decisions every 2 s: t, x, y, yaw, dist, heading_err, gyro_z, turn, speed, left, right")
    for row in cl["rows"]:
        if row[0] == "reached waypoint":
            print(f"  -- reached waypoint {row[1]} at t = {row[2]} s --")
        else:
            print("  " + "  ".join(f"{v:7.4f}" for v in row))

    print("\n=== 7. Change one thing (same plans, same gains) ===")
    cases = [("nominal floor (mu 1.5)", dict(), 0.0),
             ("slippery floor (mu 0.3)", dict(mu=0.3), 0.0),
             ("payload +5 kg", dict(extra_mass=5.0), 0.0),
             ("right motors 10% slow", dict(), -0.10)]
    results = {}
    print("case                      | open loop error (m) | open time (s) | closed error (m) | closed time (s)")
    for name, kw, bias in cases:
        mm = make_model(**kw)
        o = open_loop(mm, B_cmd, bias, record=True)
        c = closed_loop(mm, bias, record=True)
        results[name] = (o, c)
        print(f"{name:25s} | {o['err']:.4f}              | {o['t_arrive']:.2f}         | {c['err']:.4f}           | {c['t_arrive']:.2f}")
        print(f"{'':25s}   open loop ends at x={o['x']:.3f} y={o['y']:.3f} yaw={np.degrees(o['yaw']):.1f} deg")

    ms = make_model(mu=0.3)
    vs, _, whs, _ = drive_steady(ms, 5, 5)
    _, ws, whs2, fs = drive_steady(ms, -5, 5)
    print(f"Why the slippery floor fools open loop: on mu 0.3, both 5 rad/s still gives v = {vs:.4f} m/s,"
          f" but -5/+5 turns at {ws:.4f} rad/s (mu 1.5: {w55:.4f}) with wheels at {whs2[1]:.3f} rad/s"
          f" and mean torque {fs[1]:.3f} N m. B_eff there = {R_WHEEL*2*whs2[1]/ws:.3f} m")
    print("\n=== 8. Calibrate a kinematic model for the browser ===")
    # least squares over every nominal closed-loop step: w_meas = r (cR - cL) / B, v_meas = k r (cL + cR)/2
    mm = make_model()
    dd = mujoco.MjData(mm)
    settle(mm, dd)
    A_w, Y_w, A_v, Y_v = [], [], [], []
    for cmd in [(5, 5), (-5, 5), (-2, 2), (0, 5), (2, 5), (4, 6), (3, 6), (-6, 6), (1, 3)]:
        v, w, wh, _ = drive_steady(mm, *cmd)
        A_w.append(R_WHEEL * (cmd[1] - cmd[0])); Y_w.append(w)
        A_v.append(R_WHEEL * (cmd[0] + cmd[1]) / 2); Y_v.append(v)
    A_w, Y_w, A_v, Y_v = map(np.array, (A_w, Y_w, A_v, Y_v))
    inv_B = float(A_w @ Y_w / (A_w @ A_w))
    k_v = float(A_v @ Y_v / (A_v @ A_v))
    B_fit = 1 / inv_B
    print(f"fit over 9 steady commands, using COMMANDED wheel speeds: B_eff = {B_fit:.3f} m, speed factor k_v = {k_v:.4f}")
    # replay the closed-loop controller on the kinematic unicycle
    def unicycle_closed(bias=0.0):
        x = y = th = 0.0; t = 0.0; k = 0; wz = 0.0
        while t < 60 and k < len(WP):
            dx, dy = WP[k][0] - x, WP[k][1] - y
            dist = np.hypot(dx, dy)
            if dist < ARRIVE:
                k += 1; continue
            he = wrap(np.arctan2(dy, dx) - th)
            turn = np.clip(KP_HEADING * he - KD_HEADING * wz, -MAX_WHEEL, MAX_WHEEL)
            al = max(0.0, 1 - abs(he) / 0.8); creep = 1.5 if abs(he) < 0.4 else 0.0
            sp = np.clip(max(KP_SPEED * dist * al, creep), 0, MAX_WHEEL)
            cl_ = np.clip(sp - turn, -MAX_WHEEL, MAX_WHEEL); cr = np.clip(sp + turn, -MAX_WHEEL, MAX_WHEEL) * (1 + bias)
            v = k_v * R_WHEEL * (cl_ + cr) / 2; wz = R_WHEEL * (cr - cl_) / B_fit
            th += wz * 0.01; x += v * np.cos(th) * 0.01; y += v * np.sin(th) * 0.01; t += 0.01
        return np.hypot(x - WP[1][0], y - WP[1][1]), t
    e_u, t_u = unicycle_closed()
    print(f"kinematic model, same controller, dt 0.01 s: arrival error {e_u:.4f} m, time {t_u:.2f} s"
          f" (MuJoCo: {cl['err']:.4f} m, {cl['t_arrive']:.2f} s)")

    # recorded runs for the sandbox (every 0.2 s): x, y, yaw
    out = {"B_eff": round(B_fit, 4), "k_v": round(k_v, 4), "B_cal": round(B_cmd, 4), "runs": {}}
    keys = dict(zip([c[0] for c in cases], ["nominal", "slippery", "heavy", "slowright"]))
    for name, (o, c) in results.items():
        out["runs"][keys[name]] = {
            "label": name,
            "open": {"err": round(o["err"], 4), "t": round(o["t_arrive"], 2),
                     "path": [[round(p[1], 3), round(p[2], 3), round(p[3], 3)] for p in o["path"]]},
            "closed": {"err": round(c["err"], 4), "t": round(c["t_arrive"], 2),
                       "path": [[round(p[1], 3), round(p[2], 3), round(p[3], 3)] for p in c["path"]]},
        }
    path = Path(__file__).parent / "14_recorded_runs.json"
    path.write_text(json.dumps(out, separators=(",", ":")))
    print(f"wrote {path.name} ({path.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
