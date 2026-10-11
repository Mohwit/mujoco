"""
Capstone 2 (lesson 16): the numbers behind the interactive lesson
=================================================================
The warehouse folder already has the working robot:
    16_capstone_warehouse_robot.py   scripted, open loop (fixed wheel speeds for fixed times)
    16_capstone_closed_loop.py       closed-loop state machine (sensors, IK, grasp check)
    warehouse_grasp_env.py           Gymnasium env for the grasp, plus GraspPolicyRunner
    train_grasp_rl.py                SAC training and evaluation

This script does not change any of them. It imports them, runs them headless,
and prints every number the lesson page shows:

    1. Model tour: counts, joints (qpos/qvel addresses), actuators, sensors
    2. Open loop: the 13 scripted phases, where the robot really ends up
    3. Closed loop: phase table, one navigation step by hand, IK by hand, the grasp
    4. Open vs closed loop when the start moves 5 cm, and 5 random starts
    5. RL: env spec, one reset, saved learning curve, evaluation of the saved policy,
       the policy inside the full task, and the "hover" arithmetic
    (16_check_browser_maths.py checks the page's copy of the nav law and IK.)

Run (takes about a minute; it does NOT retrain anything):
    uv run --python .venv/bin/python lessons/16-capstone-warehouse-robot/16_capstone_lesson.py
Add --write-fragment to refresh the recorded replay inside _fragments/script.js.
"""

import importlib
import json
import re
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))

from warehouse_common import (XML_PATH, ArmDriver, L_FORE, L_GRIP, L_UPPER,   # noqa: E402
                              SHOULDER_IN_CHASSIS, angle_wrap, arm_ik, body_id,
                              get_chassis_pose, ik_in_limits, sensor_val,
                              world_to_chassis)

cl = importlib.import_module("16_capstone_closed_loop")
ol = importlib.import_module("16_capstone_warehouse_robot")

POLICY = HERE / "models" / "grasp_sac_best" / "best_model.zip"


def hdr(s):
    print(f"\n== {s} ==")


def new_sim(dx=0.0, dy=0.0):
    m = mujoco.MjModel.from_xml_path(str(XML_PATH))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    d.qpos[0] += dx
    d.qpos[1] += dy
    mujoco.mj_forward(m, d)
    return m, d


def jname(m, j):
    return mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)


# ───────────────────────── recording helpers ─────────────────────────

class Recorder:
    """Samples the state for the page's replay: every 0.1 s (10 Hz) while the
    arm works, every 0.2 s (5 Hz) while the base drives. The page interpolates
    between samples. Values are stored as small integers to keep the page light:
    t in 0.1 s, positions in cm, box height in mm, angles in 0.01 rad, wheel
    commands in 0.1 rad/s, gripper opening in mm, touch in 0.1 N, range in cm."""

    def __init__(self, m, hz=None):
        self.m = m
        self.fast = int(round(0.1 / m.opt.timestep))
        self.slow = 2 * self.fast
        self.frames = []
        self.a = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
                  for n in ["drive_FL", "drive_FR"]}
        self.q = {n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
                  for n in ["arm_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch"]}
        self.box = body_id(m, "red_box")

    def maybe(self, d, k, phase, slow=True):
        if k % (self.slow if slow else self.fast) == 0:
            self.add(d, phase)

    def add(self, d, phase):
        x, y, yaw = get_chassis_pose(self.m, d)
        b = d.xpos[self.box]
        i = lambda v, s: int(round(float(v) * s))
        self.frames.append([
            i(d.time, 10), phase, i(x, 100), i(y, 100), i(yaw, 100), i(b[0], 100), i(b[1], 100), i(b[2], 1000),
            i(d.ctrl[self.a["drive_FL"]], 10), i(d.ctrl[self.a["drive_FR"]], 10),
            i(d.qpos[self.q["arm_yaw"]], 100), i(d.qpos[self.q["shoulder_pitch"]], 100),
            i(d.qpos[self.q["elbow_pitch"]], 100), i(d.qpos[self.q["wrist_pitch"]], 100),
            i(sensor_val(self.m, d, "sens_gripper")[0], 1000),
            i(sensor_val(self.m, d, "touch_L")[0], 10), i(sensor_val(self.m, d, "touch_R")[0], 10),
            i(sensor_val(self.m, d, "forward_range")[0], 100),
        ])


DRIVE_PHASES = ("SETTLE", "DRIVE", "ALIGN", "DOCK", "BACK_OFF")


# ───────────────────────── 1. model tour ─────────────────────────

def model_tour():
    m, d = new_sim()
    hdr("1. Model tour (warehouse_robot.xml, MuJoCo %s)" % mujoco.__version__)
    print(f"nbody={m.nbody} (incl. world)  njnt={m.njnt}  ngeom={m.ngeom}  nsite={m.nsite}  ncam={m.ncam}")
    print(f"nq={m.nq} (position numbers)  nv={m.nv} (velocity numbers)  nu={m.nu} (actuators)")
    print(f"nsensor={m.nsensor}  nsensordata={m.nsensordata}  ntendon={m.ntendon}  neq={m.neq}  nkey={m.nkey}")
    print(f"timestep={m.opt.timestep} s  integrator={mujoco.mjtIntegrator(m.opt.integrator).name}  "
          f"cone={mujoco.mjtCone(m.opt.cone).name}  impratio={m.opt.impratio}")
    print("joints (name, type, qposadr, nq, dofadr, nv):")
    tq = {0: 7, 1: 4, 2: 1, 3: 1}
    tv = {0: 6, 1: 3, 2: 1, 3: 1}
    tn = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}
    for j in range(m.njnt):
        t = int(m.jnt_type[j])
        print(f"  {jname(m, j):15s} {tn[t]:6s} qpos[{m.jnt_qposadr[j]}:{m.jnt_qposadr[j] + tq[t]}]"
              f"  nq={tq[t]}  qvel[{m.jnt_dofadr[j]}:{m.jnt_dofadr[j] + tv[t]}]  nv={tv[t]}")
    free = sum(1 for j in range(m.njnt) if m.jnt_type[j] == 0)
    other = m.njnt - free
    print(f"free joints={free} -> nq = {free}*7 + {other}*1 = {free * 7 + other},"
          f"  nv = {free}*6 + {other}*1 = {free * 6 + other}")
    print("actuators (ctrl index, name, type, joint, gain):")
    for i in range(m.nu):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        j = m.actuator_trnid[i, 0]
        g, b = m.actuator_gainprm[i, 0], m.actuator_biasprm[i, :3]
        if b[1] != 0:
            kind = f"position kp={g:g}"
        elif b[2] != 0:
            kind = f"velocity kv={g:g}"
        else:
            kind = f"motor gear={m.actuator_gear[i, 0]:g}"
        print(f"  ctrl[{i}] {name:16s} {kind:18s} joint={jname(m, j)}"
              f"  ctrlrange={m.actuator_ctrlrange[i].round(2).tolist() if m.actuator_ctrllimited[i] else 'none'}")
    print("sensors (name, type, sensordata slice, unit):")
    unit = {"jointpos": "rad or m", "touch": "N", "accelerometer": "m/s^2", "gyro": "rad/s",
            "rangefinder": "m", "framepos": "m, world"}
    for s in range(m.nsensor):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SENSOR, s)
        tname = mujoco.mjtSensor(m.sensor_type[s]).name.replace("mjSENS_", "").lower()
        a = m.sensor_adr[s]
        print(f"  {name:16s} {tname:14s} sensordata[{a}:{a + m.sensor_dim[s]}]  {unit.get(tname, '')}")
    print(f"keyframes: {[mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_KEY, k) for k in range(m.nkey)]}")
    print(f"cameras: {[mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_CAMERA, k) for k in range(m.ncam)]}")
    masses = {n: m.body_mass[body_id(m, n)] for n in ["chassis", "red_box"]}
    robot_mass = m.body_subtreemass[body_id(m, "chassis")]
    print(f"chassis body mass={masses['chassis']:.2f} kg  whole robot (subtree)={robot_mass:.3f} kg"
          f"  red box={masses['red_box']:.2f} kg")
    g = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n) for n in
         ["chassis_body", "wheel_FL_geom", "finger_L_geom", "red_box_geom", "sA_top", "turret"]}
    print("geom contype/conaffinity/condim/friction[0]:")
    for n, i in g.items():
        print(f"  {n:14s} contype={m.geom_contype[i]} conaffinity={m.geom_conaffinity[i]}"
              f" condim={m.geom_condim[i]} friction={m.geom_friction[i, 0]:.1f}")
    print(f"wheel radius={m.geom_size[g['wheel_FL_geom'], 0]:.2f} m  -> 6 rad/s wheel = "
          f"{6 * m.geom_size[g['wheel_FL_geom'], 0]:.2f} m/s")
    return m


# ───────────────────────── 2. open loop ─────────────────────────

def run_open_loop(dx=0.0, dy=0.0, rec=None, verbose=False):
    """Same logic as 16_capstone_warehouse_robot.run_demo (fixed wheel speeds
    for fixed times, interpolated arm targets). Returns per-phase end poses."""
    m, d = new_sim(dx, dy)
    if rec is not None:
        rec.__init__(m)
    wheel = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
             for n in ["drive_FL", "drive_FR", "drive_RL", "drive_RR"]]
    yaw_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "yaw_motor")
    grip_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper_pos")
    arm_names = ["shoulder_pos", "elbow_pos", "wrist_roll_pos", "wrist_pitch_pos"]
    arm_ids = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in arm_names}
    prev_arm = {n: 0.3 if "shoulder" in n else (-0.3 if "elbow" in n else 0.0) for n in arm_names}
    prev_grip = 0.035
    ends, k = [], 0
    for pi, ph in enumerate(ol.PHASES):
        n_steps = int(ph["duration"] / m.opt.timestep)
        _, _, yprev = get_chassis_pose(m, d)
        turned = 0.0                              # unwrapped yaw change in this phase (rad)
        for s in range(n_steps):
            t = s / max(1, n_steps - 1)
            for i, w in enumerate(wheel):
                d.ctrl[w] = ph["wheels"][i]
            d.ctrl[yaw_id] = ph["yaw_torque"]
            for a in arm_names:
                d.ctrl[arm_ids[a]] = ol.lerp(prev_arm[a], ph["arm"][a], t)
            d.ctrl[grip_id] = ol.lerp(prev_grip, ph["gripper"], t)
            if rec is not None:
                rec.maybe(d, k, pi, slow=True)
            mujoco.mj_step(m, d)
            k += 1
            _, _, ynow = get_chassis_pose(m, d)
            turned += angle_wrap(ynow - yprev)
            yprev = ynow
        prev_arm, prev_grip = dict(ph["arm"]), ph["gripper"]
        if rec is not None and pi == len(ol.PHASES) - 1:
            rec.add(d, pi)
        x, y, yaw = get_chassis_pose(m, d)
        ends.append((ph["name"], d.time, x, y, yaw, np.degrees(turned),
                     d.xpos[body_id(m, "red_box")].copy(),
                     sensor_val(m, d, "touch_L")[0], sensor_val(m, d, "touch_R")[0]))
    return m, d, ends


def open_loop_report():
    hdr("2. Open loop (16_capstone_warehouse_robot.py logic, fixed wheel speeds x fixed times)")
    rec = Recorder.__new__(Recorder)
    m, d, ends = run_open_loop(rec=rec)
    print("phase                         t_end(s)  wheels FL,FR,RL,RR (rad/s)   chassis x,y (m, world)  yaw(deg)  turned(deg)")
    for (name, t, x, y, yaw, turned, box, tl, tr), ph in zip(ends, ol.PHASES):
        print(f"  {name.replace(chr(8212), '-').replace(chr(176), ' deg'):28s} {t:6.2f}   {str(ph['wheels']):24s}  "
              f"{x:7.3f} {y:7.3f}          {np.degrees(yaw):7.1f}   {turned:7.1f}")
    box = ends[-1][6]
    print(f"red box final (m, world): x={box[0]:.3f} y={box[1]:.3f} z={box[2]:.3f}  (started -1.220, 1.500, 0.595 after settling)")
    print(f"touch during 'grasp' and 'lift' phases (N): L={ends[5][7]:.2f} R={ends[5][8]:.2f} / L={ends[6][7]:.2f} R={ends[6][8]:.2f}")
    _, x4, y4 = ends[3][1], ends[3][2], ends[3][3]
    print(f"after phase 04 the chassis is at ({x4:.3f}, {y4:.3f}); the closed loop docks at x=-0.85, y=1.5"
          f" -> miss = {np.hypot(x4 + 0.85, y4 - 1.5):.3f} m")
    return rec, ends


# ───────────────────────── 3. closed loop ─────────────────────────

def run_closed_loop(dx=0.0, dy=0.0, box_dx=0.0, box_dy=0.0, policy=None, rec=None, probes=False):
    m, d = new_sim(dx, dy)
    if box_dx or box_dy:
        bq = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "red_box_free")]
        d.qpos[bq] += box_dx
        d.qpos[bq + 1] += box_dy
        mujoco.mj_forward(m, d)
    if rec is not None:
        rec.__init__(m)
    ctrl = cl.PickPlaceController(m, d, policy=policy, verbose=False)
    out = {"nav": None, "ik": None, "grasp": None, "dock_a": None}
    k, last = 0, ctrl.phase
    while not ctrl.done:
        if probes and k == 1500 and ctrl.phase == "DRIVE_TO_SHELF_A":
            out["nav"] = nav_probe(m, d)
        if rec is not None:
            rec.maybe(d, k, ctrl.phase_idx, slow=ctrl.phase.startswith(DRIVE_PHASES))
        ctrl.step()
        if probes and out["nav"] is not None and out["nav"].get("ctrl") is None:
            out["nav"]["ctrl"] = (d.ctrl[ctrl.arm.wheels[0]], d.ctrl[ctrl.arm.wheels[1]])
        mujoco.mj_step(m, d)
        k += 1
        if ctrl.phase != last:
            if probes and last == "APPROACH":
                out["ik"] = ik_probe(m, d, ctrl)
            if probes and last == "GRASP":
                out["grasp"] = grasp_probe(m, d)
            if last.startswith("DOCK_AT_SHELF_A"):
                out["dock_a"] = get_chassis_pose(m, d)
            last = ctrl.phase
    if rec is not None:
        rec.add(d, len(ctrl.phases) - 1)
    box = d.xpos[body_id(m, "red_box")]
    tgt = d.site_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "target_site")]
    err = float(np.linalg.norm(box[:2] - tgt[:2]))
    ok = err < 0.05 and box[2] > 0.55
    return m, d, ctrl, out, box.copy(), err, ok


def nav_probe(m, d):
    """Recompute navigate_step's numbers by hand at this instant."""
    x, y, yaw = get_chassis_pose(m, d)
    tx, ty = cl.SHELF_A_STANDOFF
    dist = np.hypot(tx - x, ty - y)
    want = np.arctan2(ty - y, tx - x)
    herr = angle_wrap(want - yaw)
    yr = sensor_val(m, d, "imu_gyro")[2]
    turn_raw = cl.NAV_KP_HEADING * herr - cl.NAV_KD_HEADING * yr
    turn = float(np.clip(turn_raw, -cl.NAV_MAX_WHEEL, cl.NAV_MAX_WHEEL))
    align = max(0.0, 1.0 - abs(herr) / 0.8)
    creep = 1.5 if abs(herr) < 0.4 else 0.0
    speed = float(np.clip(max(cl.NAV_KP_SPEED * dist * align, creep), 0.0, cl.NAV_MAX_WHEEL))
    return dict(t=d.time, x=x, y=y, yaw=yaw, dist=dist, want=want, herr=herr, yr=yr, turn_raw=turn_raw,
                turn=turn, align=align, creep=creep, speed_raw=cl.NAV_KP_SPEED * dist * align, speed=speed,
                left=float(np.clip(speed - turn, -6, 6)), right=float(np.clip(speed + turn, -6, 6)), ctrl=None)


def ik_probe(m, d, ctrl):
    """At the end of APPROACH: solve arm_ik by hand for the box position."""
    box_w = d.xpos[ctrl.box_id].copy()
    p = world_to_chassis(m, d, box_w)
    r = p - SHOULDER_IN_CHASSIS
    pitch = cl.GRIPPER_PITCH
    h, v = np.hypot(r[0], r[1]), r[2]
    wf, wu = h - L_GRIP * np.cos(pitch), v + L_GRIP * np.sin(pitch)
    d2 = wf * wf + wu * wu
    cos_e = (d2 - L_UPPER ** 2 - L_FORE ** 2) / (2 * L_UPPER * L_FORE)
    q = arm_ik(p, pitch)
    actual = {n: float(d.qpos[ctrl.arm.qadr[n]]) for n in q}
    grasp_c = world_to_chassis(m, d, sensor_val(m, d, "grasp_pos"))
    return dict(p=p, r=r, h=h, v=v, wf=wf, wu=wu, d=np.sqrt(d2), cos_e=cos_e, q=q, actual=actual,
                grasp=grasp_c, miss=float(np.linalg.norm(sensor_val(m, d, "grasp_pos") - box_w)),
                limits=ik_in_limits(m, q))


def grasp_probe(m, d):
    from warehouse_grasp_env import fingers_on_box
    gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper_pos")
    names = []
    for c in d.contact[:d.ncon]:
        a = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom1)
        b = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)
        if "finger" in (a or "") + (b or ""):
            names.append(f"{a}<->{b}")
    return dict(tL=sensor_val(m, d, "touch_L")[0], tR=sensor_val(m, d, "touch_R")[0],
                grip=sensor_val(m, d, "sens_gripper")[0], force=d.actuator_force[gid],
                on_box=fingers_on_box(m, d), pairs=names)


def closed_loop_report():
    hdr("3. Closed loop (16_capstone_closed_loop.PickPlaceController, controller runs every mj_step)")
    rec = Recorder.__new__(Recorder)
    m, d, ctrl, out, box, err, ok = run_closed_loop(rec=rec, probes=True)
    print(f"physics step {m.opt.timestep} s -> controller rate {1 / m.opt.timestep:.0f} Hz (step() before every mj_step)")
    print(f"gains: NAV_KP_HEADING={cl.NAV_KP_HEADING} NAV_KD_HEADING={cl.NAV_KD_HEADING} NAV_KP_SPEED={cl.NAV_KP_SPEED}"
          f" NAV_MAX_WHEEL={cl.NAV_MAX_WHEEL} NAV_ARRIVE_DIST={cl.NAV_ARRIVE_DIST}")
    print(f"       ALIGN_KP={cl.ALIGN_KP} ALIGN_TOL={cl.ALIGN_TOL} DOCK_KP={cl.DOCK_KP} DOCK_MAX_WHEEL={cl.DOCK_MAX_WHEEL}"
          f" DOCK_TOL={cl.DOCK_TOL} ARM_CONVERGE_TOL={cl.ARM_CONVERGE_TOL} GRASP_TOUCH_THRESH={cl.GRASP_TOUCH_THRESH}"
          f" PHASE_TIMEOUT={cl.PHASE_TIMEOUT}")
    print("phase table (name, outcome, duration s, start s):")
    t0 = 0.0
    for name, status, el in ctrl.results:
        print(f"  {name:18s} {status:34s} {el:6.2f}  {t0:6.2f}")
        t0 += el
    print(f"total simulated time: {d.time:.2f} s   phases: {len(ctrl.results)}")
    print(f"red box final (m, world): x={box[0]:.3f} y={box[1]:.3f} z={box[2]:.3f}")
    print(f"distance to target (m, horizontal): {err:.3f}   SUCCESS: {ok}")
    n = out["nav"]
    print(f"\nnavigation step by hand at t={n['t']:.3f} s (DRIVE_TO_SHELF_A, target standoff {cl.SHELF_A_STANDOFF}):")
    print(f"  chassis x={n['x']:.4f} y={n['y']:.4f} m  yaw={n['yaw']:.4f} rad ({np.degrees(n['yaw']):.2f} deg)")
    print(f"  distance={n['dist']:.4f} m  heading to target={n['want']:.4f} rad  heading error={n['herr']:.4f} rad")
    print(f"  gyro z (yaw rate)={n['yr']:.4f} rad/s")
    print(f"  turn  = {cl.NAV_KP_HEADING}*{n['herr']:.4f} - {cl.NAV_KD_HEADING}*{n['yr']:.4f} = {n['turn_raw']:.4f}"
          f" -> clipped {n['turn']:.4f} rad/s")
    print(f"  alignment = max(0, 1 - |{n['herr']:.4f}|/0.8) = {n['align']:.4f}   creep={n['creep']}")
    print(f"  speed = clip(max({cl.NAV_KP_SPEED}*{n['dist']:.4f}*{n['align']:.4f}, {n['creep']}), 0, 6)"
          f" = clip({n['speed_raw']:.4f}) = {n['speed']:.4f} rad/s")
    print(f"  left = speed - turn = {n['left']:.4f}   right = speed + turn = {n['right']:.4f}  (rad/s, wheel ctrl)")
    print(f"  controller actually wrote ctrl[FL]={n['ctrl'][0]:.4f} ctrl[FR]={n['ctrl'][1]:.4f}")
    print(f"  full-precision inputs (for the page preset): herr={float(n['herr'])!r} dist={float(n['dist'])!r} yaw_rate={float(n['yr'])!r}")
    k = out["ik"]
    print(f"\nIK by hand at the end of APPROACH (target = box centre, gripper pitch {cl.GRIPPER_PITCH} rad):")
    print(f"  box in chassis frame p = ({k['p'][0]:.4f}, {k['p'][1]:.4f}, {k['p'][2]:.4f}) m")
    print(f"  shoulder in chassis = {SHOULDER_IN_CHASSIS.tolist()}  r = p - shoulder = ({k['r'][0]:.4f}, {k['r'][1]:.4f}, {k['r'][2]:.4f})")
    print(f"  h = {k['h']:.4f} m (horizontal)  v = {k['v']:.4f} m (vertical)")
    print(f"  wrist joint: wf = h - {L_GRIP}*cos(0.3) = {k['wf']:.4f}   wu = v + {L_GRIP}*sin(0.3) = {k['wu']:.4f}   d = {k['d']:.4f} m")
    print(f"  cos(elbow) = (d^2 - {L_UPPER}^2 - {L_FORE:.2f}^2) / (2*{L_UPPER}*{L_FORE:.2f}) = {k['cos_e']:.4f}")
    print("  joint      IK target   actual qpos  (rad)")
    for nme in k["q"]:
        print(f"  {nme:15s} {k['q'][nme]:8.4f}   {k['actual'][nme]:8.4f}")
    print(f"  full-precision box point (for the page preset): {[float(v) for v in k['p']]}")
    print(f"  in joint limits: {k['limits']}   |grasp_site - box| = {k['miss'] * 1000:.1f} mm")
    g = out["grasp"]
    print(f"\ngrasp confirmed: touch L={g['tL']:.2f} N R={g['tR']:.2f} N  gripper opening={g['grip'] * 1000:.1f} mm"
          f"  gripper actuator force={g['force']:.2f} N  fingers_on_box={g['on_box']}")
    print(f"  finger contacts: {sorted(set(g['pairs']))}")
    mg = 0.25 * 9.81
    print(f"  friction check (lesson 13): 2*mu*N = 2*2.0*{min(g['tL'], g['tR']):.2f} = {4 * min(g['tL'], g['tR']):.1f} N"
          f"  vs  m*g = 0.25*9.81 = {mg:.2f} N")
    return rec, ctrl, d


# ───────────────────────── 4. robustness ─────────────────────────

def robustness():
    hdr("4. Open vs closed loop when the start changes")
    _, _, ends0 = run_open_loop()
    _, _, ends5 = run_open_loop(dx=0.05)
    _, d0, *_ = run_closed_loop()
    print("one variable: start the robot 5 cm east (qpos[0] += 0.05), nothing else changes")
    print(f"  open loop, end of phase 04  nominal ({ends0[3][2]:.3f}, {ends0[3][3]:.3f})"
          f"  shifted ({ends5[3][2]:.3f}, {ends5[3][3]:.3f})"
          f"  moved {np.hypot(ends5[3][2] - ends0[3][2], ends5[3][3] - ends0[3][3]) * 100:.1f} cm")
    r0 = run_closed_loop()
    r5 = run_closed_loop(dx=0.05)
    print(f"  closed loop, docked at A    nominal ({r0[3]['dock_a'][0]:.3f}, {r0[3]['dock_a'][1]:.3f})"
          f"  shifted ({r5[3]['dock_a'][0]:.3f}, {r5[3]['dock_a'][1]:.3f})"
          f"  moved {np.hypot(r5[3]['dock_a'][0] - r0[3]['dock_a'][0], r5[3]['dock_a'][1] - r0[3]['dock_a'][1]) * 100:.1f} cm")
    print(f"  closed loop shifted: error {r5[5] * 1000:.1f} mm  SUCCESS {r5[6]}  time {r5[1].time:.2f} s")
    rng = np.random.default_rng(0)
    print("5 random starts (seed 0): start +-15 cm in x and y, box +-3 cm (x) and +-4 cm (y)")
    print("  run  start dx,dy (m)   box dx,dy (m)    open loop: box moved?   closed loop: error (mm), success, time (s)")
    wins = 0
    for i in range(5):
        dx, dy = rng.uniform(-0.15, 0.15, 2)
        bx, by = rng.uniform(-0.03, 0.03), rng.uniform(-0.04, 0.04)
        _, _, e = run_open_loop(dx, dy)
        moved = np.linalg.norm(e[-1][6][:2] - np.array([-1.22 + 0, 1.5])) > 0.01
        r = run_closed_loop(dx, dy, bx, by)
        wins += r[6]
        print(f"  {i}    {dx:+.3f},{dy:+.3f}     {bx:+.3f},{by:+.3f}       {'yes' if moved else 'no (never reached it)'}"
              f"              {r[5] * 1000:5.1f}  {r[6]}  {r[1].time:.2f}")
    print(f"closed loop successes: {wins}/5")


# ───────────────────────── 5. RL ─────────────────────────

def rl_report():
    hdr("5. RL: WarehouseGraspEnv + SAC (saved policy, nothing retrained)")
    from warehouse_grasp_env import GraspPolicyRunner, WarehouseGraspEnv, LIFT_GOAL
    env = WarehouseGraspEnv()
    m = env.model
    print(f"observation_space: {env.observation_space}")
    print(f"action_space:      {env.action_space}")
    print(f"frame_skip={env.frame_skip} -> one env step = {env.frame_skip * m.opt.timestep:.3f} s"
          f" = {1 / (env.frame_skip * m.opt.timestep):.0f} Hz;  max_steps={env.max_steps}"
          f" -> {env.max_steps * env.frame_skip * m.opt.timestep:.1f} s;  LIFT_GOAL={LIFT_GOAL} m")
    obs, _ = env.reset(seed=0)
    labels = ["grasp x", "grasp y", "grasp z", "box x", "box y", "box z", "box-grasp x", "box-grasp y",
              "box-grasp z", "gripper opening", "touch L/10", "touch R/10", "lift", "arm_yaw", "shoulder",
              "elbow", "wrist_pitch", "park err y", "park err x"]
    print("reset(seed=0) observation (chassis frame):")
    for i, (lab, v) in enumerate(zip(labels, obs)):
        print(f"  obs[{i:2d}] {lab:16s} {v:8.4f}")
    x, y, yaw = get_chassis_pose(m, env.data)
    print(f"  parked at x={x:.4f} y={y:.4f} yaw={np.degrees(yaw):.2f} deg (nominal -0.85, 1.5, 180)")

    ev = np.load(HERE / "models" / "eval_logs" / "evaluations.npz")
    ts, R, S = ev["timesteps"], ev["results"].mean(1), ev["successes"].mean(1)
    best = int(np.argmax(R))
    print(f"\nsaved learning curve (models/eval_logs/evaluations.npz): {len(ts)} evaluations x 20 episodes")
    print("  steps   mean return   success")
    for i in range(0, len(ts)):
        print(f"  {ts[i]:6d}  {R[i]:8.2f}     {S[i]:.2f}")
    print(f"  best mean return {R[best]:.2f} at {ts[best]} steps (EvalCallback saved this as best_model.zip)")
    first = ts[np.argmax(S > 0)]
    print(f"  first eval with any success: {first} steps")
    try:
        from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
        ea = EventAccumulator(str(HERE / "rl_runs" / "grasp_sac_1"))
        ea.Reload()
        rw = ea.Scalars("rollout/ep_rew_mean")
        fps = ea.Scalars("time/fps")
        print(f"tensorboard run grasp_sac_1: last step {rw[-1].step}, wall time {(rw[-1].wall_time - rw[0].wall_time) / 60:.1f} min,"
              f" final fps {fps[-1].value:.0f}, final training-episode return {rw[-1].value:.1f}")
    except Exception as e:                       # tensorboard not installed
        print(f"tensorboard not read ({e.__class__.__name__})")

    from stable_baselines3 import SAC
    pol = SAC.load(str(POLICY), device="cpu")
    print(f"\nSAC: lr={pol.learning_rate} gamma={pol.gamma} batch={pol.batch_size} buffer={pol.buffer_size}"
          f" tau={pol.tau} learning_starts={pol.learning_starts} net={pol.policy_kwargs.get('net_arch')}")
    n_ep = 20
    succ, rets, first_ok, rew_hover = 0, [], [], None
    for ep in range(n_ep):
        obs, _ = env.reset(seed=10_000 + ep)
        done, ret, fs = False, 0.0, None
        while not done:
            a, _ = pol.predict(obs, deterministic=True)
            obs, r, term, trunc, info = env.step(a)
            ret += r
            if fs is None and info["is_success"]:
                fs = env.steps
            if ep == 0 and rew_hover is None and info["grasped"] and info["lift"] >= 0.05:
                reach = 1 - np.tanh(10 * info["dist"])
                rew_hover = (env.steps, info["lift"], info["dist"], reach, 0.5,
                             2 * np.clip(info["lift"] / 0.10, 0, 1), r)
            done = term or trunc
        succ += info["is_success"]
        rets.append(ret)
        first_ok.append(fs)
        if ep == 0:
            final0 = (info["lift"], info["dist"], r, ret)
    ok_steps = [f for f in first_ok if f is not None]
    print(f"evaluation of best_model.zip, seeds 10000-{10_000 + n_ep - 1} (never used in training), deterministic:")
    print(f"  success at episode end: {succ}/{n_ep}   mean return {np.mean(rets):.1f}"
          f"  (min {np.min(rets):.1f}, max {np.max(rets):.1f})")
    print(f"  first env step with box > 8 cm and both fingers on it: mean {np.mean(ok_steps):.1f}"
          f"  (min {min(ok_steps)}, max {max(ok_steps)}) -> {np.mean(ok_steps) * 0.05:.2f} s")
    st, lift, dist, reach, gr, lf, r = rew_hover
    print(f"\nhover arithmetic (seed 10000, first step with lift >= 5 cm: env step {st}):")
    print(f"  lift={lift * 100:.2f} cm  |box-grasp|={dist * 1000:.1f} mm")
    print(f"  reward = reach {reach:.3f} + grasp {gr:.3f} + lift 2*clip({lift:.4f}/0.10) {lf:.3f} = {reach + gr + lf:.3f}"
          f"   (env returned {r:.3f})")
    gamma = 0.98
    print(f"  old design: value of hovering forever ~ r/(1-gamma) = {reach + gr + lf:.3f}/{1 - gamma:.2f} = {(reach + gr + lf) / (1 - gamma):.1f}")
    print(f"  value of crossing 8 cm once and ending: about r + 10 = {reach + gr + lf + 10:.1f}")
    print(f"  horizon 1/(1-gamma) = {1 / (1 - gamma):.0f} steps = {0.05 / (1 - gamma):.1f} s at 20 Hz")
    print(f"  end of episode, seed 10000: lift={final0[0] * 100:.2f} cm  reward/step={final0[2]:.3f}  return={final0[3]:.1f}")

    hdr("5b. The trained policy inside the full task (--policy)")
    rec = Recorder.__new__(Recorder)
    runner = GraspPolicyRunner(str(POLICY))
    m2, d2, ctrl, _, box, err, ok = run_closed_loop(policy=runner, rec=rec)
    print(f"phases: {ctrl.phases[4:7]} -> RL_GRASP replaces PRE_GRASP, APPROACH, GRASP, LIFT")
    for name, status, el in ctrl.results:
        if name in ("DOCK_AT_SHELF_A", "RL_GRASP", "CARRY"):
            print(f"  {name:18s} {status:42s} {el:6.2f} s")
    print(f"policy steps used: {runner.steps} (x25 physics steps)  total time {d2.time:.2f} s")
    print(f"red box final (m, world): x={box[0]:.3f} y={box[1]:.3f} z={box[2]:.3f}"
          f"   distance to target {err:.3f} m   SUCCESS {ok}")
    return rec, ctrl


def write_fragment(open_rec, closed_rec, rl_rec, closed_ctrl, rl_ctrl, open_ends):
    rec5 = Recorder.__new__(Recorder)
    run_open_loop(dx=0.05, rec=rec5)                  # same script, start 5 cm east (path only)
    ev = np.load(HERE / "models" / "eval_logs" / "evaluations.npz")
    js = HERE / "_fragments" / "script.js"
    text = js.read_text()
    ph_open = [p["name"].split(" ", 2)[-1].replace(chr(8212), "").replace(chr(176), " deg").strip().lower().capitalize() for p in ol.PHASES]
    payload = {
        "fields": ["t", "phase", "x", "y", "yaw", "bx", "by", "bz", "wl", "wr", "armYaw", "sh", "el", "wp",
                   "grip", "tL", "tR", "range"],
        "scale": [10, 1, 100, 100, 100, 100, 100, 1000, 10, 10, 100, 100, 100, 100, 1000, 10, 10, 100],
        "open": {"phases": ph_open,
                 "dur": [p["duration"] for p in ol.PHASES], "frames": open_rec.frames},
        "closed": {"phases": closed_ctrl.phases,
                   "dur": [round(e, 2) for _, _, e in closed_ctrl.results],
                   "res": [s for _, s, _ in closed_ctrl.results], "frames": closed_rec.frames},
        "open5": [[f[2], f[3]] for f in rec5.frames],
        "curve": {"t": ev["timesteps"].tolist(), "r": [round(float(v), 1) for v in ev["results"].mean(1)],
                  "s": [round(float(v), 2) for v in ev["successes"].mean(1)]},
        "rl": {"phases": rl_ctrl.phases,
               "dur": [round(e, 2) for _, _, e in rl_ctrl.results],
               "res": [s for _, s, _ in rl_ctrl.results], "frames": rl_rec.frames},
    }
    blob = json.dumps(payload, separators=(",", ":"))
    block = ("// REPLAY16-BEGIN (written by 16_capstone_lesson.py --write-fragment; recorded from MuJoCo)\n"
             f"const REPLAY16 = {blob};\n// REPLAY16-END")
    text, n = re.subn(r"// REPLAY16-BEGIN.*?// REPLAY16-END", lambda _: block, text, flags=re.S)
    if n != 1:
        sys.exit("REPLAY16 markers not found in _fragments/script.js")
    js.write_text(text)
    print(f"\nwrote replay into {js.relative_to(HERE)}: open {len(open_rec.frames)} frames,"
          f" closed {len(closed_rec.frames)}, rl {len(rl_rec.frames)}; data {len(blob) / 1024:.1f} KB")


def main():
    np.set_printoptions(precision=4, suppress=True)
    model_tour()
    open_rec, open_ends = open_loop_report()
    closed_rec, closed_ctrl, _ = closed_loop_report()
    robustness()
    rl_rec, rl_ctrl = rl_report()
    if "--write-fragment" in sys.argv:
        write_fragment(open_rec, closed_rec, rl_rec, closed_ctrl, rl_ctrl, open_ends)


if __name__ == "__main__":
    main()
