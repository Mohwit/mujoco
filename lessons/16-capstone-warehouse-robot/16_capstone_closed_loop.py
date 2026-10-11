"""
Capstone 2 (lesson 16): Closed-Loop Warehouse Robot Pick-and-Place
================================================================
Same robot and warehouse as the scripted demo, but now the robot
uses sensor feedback to navigate, grasp, and place.

Key differences from the open-loop script:
  - Navigation: PD heading control (gyro damping) + proportional distance
  - Docking:    fine position control so the arm base ends up at a known
                distance from the shelf edge
  - Arm:        analytic inverse kinematics (IK) — we ask for a 3D point,
                not for joint angles — moved along straight Cartesian lines
  - Grasp:      touch sensors confirm contact before lifting
  - Every phase has a goal condition, not a fixed duration

Perception shortcut: the controller reads the box pose from data.xpos.
On a real robot this would come from a camera + object detector.

Optional: --policy PATH swaps the classical REACH/GRASP/LIFT phases for an
RL policy trained with train_grasp_rl.py.

Run headless:
    uv run --python .venv/bin/python lessons/16-capstone-warehouse-robot/16_capstone_closed_loop.py --headless

Run with viewer:
    .venv/bin/mjpython lessons/16-capstone-warehouse-robot/16_capstone_closed_loop.py
"""

import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

from warehouse_common import (XML_PATH, ArmDriver, angle_wrap, arm_ik, body_id,
                              get_chassis_pose, ik_in_limits, sensor_val,
                              world_to_chassis)


# ── Controller Gains ─────────────────────────────────────────────

NAV_KP_HEADING = 6.0     # wheel rad/s per radian of heading error
NAV_KD_HEADING = 2.0     # wheel rad/s per (rad/s) of yaw rate (damping)
NAV_KP_SPEED = 4.0       # wheel rad/s per meter of distance
NAV_MAX_WHEEL = 6.0      # max wheel angular velocity (rad/s)
NAV_ARRIVE_DIST = 0.10   # meters — close enough to a standoff waypoint

ALIGN_KP = 5.0           # wheel rad/s per radian for pure rotation
ALIGN_TOL = 0.03         # radians — aligned enough (~1.7°)

DOCK_KP = 25.0           # wheel rad/s per meter of along-track error
DOCK_MAX_WHEEL = 2.5     # slow, careful final approach (rad/s ≈ 15 cm/s)
DOCK_TOL = 0.01          # meters

ARM_CONVERGE_TOL = 0.04  # radians — joint close enough to target
GRASP_TOUCH_THRESH = 0.5 # Newtons — minimum fingertip force to confirm grasp
PHASE_TIMEOUT = 30.0     # seconds — max time per phase before moving on

GRIP_OPEN = 0.05
GRIP_CLOSED = 0.0
GRIPPER_PITCH = 0.3      # gripper tilted 17° down: clears the shelf edge


# ── Task geometry (world frame, meters) ──────────────────────────

SHELF_A_STANDOFF = (-0.55, 1.5)   # stop here first, in open floor
SHELF_A_HEADING = np.pi           # face west (-X) toward shelf A
SHELF_A_DOCK_X = -0.85            # chassis x when docked: turret 37 cm from box

SHELF_B_STANDOFF = (0.55, -1.0)
SHELF_B_HEADING = 0.0             # face east (+X) toward shelf B
SHELF_B_DOCK_X = 0.85

# Arm poses in joint space (rad) for when the arm is not reaching
ARM_TUCKED = {"arm_yaw": 0.0, "shoulder_pitch": 0.0, "elbow_pitch": 2.4,
              "wrist_roll": 0.0, "wrist_pitch": -0.8}
# "Ready" / carry pose: grasp point held just in front of the turret, above
# shelf height, so docking never pushes the gripper under the shelf top.
# Point in the chassis frame (m): 10 cm ahead of the turret, ~66 cm above floor.
READY_POINT = np.array([0.22, 0.0, 0.55])


# ── Navigation ───────────────────────────────────────────────────

def set_wheels(data, wheels, left, right):
    """Differential drive: FL/RL are left wheels, FR/RR are right wheels."""
    left = np.clip(left, -NAV_MAX_WHEEL, NAV_MAX_WHEEL)
    right = np.clip(right, -NAV_MAX_WHEEL, NAV_MAX_WHEEL)
    data.ctrl[wheels[0]] = left
    data.ctrl[wheels[1]] = right
    data.ctrl[wheels[2]] = left
    data.ctrl[wheels[3]] = right


def navigate_step(model, data, wheels, target):
    """PD heading + proportional speed toward a 2D point. Returns done."""
    x, y, yaw = get_chassis_pose(model, data)
    dx, dy = target[0] - x, target[1] - y
    dist = np.hypot(dx, dy)
    if dist < NAV_ARRIVE_DIST:
        set_wheels(data, wheels, 0, 0)
        return True

    heading_err = angle_wrap(np.arctan2(dy, dx) - yaw)
    yaw_rate = sensor_val(model, data, "imu_gyro")[2]   # rad/s about world-ish z
    turn = np.clip(NAV_KP_HEADING * heading_err - NAV_KD_HEADING * yaw_rate,
                   -NAV_MAX_WHEEL, NAV_MAX_WHEEL)

    alignment = max(0.0, 1.0 - abs(heading_err) / 0.8)   # 0 when > 0.8 rad off
    min_creep = 1.5 if abs(heading_err) < 0.4 else 0.0
    speed = np.clip(max(NAV_KP_SPEED * dist * alignment, min_creep), 0.0, NAV_MAX_WHEEL)
    set_wheels(data, wheels, speed - turn, speed + turn)
    return False


def align_step(model, data, wheels, target_yaw):
    """Rotate in place to face target_yaw. Returns done."""
    _, _, yaw = get_chassis_pose(model, data)
    err = angle_wrap(target_yaw - yaw)
    yaw_rate = sensor_val(model, data, "imu_gyro")[2]
    if abs(err) < ALIGN_TOL and abs(yaw_rate) < 0.05:
        set_wheels(data, wheels, 0, 0)
        return True
    turn = np.clip(ALIGN_KP * err - 1.0 * yaw_rate, -NAV_MAX_WHEEL, NAV_MAX_WHEEL)
    # Four skid-steer wheels scrub sideways when turning in place; below ~1 rad/s
    # the friction wins and the robot stalls just outside the tolerance.
    if abs(err) >= ALIGN_TOL:
        turn = np.sign(turn) * max(abs(turn), 1.0)
    set_wheels(data, wheels, -turn, turn)
    return False


def dock_step(model, data, wheels, target_x, heading):
    """Drive straight (forward or backward) until chassis x == target_x,
    holding the heading. Used for the last few centimetres before a shelf
    and for backing away from it. Returns done."""
    x, _, yaw = get_chassis_pose(model, data)
    fwd = np.array([np.cos(heading), np.sin(heading)])
    along = (target_x - x) * fwd[0]          # signed along-track error (m)
    vel = data.qvel[0:2] @ fwd               # chassis speed along track (m/s)
    if abs(along) < DOCK_TOL and abs(vel) < 0.02:
        set_wheels(data, wheels, 0, 0)
        return True
    speed = np.clip(DOCK_KP * along, -DOCK_MAX_WHEEL, DOCK_MAX_WHEEL)
    yaw_rate = sensor_val(model, data, "imu_gyro")[2]
    turn = 4.0 * angle_wrap(heading - yaw) - 1.0 * yaw_rate
    set_wheels(data, wheels, speed - turn, speed + turn)
    return False


# ── Arm motion ───────────────────────────────────────────────────

class CartesianMove:
    """Move the grasp point along a straight line in the WORLD frame.

    Each control step we interpolate a waypoint on the line, convert it to
    the chassis frame, and solve IK. Straight lines matter near shelves:
    a joint-space move from A to B can swing the arm through the shelf.
    """

    def __init__(self, model, data, goal_world, duration, pitch=GRIPPER_PITCH):
        self.start = sensor_val(model, data, "grasp_pos")
        self.goal = np.asarray(goal_world, dtype=float)
        self.duration = duration
        self.pitch = pitch
        self.t0 = data.time

    def target(self, model, data):
        s = np.clip((data.time - self.t0) / self.duration, 0.0, 1.0)
        s = s * s * (3 - 2 * s)                   # smoothstep: zero velocity at ends
        p = self.start + s * (self.goal - self.start)
        q = arm_ik(world_to_chassis(model, data, p), self.pitch)
        return q if ik_in_limits(model, q) else None, s >= 1.0


class JointMove:
    """Interpolate joint targets from the current pose to a goal pose.
    Used in open space, where the path of the gripper does not matter and
    a straight Cartesian line might pass through joint limits."""

    def __init__(self, model, data, arm, goal_q, duration):
        self.start = {n: float(data.qpos[arm.qadr[n]]) for n in goal_q}
        self.goal = goal_q
        self.duration = duration
        self.t0 = data.time

    def target(self, model, data):
        s = np.clip((data.time - self.t0) / self.duration, 0.0, 1.0)
        s = s * s * (3 - 2 * s)
        return {n: self.start[n] + s * (self.goal[n] - self.start[n])
                for n in self.goal}, s >= 1.0


# ── Pick-and-place state machine ─────────────────────────────────

PHASES = [
    "SETTLE",
    "DRIVE_TO_SHELF_A", "ALIGN_TO_SHELF_A", "READY", "DOCK_AT_SHELF_A",
    "PRE_GRASP", "APPROACH", "GRASP", "LIFT", "CARRY",
    "BACK_OFF_A",
    "DRIVE_TO_SHELF_B", "ALIGN_TO_SHELF_B", "DOCK_AT_SHELF_B",
    "PLACE_ABOVE", "PLACE_DOWN", "RELEASE", "RETREAT",
    "BACK_OFF_B", "TUCK",
]


class PickPlaceController:
    """One step() call per mj_step. Holds the current phase and its goal."""

    def __init__(self, model, data, policy=None, verbose=True):
        self.model, self.data = model, data
        self.arm = ArmDriver(model)
        self.arm.set_targets(ARM_TUCKED, GRIP_OPEN)
        self.box_id = body_id(model, "red_box")
        self.target_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "target_site")
        self.policy = policy            # optional RL grasp policy (see warehouse_grasp_env.py)
        self.phases = list(PHASES)
        if policy is not None:
            # One learned phase replaces the four hand-written grasp phases
            i = self.phases.index("PRE_GRASP")
            self.phases[i:i + 4] = ["RL_GRASP"]
        self.verbose = verbose
        self.phase_idx = 0
        self.phase_t0 = data.time
        self.move = None
        self.status = ""
        self.results = []
        self.box_start = None
        self._enter()

    @property
    def phase(self):
        return self.phases[self.phase_idx] if self.phase_idx < len(self.phases) else "DONE"

    @property
    def done(self):
        return self.phase_idx >= len(self.phases)

    # Phase bookkeeping
    def _enter(self):
        m, d = self.model, self.data
        p = self.phase
        self.phase_t0 = d.time
        self.move = None
        box = d.xpos[self.box_id].copy()
        tgt = d.site_xpos[self.target_site].copy()
        back_a = np.array([0.08, 0.0, 0.03])     # pre-grasp: 8 cm toward the robot, 3 cm up
        if p == "PRE_GRASP":
            self.box_start = box
            self.move = CartesianMove(m, d, box + back_a, 1.5)
        elif p == "APPROACH":
            self.move = CartesianMove(m, d, box, 1.2)
        elif p == "LIFT":
            self.move = CartesianMove(m, d, self.box_start + [0, 0, 0.10], 1.2)
        elif p == "READY":
            q = arm_ik(READY_POINT, GRIPPER_PITCH)
            q["wrist_roll"] = 0.0
            self.move = JointMove(m, d, self.arm, q, 1.5)
        elif p == "CARRY":
            cid = body_id(m, "chassis")
            ready_world = d.xpos[cid] + d.xmat[cid].reshape(3, 3) @ READY_POINT
            self.move = CartesianMove(m, d, ready_world, 1.5)
        elif p == "TUCK":
            self.move = JointMove(m, d, self.arm, ARM_TUCKED, 1.5)
        elif p == "PLACE_ABOVE":
            self.move = CartesianMove(m, d, tgt + [0, 0, 0.08], 2.0)
        elif p == "PLACE_DOWN":
            self.move = CartesianMove(m, d, tgt + [0, 0, 0.005], 1.2)
        elif p == "RETREAT":
            self.move = CartesianMove(m, d, tgt + [-0.10, 0, 0.06], 1.2)
        if self.verbose:
            print(f"\n{'─' * 60}\n  Phase: {p}\n{'─' * 60}")

    def _finish(self, status):
        elapsed = self.data.time - self.phase_t0
        self.results.append((self.phase, status, elapsed))
        if self.verbose:
            print(f"  Result: {status}  ({elapsed:.2f}s)")
            show_state(self.model, self.data, "end of phase")
        self.phase_idx += 1
        if not self.done:
            self._enter()

    def _follow_move(self):
        """Track the active Cartesian move. Returns True when finished and settled."""
        q, finished = self.move.target(self.model, self.data)
        if q is None:
            return None                      # out of reach
        self.arm.set_targets(q)
        return finished and self.arm.joint_error(self.data) < ARM_CONVERGE_TOL

    def step(self):
        """Compute controls for this timestep. Call before mj_step."""
        if self.done:
            self.arm.stop_wheels(self.data)
            self.arm.apply(self.data)
            return
        m, d, p = self.model, self.data, self.phase
        wheels = self.arm.wheels
        elapsed = d.time - self.phase_t0
        result = None

        if p == "SETTLE":
            self.arm.stop_wheels(d)
            if elapsed > 0.5:
                result = "settled"
        elif p == "DRIVE_TO_SHELF_A":
            if navigate_step(m, d, wheels, SHELF_A_STANDOFF):
                result = "arrived at standoff"
        elif p == "ALIGN_TO_SHELF_A":
            if align_step(m, d, wheels, SHELF_A_HEADING):
                result = "aligned"
        elif p == "DOCK_AT_SHELF_A":
            if dock_step(m, d, wheels, SHELF_A_DOCK_X, SHELF_A_HEADING):
                result = "docked"
        elif p == "BACK_OFF_A":
            if dock_step(m, d, wheels, SHELF_A_STANDOFF[0], SHELF_A_HEADING):
                result = "backed off"
        elif p == "DRIVE_TO_SHELF_B":
            if navigate_step(m, d, wheels, SHELF_B_STANDOFF):
                result = "arrived at standoff"
        elif p == "ALIGN_TO_SHELF_B":
            if align_step(m, d, wheels, SHELF_B_HEADING):
                result = "aligned"
        elif p == "DOCK_AT_SHELF_B":
            if dock_step(m, d, wheels, SHELF_B_DOCK_X, SHELF_B_HEADING):
                result = "docked"
        elif p == "BACK_OFF_B":
            if dock_step(m, d, wheels, SHELF_B_STANDOFF[0], SHELF_B_HEADING):
                result = "backed off"

        elif p == "RL_GRASP":
            # The learned policy drives the arm + gripper until it has lifted the box
            self.arm.stop_wheels(d)
            result = self.policy.step(m, d, self.arm)

        elif self.move is not None:
            self.arm.stop_wheels(d)
            ok = self._follow_move()
            if ok is None:
                result = "IK failed (target out of reach)"
            elif ok:
                result = "reached"

        elif p == "GRASP":
            self.arm.stop_wheels(d)
            self.arm.set_targets(grip=GRIP_CLOSED)
            t_L = sensor_val(m, d, "touch_L")[0]
            t_R = sensor_val(m, d, "touch_R")[0]
            if t_L > GRASP_TOUCH_THRESH and t_R > GRASP_TOUCH_THRESH and elapsed > 0.6:
                result = f"grasped (L={t_L:.1f}N R={t_R:.1f}N)"
            elif elapsed > 3.0:
                result = f"no contact (L={t_L:.2f}N R={t_R:.2f}N)"
        elif p == "RELEASE":
            self.arm.stop_wheels(d)
            self.arm.set_targets(grip=GRIP_OPEN)
            if sensor_val(m, d, "sens_gripper")[0] > 0.045:
                result = "released"

        if result is None and elapsed > PHASE_TIMEOUT:
            result = "timeout"
        self.arm.apply(d)
        if result is not None:
            self._finish(result)


# ── Reporting ────────────────────────────────────────────────────

def show_state(model, data, label):
    """Print key robot state with physical labels and units."""
    x, y, yaw = get_chassis_pose(model, data)
    t_L = sensor_val(model, data, "touch_L")[0]
    t_R = sensor_val(model, data, "touch_R")[0]
    g = sensor_val(model, data, "grasp_pos")
    box = data.xpos[body_id(model, "red_box")]
    print(f"  [{label}]  t={data.time:.2f}s")
    print(f"    chassis pos   (m, world):  x={x:.3f}  y={y:.3f}  yaw={np.degrees(yaw):.1f}°")
    print(f"    grasp point   (m, world):  x={g[0]:.3f}  y={g[1]:.3f}  z={g[2]:.3f}")
    print(f"    red box       (m, world):  x={box[0]:.3f}  y={box[1]:.3f}  z={box[2]:.3f}")
    print(f"    gripper opening      (m):  {sensor_val(model, data, 'sens_gripper')[0]:.4f}")
    print(f"    touch                (N):  L={t_L:.2f}  R={t_R:.2f}")


def report(model, data, ctrl):
    print("\n" + "═" * 60)
    print("  TASK FINISHED")
    print("═" * 60)
    print(f"\n  Total simulation time: {data.time:.2f}s\n")
    for name, status, elapsed in ctrl.results:
        bad = any(w in status for w in ("timeout", "failed", "no contact"))
        print(f"    {'✗' if bad else '✓'} {name:18s}  {status:34s}  ({elapsed:.2f}s)")
    box = data.xpos[body_id(model, "red_box")]
    tgt = data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "target_site")]
    err = np.linalg.norm(box[:2] - tgt[:2])
    on_shelf = box[2] > 0.55
    print(f"\n  Red box final pos (m): x={box[0]:.3f}  y={box[1]:.3f}  z={box[2]:.3f}")
    print(f"  Distance to target (m, horizontal): {err:.3f}")
    print(f"  SUCCESS: {err < 0.05 and on_shelf}\n")
    return err < 0.05 and on_shelf


# ── Main ─────────────────────────────────────────────────────────

def load_policy():
    if "--policy" not in sys.argv:
        return None
    from warehouse_grasp_env import GraspPolicyRunner
    path = sys.argv[sys.argv.index("--policy") + 1]
    print(f"Using RL grasp policy: {path}")
    return GraspPolicyRunner(path)


def main():
    model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    headless = "--headless" in sys.argv or "--print" in sys.argv
    policy = load_policy()

    print(f"Warehouse Robot — closed-loop controller  "
          f"(nbody={model.nbody}, nu={model.nu}, dt={model.opt.timestep}s)")

    if headless:
        ctrl = PickPlaceController(model, data, policy=policy, verbose=True)
        while not ctrl.done:
            ctrl.step()
            mujoco.mj_step(model, data)
        ok = report(model, data, ctrl)
        sys.exit(0 if ok else 1)

    ctrl = PickPlaceController(model, data, policy=policy, verbose=False)
    print("Launching viewer. Esc to quit.")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        last_phase = None
        while viewer.is_running():
            t0 = time.time()
            if ctrl.phase != last_phase:
                print(f"Phase: {ctrl.phase}")
                last_phase = ctrl.phase
            ctrl.step()
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0.0, model.opt.timestep - (time.time() - t0)))
            if ctrl.done and last_phase != "DONE":
                report(model, data, ctrl)
                last_phase = "DONE"


if __name__ == "__main__":
    main()
