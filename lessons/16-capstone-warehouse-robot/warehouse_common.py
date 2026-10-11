"""
Shared helpers for the warehouse robot: sensor access, chassis pose,
analytic arm inverse kinematics, and gravity compensation.

Used by:
  16_capstone_closed_loop.py   (classical controller)
  warehouse_grasp_env.py       (Gymnasium RL environment)
"""

from pathlib import Path

import mujoco
import numpy as np

XML_PATH = Path(__file__).parent / "warehouse_robot.xml"

# ── Arm geometry (must match warehouse_robot.xml) ────────────────
# Shoulder pivot in the chassis frame: arm_base (0.12, 0, 0.05) + upper_arm (0, 0, 0.06)
SHOULDER_IN_CHASSIS = np.array([0.12, 0.0, 0.11])
L_UPPER = 0.30           # shoulder → elbow (m)
L_FORE = 0.26 + 0.03     # elbow → wrist_pitch joint: forearm 0.26 + wrist body 0.03 (m)
L_GRIP = 0.06            # wrist_pitch joint → grasp_site (m)

ARM_JOINTS = ["arm_yaw", "shoulder_pitch", "elbow_pitch", "wrist_roll", "wrist_pitch"]


def act_id(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)


def body_id(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)


def sensor_val(model, data, name):
    """Read a named sensor's current value from sensordata."""
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    adr = model.sensor_adr[sid]
    return data.sensordata[adr:adr + model.sensor_dim[sid]].copy()


def angle_wrap(angle):
    """Wrap angle to [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def get_chassis_pose(model, data):
    """Return (x, y, yaw) of the chassis in the world frame.
    The chassis local +X axis is 'forward'."""
    cid = body_id(model, "chassis")
    xmat = data.xmat[cid].reshape(3, 3)
    yaw = np.arctan2(xmat[1, 0], xmat[0, 0])
    return data.xpos[cid, 0], data.xpos[cid, 1], yaw


def world_to_chassis(model, data, p_world):
    """Express a world point in the chassis frame (m)."""
    cid = body_id(model, "chassis")
    R = data.xmat[cid].reshape(3, 3)
    return R.T @ (np.asarray(p_world) - data.xpos[cid])


def arm_ik(p_chassis, pitch_down=0.0):
    """Analytic IK: joint angles that put grasp_site at p_chassis.

    p_chassis   target point in the chassis frame (m)
    pitch_down  gripper angle below horizontal (rad); 0 = pointing straight ahead

    The arm is a yaw turret + a planar 3-link chain (shoulder, elbow, wrist
    pitch). Yaw aims the plane at the target; then a 2-link solution places
    the wrist joint, and wrist pitch sets the gripper angle.

    Joint angles are measured from vertical: 0 = link points straight up,
    positive = tilts forward. Returns dict of joint targets or None if the
    point is out of reach.
    """
    r = np.asarray(p_chassis) - SHOULDER_IN_CHASSIS
    yaw = np.arctan2(r[1], r[0])
    h = np.hypot(r[0], r[1])        # horizontal distance from shoulder
    v = r[2]                        # height above shoulder

    # Back off from the grasp point along the gripper direction to the wrist joint
    wf = h - L_GRIP * np.cos(pitch_down)
    wu = v + L_GRIP * np.sin(pitch_down)
    d2 = wf * wf + wu * wu

    cos_e = (d2 - L_UPPER**2 - L_FORE**2) / (2 * L_UPPER * L_FORE)
    if abs(cos_e) > 1.0:
        return None
    elbow = np.arccos(cos_e)        # positive branch: elbow bends forward
    shoulder = np.arctan2(wf, wu) - np.arctan2(L_FORE * np.sin(elbow),
                                               L_UPPER + L_FORE * np.cos(elbow))
    wrist = (np.pi / 2 + pitch_down) - shoulder - elbow
    return {"arm_yaw": yaw, "shoulder_pitch": shoulder,
            "elbow_pitch": elbow, "wrist_pitch": wrist}


def ik_in_limits(model, q):
    """True if every IK joint angle is inside its joint range."""
    if q is None:
        return False
    for name, val in q.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        lo, hi = model.jnt_range[jid]
        if not (lo <= val <= hi):
            return False
    return True


class ArmDriver:
    """Low-level arm commands shared by the classical controller and RL env.

    - Position actuators servo shoulder/elbow/wrist to targets.
    - The yaw turret is a raw torque motor, so we close a PD loop on it here.
    - Gravity compensation: add the gravity+Coriolis torque the arm needs
      (data.qfrc_bias) as a feed-forward, so the position servos only fight
      errors, not weight. Real arms do exactly this in firmware.
    """

    YAW_KP = 30.0   # N·m per rad
    YAW_KD = 3.0    # N·m per rad/s

    def __init__(self, model):
        self.model = model
        self.a = {n: act_id(model, n) for n in
                  ["shoulder_pos", "elbow_pos", "wrist_roll_pos",
                   "wrist_pitch_pos", "gripper_pos", "yaw_motor"]}
        self.wheels = [act_id(model, n) for n in
                       ["drive_FL", "drive_FR", "drive_RL", "drive_RR"]]
        jid = {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in ARM_JOINTS}
        self.qadr = {n: model.jnt_qposadr[j] for n, j in jid.items()}
        self.dadr = {n: model.jnt_dofadr[j] for n, j in jid.items()}
        grip_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "grip_L")
        grip_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "grip_R")
        self.arm_dofs = [self.dadr[n] for n in ARM_JOINTS] + \
                        [model.jnt_dofadr[grip_l], model.jnt_dofadr[grip_r]]
        self.target = {"arm_yaw": 0.0, "shoulder_pitch": 0.0, "elbow_pitch": 2.4,
                       "wrist_roll": 0.0, "wrist_pitch": -0.8}
        self.grip = 0.04

    def set_targets(self, q=None, grip=None):
        if q:
            self.target.update(q)
        if grip is not None:
            self.grip = grip

    def apply(self, data):
        """Write ctrl + feed-forward torques. Call once before every mj_step."""
        t = self.target
        data.ctrl[self.a["shoulder_pos"]] = t["shoulder_pitch"]
        data.ctrl[self.a["elbow_pos"]] = t["elbow_pitch"]
        data.ctrl[self.a["wrist_roll_pos"]] = t["wrist_roll"]
        data.ctrl[self.a["wrist_pitch_pos"]] = t["wrist_pitch"]
        data.ctrl[self.a["gripper_pos"]] = self.grip
        yaw = data.qpos[self.qadr["arm_yaw"]]
        yaw_vel = data.qvel[self.dadr["arm_yaw"]]
        data.ctrl[self.a["yaw_motor"]] = self.YAW_KP * angle_wrap(t["arm_yaw"] - yaw) \
            - self.YAW_KD * yaw_vel
        data.qfrc_applied[:] = 0.0
        data.qfrc_applied[self.arm_dofs] = data.qfrc_bias[self.arm_dofs]

    def joint_error(self, data):
        """Largest |target - actual| over the arm joints (rad)."""
        return max(abs(angle_wrap(self.target[n] - data.qpos[self.qadr[n]]))
                   for n in ARM_JOINTS)

    def stop_wheels(self, data):
        for w in self.wheels:
            data.ctrl[w] = 0.0
