"""
Gymnasium environment: learn to grasp and lift the red box with RL.
===================================================================

Scope: only the MANIPULATION part of the task. The robot starts already
docked at shelf A (with random parking error), arm in the "ready" pose.
Navigation stays with the classical controller — it already works, and
learning to drive across the warehouse would cost far more samples.
This split (classical for what is easy, learned for what is hard) is how
most real mobile manipulators are built.

    observation (19,)  — what the policy sees, all in the CHASSIS frame:
        [0:3]   grasp point position (m)
        [3:6]   box position (m)
        [6:9]   box − grasp point (m)
        [9]     gripper opening (m, 0 = closed, 0.05 = open)
        [10:12] touch L, R (N / 10)
        [12]    box height above its start (m)
        [13:17] arm joint angles: yaw, shoulder, elbow, wrist pitch (rad)
        [17:19] parking error the policy must compensate: box y in chassis, box x − 0.37

    action (4,) in [-1, 1]:
        [0:3]   move the grasp-point target by up to 1.5 cm per step (chassis frame)
        [3]     gripper: -1 = close, +1 = open

The action is in TASK SPACE: the policy says "move the gripper 1 cm
forward", and analytic IK (warehouse_common.arm_ik) turns that into joint
targets. Learning raw joint torques is possible but needs ~10x more data.

Reward (dense, so PPO/SAC get a gradient from the first episode):
    reach  = 1 − tanh(10 · |box − grasp|)          in [0, 1]
    grasp  = 0.5 if BOTH fingers are in contact with the BOX
             (not the touch sensors: see "reward hacking" below)
    lift   = 2 · clip(lift_height / 0.10, 0, 1) while grasped
    −5 and episode ends if the box is knocked over / pushed away
    success = box held 8 cm above its start (the episode keeps running:
    the policy must HOLD it, and the lift reward is already at its max)

Reward hacking lesson: the first version used the touch sensors for
"grasped". A touch sensor reports force from ANY contact inside its site —
so SAC learned to press the open fingers onto the shelf top, collect the
grasp bonus forever, and never touch the box. The fix is to check the
actual contact pairs (finger geom ↔ box geom) in data.contact, and to
penalize fingers touching the shelf.

Second lesson: the next version ENDED the episode with +10 on success.
But holding the box half-lifted pays ~2.4 per step forever, and with
gamma=0.98 that is worth ~100 — far more than a one-off +10. SAC learned
to lift 5 cm and hover, never crossing the 8 cm "finish line". Never make
success end the episode if staying alive is worth more than the bonus.

Control rate: 20 Hz (25 physics steps of 2 ms per env step).
"""

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from warehouse_common import (XML_PATH, ArmDriver, arm_ik, body_id,
                              ik_in_limits, sensor_val, world_to_chassis)

GRIPPER_PITCH = 0.3
READY_POINT = np.array([0.22, 0.0, 0.55])        # same as the classical controller
WORKSPACE_LO = np.array([0.15, -0.25, 0.38])     # chassis-frame box the target may move in
WORKSPACE_HI = np.array([0.60, 0.25, 0.75])
DOCK_X, DOCK_Y, DOCK_YAW = -0.85, 1.5, np.pi
LIFT_GOAL = 0.08


class WarehouseGraspEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": 20}

    def __init__(self, frame_skip=25, max_steps=100, render_mode=None):
        self.model = mujoco.MjModel.from_xml_path(str(XML_PATH))
        self.data = mujoco.MjData(self.model)
        self.arm = ArmDriver(self.model)
        self.frame_skip = frame_skip
        self.max_steps = max_steps
        self.render_mode = render_mode
        self._renderer = None

        self.box_id = body_id(self.model, "red_box")
        bj = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "red_box_free")
        self.box_qadr = self.model.jnt_qposadr[bj]
        cj = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "blue_cyl_free")
        self.cyl_qadr = self.model.jnt_qposadr[cj]
        shelf = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, n)
                 for n in ["sA_top", "sA_low", "sA_leg1", "sA_leg2", "sA_leg3", "sA_leg4"]]
        arm_bodies = {body_id(self.model, n) for n in
                      ["upper_arm", "forearm", "wrist", "wrist_tilt", "finger_L", "finger_R"]}
        self.shelf_geoms = set(shelf)
        self.arm_geoms = {g for g in range(self.model.ngeom)
                          if self.model.geom_bodyid[g] in arm_bodies}

        self.action_space = spaces.Box(-1.0, 1.0, shape=(4,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(19,), dtype=np.float64)

    # ── Gym API ──────────────────────────────────────────────────

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        m, d, rng = self.model, self.data, self.np_random
        mujoco.mj_resetDataKeyframe(m, d, 0)

        # Domain randomization: parking error + object placement error.
        # The policy must learn to compensate, not memorize one pose.
        yaw = DOCK_YAW + rng.uniform(-0.08, 0.08)
        d.qpos[0] = DOCK_X + rng.uniform(-0.04, 0.04)
        d.qpos[1] = DOCK_Y + rng.uniform(-0.06, 0.06)
        d.qpos[3:7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        d.qpos[self.box_qadr] = -1.22 + rng.uniform(-0.03, 0.03)
        d.qpos[self.box_qadr + 1] = 1.50 + rng.uniform(-0.04, 0.04)

        q = arm_ik(READY_POINT, GRIPPER_PITCH)
        q["wrist_roll"] = 0.0
        for n, v in q.items():
            d.qpos[self.arm.qadr[n]] = v
        self.arm.set_targets(q, grip=0.05)
        mujoco.mj_forward(m, d)

        # Let wheels and objects settle onto their supports
        for _ in range(100):
            self.arm.stop_wheels(d)
            self.arm.apply(d)
            mujoco.mj_step(m, d)

        self.target = READY_POINT.copy()
        self.box_start = d.xpos[self.box_id].copy()
        self.steps = 0
        return self._obs(), {}

    def step(self, action):
        m, d = self.model, self.data
        a = np.clip(action, -1.0, 1.0)

        new_target = np.clip(self.target + 0.015 * a[:3], WORKSPACE_LO, WORKSPACE_HI)
        q = arm_ik(new_target, GRIPPER_PITCH)
        if ik_in_limits(m, q):               # unreachable request → keep old target
            self.target = new_target
            self.arm.set_targets(q)
        self.arm.set_targets(grip=0.025 * (a[3] + 1.0))   # [-1,1] → [0, 0.05] m

        for _ in range(self.frame_skip):
            self.arm.stop_wheels(d)
            self.arm.apply(d)
            mujoco.mj_step(m, d)
        self.steps += 1

        reward, terminated, info = self._reward()
        truncated = self.steps >= self.max_steps
        return self._obs(), reward, terminated, truncated, info

    def render(self):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, 480, 640)
        self._renderer.update_scene(self.data, camera="side")
        return self._renderer.render()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()

    # ── Internals ────────────────────────────────────────────────

    def _obs(self):
        return build_obs(self.model, self.data, self.box_start)

    def _grasped(self):
        return fingers_on_box(self.model, self.data)

    def _arm_hits_shelf(self):
        d = self.data
        for c in d.contact[:d.ncon]:
            pair = {c.geom1, c.geom2}
            if pair & self.shelf_geoms and pair & self.arm_geoms:
                return True
        return False

    def _reward(self):
        m, d = self.model, self.data
        box = d.xpos[self.box_id]
        grasp = sensor_val(m, d, "grasp_pos")
        dist = np.linalg.norm(box - grasp)
        grasped = self._grasped()
        lift = box[2] - self.box_start[2]

        reward = 1.0 - np.tanh(10.0 * dist)
        if grasped:
            reward += 0.5 + 2.0 * np.clip(lift / 0.10, 0.0, 1.0)   # saturates past the 8 cm goal
        if self._arm_hits_shelf():
            reward -= 0.2

        info = {"dist": dist, "lift": lift, "grasped": grasped,
                "is_success": bool(grasped and lift > LIFT_GOAL)}
        pushed = np.linalg.norm(box[:2] - self.box_start[:2]) > 0.08
        if box[2] < self.box_start[2] - 0.03 or (pushed and not grasped):
            return reward - 5.0, True, info
        return reward, False, info


def fingers_on_box(model, data):
    """True only if BOTH finger geoms are in contact with the red box geom."""
    g = {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n)
         for n in ["finger_L_geom", "finger_R_geom", "red_box_geom"]}
    left = right = False
    for c in data.contact[:data.ncon]:
        pair = {c.geom1, c.geom2}
        if g["red_box_geom"] in pair:
            left |= g["finger_L_geom"] in pair
            right |= g["finger_R_geom"] in pair
    return left and right


def build_obs(model, data, box_start):
    """Observation vector — shared by the env and the policy runner so the
    policy sees exactly the same features in training and deployment."""
    bid = body_id(model, "red_box")
    grasp = world_to_chassis(model, data, sensor_val(model, data, "grasp_pos"))
    box = world_to_chassis(model, data, data.xpos[bid])
    joints = [data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]]
              for n in ["arm_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch"]]
    return np.concatenate([
        grasp, box, box - grasp,
        sensor_val(model, data, "sens_gripper"),
        sensor_val(model, data, "touch_L") / 10.0,
        sensor_val(model, data, "touch_R") / 10.0,
        [data.xpos[bid, 2] - box_start[2]],
        joints,
        [box[1], box[0] - 0.37],
    ])


class GraspPolicyRunner:
    """Runs a trained policy inside the classical state machine
    (16_capstone_closed_loop.py --policy PATH).

    Call step() once per physics step; it queries the network every
    frame_skip steps (20 Hz), exactly like the training env did.
    Returns None while working, or a status string when finished.
    """

    def __init__(self, path, frame_skip=25, max_steps=150):
        from stable_baselines3 import SAC
        self.policy = SAC.load(path, device="cpu")
        self.frame_skip = frame_skip
        self.max_steps = max_steps
        self.reset()

    def reset(self):
        self.k = 0
        self.steps = 0
        self.box_start = None
        self.target = READY_POINT.copy()

    def step(self, model, data, arm):
        bid = body_id(model, "red_box")
        if self.box_start is None:
            self.box_start = data.xpos[bid].copy()
        if self.k % self.frame_skip == 0:
            obs = build_obs(model, data, self.box_start)
            action, _ = self.policy.predict(obs, deterministic=True)
            new_target = np.clip(self.target + 0.015 * action[:3], WORKSPACE_LO, WORKSPACE_HI)
            q = arm_ik(new_target, GRIPPER_PITCH)
            if ik_in_limits(model, q):
                self.target = new_target
                arm.set_targets(q)
            arm.set_targets(grip=0.025 * (float(action[3]) + 1.0))
            self.steps += 1
        self.k += 1

        t_L = sensor_val(model, data, "touch_L")[0]
        t_R = sensor_val(model, data, "touch_R")[0]
        lift = data.xpos[bid, 2] - self.box_start[2]
        if fingers_on_box(model, data) and lift > LIFT_GOAL:
            return f"grasped + lifted {lift * 100:.1f} cm (L={t_L:.1f}N R={t_R:.1f}N)"
        if self.steps >= self.max_steps:
            return f"policy timeout (lift={lift * 100:.1f} cm)"
        return None
