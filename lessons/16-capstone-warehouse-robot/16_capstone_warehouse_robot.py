"""
Capstone 2 (lesson 16): Warehouse Robot Pick-and-Place
=====================================================
A mobile manipulator drives to a shelf, picks up a box, carries it
across a warehouse, and places it on a second shelf.

This script demonstrates every major MuJoCo concept:
  - Free joints (chassis, objects)
  - Hinge joints (wheels, arm)
  - Slide joints (gripper fingers)
  - Velocity actuators (wheels)
  - Motor actuators (arm yaw)
  - Position actuators (arm joints, gripper)
  - Tendon + equality constraint (gripper coupling)
  - Sensors (joint, touch, accelerometer, gyro, rangefinder)
  - Contacts and friction
  - Keyframes

Run headless:
    uv run --python .venv/bin/python lessons/16-capstone-warehouse-robot/16_capstone_warehouse_robot.py --headless

Run with viewer:
    .venv/bin/mjpython lessons/16-capstone-warehouse-robot/16_capstone_warehouse_robot.py
"""

import sys
import time
from pathlib import Path

import mujoco
import numpy as np


# ── Helpers ──────────────────────────────────────────────────────

def act_id(model, name):
    """Get actuator index by name."""
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)


def sensor_val(model, data, name):
    """Read a named sensor's current value from sensordata."""
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sid_name := name)
    adr = model.sensor_adr[sid]
    dim = model.sensor_dim[sid]
    return data.sensordata[adr:adr + dim].copy()


def show_model(model):
    """Print a summary of the model — every number has a physical meaning."""
    print("╔══════════════════════════════════════════════════════════╗")
    print("║        Warehouse Robot — Model Summary                  ║")
    print("╠══════════════════════════════════════════════════════════╣")
    print(f"║  bodies:     {model.nbody:3d}    (rigid links in the tree)     ║")
    print(f"║  joints:     {model.njnt:3d}    (degrees of freedom)           ║")
    print(f"║  geoms:      {model.ngeom:3d}    (collision/visual shapes)     ║")
    print(f"║  actuators:  {model.nu:3d}    (motors that accept ctrl)     ║")
    print(f"║  sensors:    {model.nsensor:3d}    (measurement channels)       ║")
    print(f"║  tendons:    {model.ntendon:3d}    (gripper coupling)           ║")
    print(f"║  equality:   {model.neq:3d}    (finger synchronization)     ║")
    print(f"║  keyframes:  {model.nkey:3d}    (home, pre-grasp)             ║")
    print(f"║  nq:         {model.nq:3d}    (position coordinates)        ║")
    print(f"║  nv:         {model.nv:3d}    (velocity coordinates)        ║")
    print(f"║  timestep:   {model.opt.timestep:.3f}  (seconds per step)         ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print()
    print("Actuators:")
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"  ctrl[{i}] → {name}")
    print()


def show_state(model, data, label):
    """Print key robot state with physical labels and units."""
    pos = data.qpos[:3]
    t_L = sensor_val(model, data, "touch_L")[0]
    t_R = sensor_val(model, data, "touch_R")[0]
    ee = sensor_val(model, data, "ee_pos")
    rng = sensor_val(model, data, "forward_range")[0]
    grip = sensor_val(model, data, "sens_gripper")[0]

    print(f"  [{label}]  t={data.time:.2f}s")
    print(f"    chassis world pos (m):  x={pos[0]:.3f}  y={pos[1]:.3f}  z={pos[2]:.3f}")
    print(f"    end-effector pos  (m):  x={ee[0]:.3f}  y={ee[1]:.3f}  z={ee[2]:.3f}")
    print(f"    gripper opening   (m):  {grip:.4f}")
    print(f"    touch sensors     (N):  L={t_L:.2f}  R={t_R:.2f}")
    print(f"    rangefinder       (m):  {rng:.3f}")


# ── Trajectory Phases ────────────────────────────────────────────
#
# Each phase sets wheel speeds (velocity actuators, rad/s) and
# arm targets (position actuators, rad or m).  The controller
# interpolates smoothly from the previous arm pose to the new one.
#
# Wheel convention: positive = forward rotation.
# Differential steering: left wheels and right wheels at different
# speeds produce turning.

PHASES = [
    # ── Phase 1: Settle and prepare ──
    {
        "name": "01 — SETTLE & OPEN GRIPPER",
        "duration": 0.5,
        "wheels": [0, 0, 0, 0],
        "yaw_torque": 0,
        "arm": {"shoulder_pos": 0.3, "elbow_pos": -0.3,
                "wrist_roll_pos": 0, "wrist_pitch_pos": 0},
        "gripper": 0.035,
    },
    # ── Phase 2: Drive north (+Y) toward Shelf A latitude ──
    {
        "name": "02 — DRIVE NORTH",
        "duration": 5.0,
        "wheels": [8, 8, 8, 8],     # all wheels forward, ~0.47 m/s
        "yaw_torque": 0,
        "arm": {"shoulder_pos": 0.3, "elbow_pos": -0.3,
                "wrist_roll_pos": 0, "wrist_pitch_pos": 0},
        "gripper": 0.035,
    },
    # ── Phase 3: Turn left 90° to face west (-X) ──
    {
        "name": "03 — TURN LEFT 90°",
        "duration": 3.2,
        "wheels": [-5, 5, -5, 5],   # left backward, right forward = CCW
        "yaw_torque": 0,
        "arm": {"shoulder_pos": 0.3, "elbow_pos": -0.3,
                "wrist_roll_pos": 0, "wrist_pitch_pos": 0},
        "gripper": 0.035,
    },
    # ── Phase 4: Drive west toward Shelf A, steer slightly north ──
    {
        "name": "04 — DRIVE TO SHELF A",
        "duration": 4.0,
        "wheels": [5, 6, 5, 6],     # slight left curve toward shelf
        "yaw_torque": 0,
        "arm": {"shoulder_pos": 0.3, "elbow_pos": -0.3,
                "wrist_roll_pos": 0, "wrist_pitch_pos": 0},
        "gripper": 0.035,
    },
    # ── Phase 5: Extend arm toward object on shelf ──
    {
        "name": "05 — EXTEND ARM",
        "duration": 2.5,
        "wheels": [0, 0, 0, 0],
        "yaw_torque": 0,
        "arm": {"shoulder_pos": 1.5, "elbow_pos": -1.2,
                "wrist_roll_pos": 0, "wrist_pitch_pos": 0.4},
        "gripper": 0.035,
    },
    # ── Phase 6: Close gripper ──
    {
        "name": "06 — CLOSE GRIPPER (GRASP)",
        "duration": 1.0,
        "wheels": [0, 0, 0, 0],
        "yaw_torque": 0,
        "arm": {"shoulder_pos": 1.5, "elbow_pos": -1.2,
                "wrist_roll_pos": 0, "wrist_pitch_pos": 0.4},
        "gripper": 0.0,
    },
    # ── Phase 7: Lift object off shelf ──
    {
        "name": "07 — LIFT OBJECT",
        "duration": 1.5,
        "wheels": [0, 0, 0, 0],
        "yaw_torque": 0,
        "arm": {"shoulder_pos": 0.8, "elbow_pos": -0.5,
                "wrist_roll_pos": 0, "wrist_pitch_pos": -0.2},
        "gripper": 0.0,
    },
    # ── Phase 8: Reverse away from Shelf A ──
    {
        "name": "08 — REVERSE FROM SHELF",
        "duration": 2.5,
        "wheels": [-6, -6, -6, -6],
        "yaw_torque": 0,
        "arm": {"shoulder_pos": 0.6, "elbow_pos": -0.4,
                "wrist_roll_pos": 0, "wrist_pitch_pos": -0.2},
        "gripper": 0.0,
    },
    # ── Phase 9: Turn right 180° to face east (+X) ──
    {
        "name": "09 — TURN RIGHT 180°",
        "duration": 6.0,
        "wheels": [6, -6, 6, -6],   # left forward, right backward = CW
        "yaw_torque": 0,
        "arm": {"shoulder_pos": 0.6, "elbow_pos": -0.4,
                "wrist_roll_pos": 0, "wrist_pitch_pos": -0.2},
        "gripper": 0.0,
    },
    # ── Phase 10: Drive SE toward Shelf B, steer right ──
    {
        "name": "10 — DRIVE TO SHELF B",
        "duration": 5.5,
        "wheels": [8, 5, 8, 5],     # slight right curve toward shelf B
        "yaw_torque": 0,
        "arm": {"shoulder_pos": 0.6, "elbow_pos": -0.4,
                "wrist_roll_pos": 0, "wrist_pitch_pos": -0.2},
        "gripper": 0.0,
    },
    # ── Phase 11: Lower arm to placement height ──
    {
        "name": "11 — LOWER ARM TO PLACE",
        "duration": 2.0,
        "wheels": [0, 0, 0, 0],
        "yaw_torque": 0,
        "arm": {"shoulder_pos": 1.4, "elbow_pos": -1.0,
                "wrist_roll_pos": 0, "wrist_pitch_pos": 0.3},
        "gripper": 0.0,
    },
    # ── Phase 12: Open gripper to release ──
    {
        "name": "12 — RELEASE OBJECT",
        "duration": 1.0,
        "wheels": [0, 0, 0, 0],
        "yaw_torque": 0,
        "arm": {"shoulder_pos": 1.4, "elbow_pos": -1.0,
                "wrist_roll_pos": 0, "wrist_pitch_pos": 0.3},
        "gripper": 0.035,
    },
    # ── Phase 13: Retract arm to home position ──
    {
        "name": "13 — RETRACT ARM",
        "duration": 2.0,
        "wheels": [0, 0, 0, 0],
        "yaw_torque": 0,
        "arm": {"shoulder_pos": 0.3, "elbow_pos": -0.3,
                "wrist_roll_pos": 0, "wrist_pitch_pos": 0},
        "gripper": 0.035,
    },
]


def lerp(a, b, t):
    """Linear interpolation, t clamped to [0, 1]."""
    t = max(0.0, min(1.0, t))
    return a + (b - a) * t


def run_demo(model, data, headless=True):
    """Execute the full pick-and-place sequence."""

    dt = model.opt.timestep
    wheel_ids = [act_id(model, n) for n in
                 ["drive_FL", "drive_FR", "drive_RL", "drive_RR"]]
    yaw_id = act_id(model, "yaw_motor")
    grip_id = act_id(model, "gripper_pos")

    # Cache arm actuator IDs
    arm_names = ["shoulder_pos", "elbow_pos", "wrist_roll_pos", "wrist_pitch_pos"]
    arm_ids = {n: act_id(model, n) for n in arm_names}

    # Track previous arm targets for interpolation
    prev_arm = {n: 0.3 if "shoulder" in n else (-0.3 if "elbow" in n else 0.0)
                for n in arm_names}
    prev_grip = 0.035

    log = []

    for phase in PHASES:
        name = phase["name"]
        dur = phase["duration"]
        n_steps = int(dur / dt)

        if headless:
            print(f"\n{'─' * 60}")
            print(f"  Phase: {name}")
            print(f"  Duration: {dur:.1f}s  ({n_steps} steps)")
            print(f"{'─' * 60}")

        for step in range(n_steps):
            t = step / max(1, n_steps - 1)

            # Wheels: set velocity targets directly (no interpolation)
            for i, wid in enumerate(wheel_ids):
                data.ctrl[wid] = phase["wheels"][i]

            # Arm yaw: raw torque
            data.ctrl[yaw_id] = phase["yaw_torque"]

            # Arm position actuators: smooth interpolation
            for aname in arm_names:
                data.ctrl[arm_ids[aname]] = lerp(prev_arm[aname],
                                                  phase["arm"][aname], t)

            # Gripper
            data.ctrl[grip_id] = lerp(prev_grip, phase["gripper"], t)

            mujoco.mj_step(model, data)

            # Log every 250 steps (~0.5s)
            if step % 250 == 0:
                entry = {
                    "time": data.time,
                    "phase": name,
                    "chassis_x": data.qpos[0],
                    "chassis_y": data.qpos[1],
                    "chassis_z": data.qpos[2],
                    "touch_L": sensor_val(model, data, "touch_L")[0],
                    "touch_R": sensor_val(model, data, "touch_R")[0],
                    "gripper": sensor_val(model, data, "sens_gripper")[0],
                }
                log.append(entry)

        # Update previous targets for next phase
        prev_arm = dict(phase["arm"])
        prev_grip = phase["gripper"]

        if headless:
            show_state(model, data, "end of phase")

    return log


# ── Main ─────────────────────────────────────────────────────────

def main():
    xml_path = Path(__file__).parent / "warehouse_robot.xml"
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    show_model(model)

    # Reset to home keyframe
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    headless = "--headless" in sys.argv or "--print" in sys.argv

    if headless:
        print("Running in headless mode (no viewer).\n")
        print("Initial state:")
        show_state(model, data, "home keyframe")

        log = run_demo(model, data, headless=True)

        print("\n" + "═" * 60)
        print("  TASK COMPLETE — Pick-and-place sequence finished")
        print("═" * 60)
        print(f"\n  Total simulation time: {data.time:.2f}s")
        print(f"  Total logged entries:  {len(log)}")
        print(f"  Final chassis pos (m): x={data.qpos[0]:.3f}  "
              f"y={data.qpos[1]:.3f}  z={data.qpos[2]:.3f}")

        # Check if object moved
        # Red box freejoint starts at qpos index 18 (after 18 robot DoF)
        # Actually need to find it by joint name
        box_jnt = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "red_box_free")
        box_qadr = model.jnt_qposadr[box_jnt]
        box_pos = data.qpos[box_qadr:box_qadr + 3]
        print(f"  Red box final pos  (m): x={box_pos[0]:.3f}  "
              f"y={box_pos[1]:.3f}  z={box_pos[2]:.3f}")
        print()

    else:
        # Launch interactive viewer
        print("Launching viewer. Watch the pick-and-place sequence.\n")
        print("Controls:")
        print("  Left-click + drag  — rotate view")
        print("  Right-click + drag — pan view")
        print("  Scroll             — zoom")
        print("  Double-click body  — track it")
        print("  Esc                — quit\n")

        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                # Let it settle for a moment
                for _ in range(200):
                    mujoco.mj_step(model, data)
                    viewer.sync()

                dt = model.opt.timestep
                wheel_ids = [act_id(model, n) for n in
                             ["drive_FL", "drive_FR", "drive_RL", "drive_RR"]]
                yaw_id = act_id(model, "yaw_motor")
                grip_id = act_id(model, "gripper_pos")
                arm_names = ["shoulder_pos", "elbow_pos",
                             "wrist_roll_pos", "wrist_pitch_pos"]
                arm_ids = {n: act_id(model, n) for n in arm_names}

                prev_arm = {n: 0.3 if "shoulder" in n
                            else (-0.3 if "elbow" in n else 0.0)
                            for n in arm_names}
                prev_grip = 0.035

                for phase in PHASES:
                    if not viewer.is_running():
                        break
                    dur = phase["duration"]
                    n_steps = int(dur / dt)
                    print(f"Phase: {phase['name']}")

                    for step in range(n_steps):
                        if not viewer.is_running():
                            break
                        t = step / max(1, n_steps - 1)

                        for i, wid in enumerate(wheel_ids):
                            data.ctrl[wid] = phase["wheels"][i]
                        data.ctrl[yaw_id] = phase["yaw_torque"]
                        for aname in arm_names:
                            data.ctrl[arm_ids[aname]] = lerp(
                                prev_arm[aname], phase["arm"][aname], t)
                        data.ctrl[grip_id] = lerp(prev_grip, phase["gripper"], t)

                        mujoco.mj_step(model, data)
                        viewer.sync()
                        time.sleep(dt)  # real-time playback

                    prev_arm = dict(phase["arm"])
                    prev_grip = phase["gripper"]

                print("\nTask complete. Viewer remains open.")
                while viewer.is_running():
                    mujoco.mj_step(model, data)
                    viewer.sync()
                    time.sleep(dt)

        except Exception as e:
            print(f"\nViewer not available ({e}).")
            print("Run with --headless flag, or use mjpython.\n")
            sys.exit(1)


if __name__ == "__main__":
    main()
