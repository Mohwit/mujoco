# Plan: Closed-Loop Controller for Warehouse Robot

## Context

The current capstone robot (lesson 10) uses an **open-loop scripted trajectory** — fixed wheel speeds for fixed durations. It drifts because there's no feedback. The user wants the robot to actually navigate to targets and perform the pick-and-place reliably.

We'll replace the blind scripted phases with a **closed-loop controller** that reads the robot's sensors (chassis position from qpos, IMU, end-effector position, touch sensors) and steers toward waypoints in real time.

## Approach: Waypoint-Following PID Controller

Instead of "drive wheels at 8 rad/s for 5 seconds," the controller will:
1. Know the target position (e.g., Shelf A at -1.5, 1.5)
2. Read the robot's current position and heading from `data.qpos[0:7]`
3. Compute heading error and distance error
4. Use proportional control to set wheel speeds (turn toward target, drive forward when aligned)
5. Stop when within tolerance of the target

This is a **state machine** with the same phases (drive to shelf, extend arm, grasp, etc.), but each navigation phase uses feedback instead of blind timing.

## What gets built

### 1. `16_capstone_closed_loop.py` (~350 lines) — New script

**Navigation controller:**
- Extract chassis (x, y) from `data.qpos[0:2]`
- Extract heading from chassis quaternion `data.qpos[3:7]` → yaw angle
- Compute `heading_error = atan2(target_y - y, target_x - x) - yaw`
- Compute `distance = sqrt((target_x - x)² + (target_y - y)²)`
- Wheel commands: `turn_rate = Kp_turn * heading_error`, `speed = Kp_speed * distance` (clamped)
- Differential drive: `left_wheels = speed - turn_rate`, `right_wheels = speed + turn_rate`

**Arm controller:**
- Position actuators already servo to targets (kp built into XML)
- Set target angles directly, wait for convergence (check joint sensor vs target)

**Grasp controller:**
- Close gripper, check touch sensors for confirmation
- If touch_L + touch_R > threshold → grasp confirmed

**State machine phases:**
1. `DRIVE_TO_SHELF_A` — navigate to (-1.2, 1.5), stop when distance < 0.15m
2. `ALIGN_TO_SHELF` — turn to face shelf (heading toward -X)
3. `EXTEND_ARM` — set shoulder/elbow targets, wait for convergence
4. `GRASP` — close gripper, wait for touch sensor confirmation
5. `LIFT` — raise arm, verify object still grasped (touch > 0)
6. `DRIVE_TO_SHELF_B` — navigate to (1.2, -1.0)
7. `ALIGN_TO_SHELF_B` — turn to face shelf B
8. `PLACE` — lower arm to shelf height
9. `RELEASE` — open gripper
10. `RETRACT` — tuck arm home

**Key differences from open-loop:**
- Navigation phases have no fixed duration — they run until the goal condition is met
- Arm phases check joint sensor convergence instead of waiting a fixed time
- Grasp phase confirms contact via touch sensors
- The robot self-corrects if it drifts off course

**Output:**
- Same headless/viewer dual mode as the original script
- Prints state at each phase transition with sensor readings
- Reports success/failure for each phase (reached target? grasped object? placed successfully?)

### 2. Update `16-capstone-warehouse-robot.md`

Add a new section documenting the closed-loop controller, how it works, and how to run it.

## Key implementation details

**Quaternion → yaw extraction:**
```python
# MuJoCo quaternion is [w, x, y, z]
# For a chassis on a flat floor, yaw = atan2(2(wz + xy), 1 - 2(y² + z²))
# But simpler: use the rotation matrix from mj_forward
# data.xmat for chassis body gives 3x3 rotation, forward direction is first column
```

Actually, the most robust approach:
```python
chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
xmat = data.xmat[chassis_id].reshape(3, 3)
forward = xmat[:, 0]  # local +X in world frame
yaw = np.arctan2(forward[1], forward[0])
```

**Wheel command mapping:**
The robot's local +X is forward. With quat (0.707, 0, 0, 0.707), local +X maps to world +Y.
- Left wheels: FL, RL (positive Y side in local frame)
- Right wheels: FR, RR (negative Y side)
- Turn left (CCW): left wheels slower, right wheels faster
- `ctrl[FL] = ctrl[RL] = base_speed - turn_correction`
- `ctrl[FR] = ctrl[RR] = base_speed + turn_correction`

**Convergence checks:**
- Navigation: `distance_to_target < 0.15 m`
- Arm positioning: `|sensor_value - target| < 0.05 rad` for all joints
- Grasp: `touch_L > 0.1 N AND touch_R > 0.1 N`
- Timeout: max 15 seconds per phase to prevent infinite loops

**Gains (starting values, will tune):**
- `Kp_heading = 8.0` (rad/s per radian of heading error)
- `Kp_distance = 6.0` (rad/s per meter of distance)
- `max_wheel_speed = 10.0` rad/s
- `arrival_threshold = 0.15` m

## Files modified

| File | Action |
|------|--------|
| `lessons/16-capstone-warehouse-robot/16_capstone_closed_loop.py` | **Create** — new closed-loop controller script |
| `lessons/16-capstone-warehouse-robot/16-capstone-warehouse-robot.md` | **Edit** — add closed-loop section and run commands |

## Verification

1. Run headless: `uv run --python .venv/bin/python lessons/16-capstone-warehouse-robot/16_capstone_closed_loop.py --headless`
2. Check that the robot reaches Shelf A (x ≈ -1.2...-1.5, y ≈ 1.0...1.5)
3. Check that touch sensors fire during grasp (> 0 N)
4. Check that robot reaches Shelf B (x ≈ 1.2...1.5, y ≈ -0.7...-1.0)
5. Check that red box final position is near Shelf B target
6. Run with viewer: `.venv/bin/mjpython lessons/16-capstone-warehouse-robot/16_capstone_closed_loop.py`
