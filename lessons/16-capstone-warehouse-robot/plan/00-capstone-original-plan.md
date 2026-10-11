# Plan: Capstone Chapter 10 — Warehouse Robot Pick-and-Place

## Context

A motivational "final boss" lesson that showcases the full power of MuJoCo — a complex world with a robot performing a real task. This sits outside the normal curriculum (lessons 1-9) as a capstone preview of what's achievable.

## What was built

**Directory:** `lessons/16-capstone-warehouse-robot/`

### 1. `warehouse_robot.xml` (~23KB) — The MJCF world

**Environment:**
- 10x10m warehouse floor with checker-grid texture
- 4 boundary walls (0.8m tall)
- 2 shelf units with legs and surfaces (source and destination)
- 1 table obstacle, stacked crates (decoration), pylons marking the aisle
- 2 graspable objects (red box, blue cylinder) with free joints
- Target placement site marker (semi-transparent green)
- 2 directional lights, 4 cameras (overview, side, tracking, gripper-cam)
- Procedural textures/materials (no external files needed)

**Robot — mobile manipulator (21 bodies, 14 joints, 10 actuators):**
- Chassis with freejoint (7q/6v) — full 6-DOF base mobility
- 4 wheels (hinge joints + velocity actuators) — differential drive
- 5-DOF arm: yaw turret (motor actuator) → shoulder (position) → elbow (position) → wrist roll (position) → wrist pitch (position)
- 2-finger parallel gripper: slide joints, coupled via fixed tendon + equality constraint, position actuated
- 12 sensors: 6 jointpos, 2 touch, 3-axis accelerometer, 3-axis gyro, rangefinder, framepos

**Advanced features demonstrated:**
- All 3 actuator types (motor, position, velocity)
- Fixed tendon with equality constraint (gripper coupling)
- Contact dynamics with friction (wheels on floor, fingers on objects)
- 2 keyframes (home pose, pre-grasp pose)
- 5 default classes for shared geom/joint properties (robot, wheel, finger, object, furniture)
- Collision filtering via contype/conaffinity

### 2. `16_capstone_warehouse_robot.py` (~420 lines) — Open-loop scripted demo

13-phase scripted pick-and-place sequence using fixed wheel speeds and durations:
1. Settle → 2. Drive north → 3. Turn left → 4. Drive to Shelf A → 5. Extend arm → 6. Grasp → 7. Lift → 8. Reverse → 9. Turn 180° → 10. Drive to Shelf B → 11. Lower arm → 12. Release → 13. Retract

- Waypoint-interpolated trajectory (no RL/IK — pure scripted joint targets)
- Logs sensor readings at milestones
- Dual mode: `--headless` prints state, otherwise launches `mujoco.viewer.launch_passive`

### 3. `16_capstone_closed_loop.py` (~400 lines) — Closed-loop controller

Replaces blind scripted phases with sensor-feedback control:
- PD heading controller using gyroscope for derivative damping
- Proportional distance controller with alignment gating
- Arm convergence checking via joint position sensors
- Touch sensor grasp confirmation
- State machine with goal conditions per phase (not fixed durations)

### 4. `16-capstone-warehouse-robot.html` (~30KB) — Interactive dashboard

- Dark/light theme matching course site
- SVG top-down warehouse floor plan with animated robot path
- Sensor dashboard: joint angles, gripper/touch bars, IMU, rangefinder, wheel velocities
- Phase timeline with click-to-jump navigation
- Playback controls with adjustable speed
- JavaScript kinematic replay

### 5. `16-capstone-warehouse-robot.svg` (~8KB) — Kinematic chain diagram

- Robot body tree showing all bodies, joint types, actuator types, sensor placements
- Color-coded: teal=position actuators, orange=velocity actuators, red=motor

### 6. `16-capstone-warehouse-robot.md` (~8KB) — Lesson guide

- Goal, model statistics, environment description, robot structure
- 13-phase task description with action table
- Run commands for headless and viewer modes
- Output format explanation
- 6 exercises with predictions
- Feature mapping table connecting to every lesson in the curriculum

## Key design decisions

1. **XML body order matters for qpos**: Graspable objects placed AFTER the chassis body so that qpos[0:7] is always the chassis freejoint. This makes keyframe definitions readable.

2. **Collision filtering**: Robot chassis and arm geoms have `contype=0 conaffinity=0` to prevent collisions with the floor. Only wheels (high friction, condim=6) and gripper fingers (very high friction, condim=4) generate contacts.

3. **Tendon coupling**: A fixed tendon couples grip_L and grip_R so a single actuator controls both fingers symmetrically via an equality constraint.

4. **Three actuator types**: Velocity (wheels — ctrl=desired rad/s), Motor (arm yaw — ctrl=raw torque N·m), Position (arm joints + gripper — ctrl=desired angle/position with built-in servo).

5. **Robot orientation**: Quaternion (0.707, 0, 0, 0.707) rotates the chassis 90° about Z, making local +X point in world +Y direction (north toward Shelf A).

## What was learned during implementation

- Open-loop trajectories drift because wheel friction, chassis mass, and contact dynamics create unpredictable motion
- Proportional-only heading control causes spiraling on long journeys — derivative damping from the gyroscope is essential
- The arm's horizontal reach (~0.27m from turret) limits how far from shelves the robot can park and still grasp objects
- The chassis tilts when the arm extends horizontally (center of mass shifts), raising chassis z from 0.110 to ~0.124
- ARM_CARRY (tucked) pose is critical during transport — an extended arm during driving causes the robot to tip over
