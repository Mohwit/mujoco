# Capstone 2 (lesson 16): Warehouse Robot Pick-and-Place

> The interactive lesson is in `lessons/index.html` (Capstone 2, nav "C2"). It explains the open-loop script, the closed-loop state machine and its three inner loops, and where RL takes over the grasp, with numbers printed by `16_capstone_lesson.py`.

## Goal

See the full scope of what MuJoCo can simulate. A mobile manipulator performs a 13-phase pick-and-place task in a warehouse environment, demonstrating every concept from the curriculum.

## What this lesson covers

This single model uses **every major MuJoCo feature**:

| Feature | Where in the model |
|---|---|
| **Free joint** | Chassis (6-DoF mobility), graspable objects |
| **Hinge joint** | Wheels (spin), arm links (pitch/yaw/roll) |
| **Slide joint** | Gripper fingers (open/close along Y) |
| **Velocity actuator** | 4 wheel drives — ctrl = desired angular velocity (rad/s) |
| **Motor actuator** | Arm yaw turret — ctrl = raw torque (N·m) |
| **Position actuator** | Arm joints & gripper — ctrl = desired angle/position |
| **Fixed tendon** | Couples grip_L and grip_R joints |
| **Equality constraint** | Forces grip_L == grip_R (symmetric closing) |
| **Touch sensor** | Fingertip contact force (N) |
| **Accelerometer** | 3-axis IMU on chassis (m/s²) |
| **Gyroscope** | 3-axis IMU on chassis (rad/s) |
| **Rangefinder** | Forward-facing distance sensor (m) |
| **Joint position sensor** | All arm joints (rad) and gripper (m) |
| **Frame position sensor** | End-effector world position (m) |
| **Contacts & friction** | Wheels on floor, fingers on objects |
| **Keyframes** | Home pose and pre-grasp pose |
| **Procedural textures** | Checker floor, wood shelves, metal chassis |
| **Multiple cameras** | Overview, side, tracking, gripper-cam |
| **Default classes** | robot, wheel, finger, object, furniture |

## Model statistics

```
bodies:     21     joints:    14     geoms:     41
actuators:  10     sensors:   13     tendons:    1
equality:    1     keyframes:  2
nq:         32     nv:        29     nu:        10
nsensordata: 21
```

## The environment

A 10×10 m warehouse floor with:
- **4 boundary walls** (0.8 m tall)
- **Shelf A** (source) at (-1.5, 1.5) — two levels, 55 cm surface height
- **Shelf B** (destination) at (1.5, -1.0) — matching height
- **Table** obstacle at (0.5, 0.5)
- **Crates** stacked at (-3, -2)
- **Pylons** marking the aisle at (±0.8, -1)
- **2 graspable objects** on Shelf A: a red box (0.25 kg) and a blue cylinder (0.15 kg)
- **Target marker** on Shelf B (semi-transparent green)

## The robot

A mobile manipulator with 14 joints and 10 actuators:

```
chassis (freejoint, 8 kg)
├── wheel_FL  (hinge, velocity actuator)
├── wheel_FR  (hinge, velocity actuator)
├── wheel_RL  (hinge, velocity actuator)
├── wheel_RR  (hinge, velocity actuator)
└── arm_base / turret  (hinge Z, motor actuator)
      └── upper_arm  (hinge Y, position actuator, kp=80)
            └── forearm  (hinge Y, position actuator, kp=60)
                  └── wrist  (hinge Z, position actuator, kp=20)
                        └── wrist_tilt  (hinge Y, position actuator, kp=20)
                              ├── finger_L  (slide -Y, position actuator, kp=200)
                              └── finger_R  (slide +Y, coupled via tendon)
```

**Collision filtering:** The chassis geoms have `contype=0`, so they never touch the floor. The wheels (high friction, `condim=6`), the arm links, and the gripper fingers (very high friction, `condim=4`) all collide with furniture and objects, so the arm cannot pass through a shelf.

## The task: 13-phase pick-and-place

The robot starts at (0, -1.5) facing north (+Y). Shelf A is at (-1.5, 1.5), Shelf B at (1.5, -1.0).

| Phase | Action | Wheels | Arm |
|---|---|---|---|
| 01 | Settle, open gripper | Stopped | Shoulder 0.3, Elbow -0.3 |
| 02 | Drive north toward Shelf A | All at 8 rad/s | Hold |
| 03 | Turn left 90° to face west | Differential: -5/+5 | Hold |
| 04 | Drive west to Shelf A | 5/6 rad/s (curve left) | Hold |
| 05 | Extend arm toward object | Stopped | Shoulder 1.5, Elbow -1.2 |
| 06 | Close gripper (grasp) | Stopped | Hold; grip→0 |
| 07 | Lift object | Stopped | Shoulder 0.8, Elbow -0.5 |
| 08 | Reverse away from shelf | All at -6 rad/s | Tuck arm |
| 09 | Turn right 180° to face east | Differential: +6/-6 | Hold |
| 10 | Drive SE toward Shelf B | 8/5 rad/s (curve right) | Hold |
| 11 | Lower arm to placement | Stopped | Shoulder 1.4, Elbow -1.0 |
| 12 | Release object | Stopped | Hold; grip→0.035 |
| 13 | Retract arm to home | Stopped | Shoulder 0.3, Elbow -0.3 |

This scripted (open-loop) version uses **linear interpolation** between hand-picked joint angles and fixed wheel speeds for fixed times: no sensors, no inverse kinematics, no learning. It drifts (its "90 degree" turn really turns about 31 degrees) and never reaches the box. The closed-loop version below (`16_capstone_closed_loop.py`) adds sensor feedback and analytic inverse kinematics, and can hand the grasp to a reinforcement-learning policy (`--policy`).

## Running the script

```bash
# Headless mode — prints sensor state at each phase
uv run --python .venv/bin/python lessons/16-capstone-warehouse-robot/16_capstone_warehouse_robot.py --headless

# Interactive viewer — watch the robot in 3D (requires mjpython)
.venv/bin/mjpython lessons/16-capstone-warehouse-robot/16_capstone_warehouse_robot.py
```

## Reading the output

Every printed value includes its physical meaning and unit:

```
  [end of phase]  t=5.00s
    chassis world pos (m):  x=1.874  y=-3.000  z=0.110
    end-effector pos  (m):  x=1.942  y=-3.000  z=0.629
    gripper opening   (m):  0.0350
    touch sensors     (N):  L=0.00  R=0.00
    rangefinder       (m):  2.317
```

- **chassis world pos**: Center of the chassis body in world frame (meters)
- **end-effector pos**: The `ee_site` position — where the gripper is in space
- **gripper opening**: Distance each finger has traveled from closed (0) to open (0.04 m)
- **touch sensors**: Normal contact force on each fingertip (Newtons, 0 = no contact)
- **rangefinder**: Distance to nearest obstacle in front of the chassis (-1 = no hit within range)

## Interactive dashboard

Open `16-capstone-warehouse-robot.html` in a browser for an interactive visualization:
- **Top-down floor plan** showing the robot driving through the warehouse
- **Sensor dashboard** with live joint angles, gripper state, touch forces, IMU, and rangefinder
- **Phase timeline** with click-to-jump navigation
- **Playback controls** with adjustable speed

## Closed-loop controller

The scripted demo (`16_capstone_warehouse_robot.py`) uses open-loop control — fixed wheel speeds for fixed durations. It drifts because there is no sensor feedback.

`16_capstone_closed_loop.py` replaces the blind phases with a **feedback controller**:

| Feature | Open-loop (scripted) | Closed-loop |
|---|---|---|
| Navigation | Fixed wheel speeds × fixed time | PD heading control + distance proportional |
| Phase duration | Hardcoded seconds | Runs until goal condition met |
| Heading correction | None (blind) | Gyroscope derivative damping |
| Arm positioning | Linear interpolation over time | Waits for joint sensor convergence |
| Grasp confirmation | Assumed after timer | Checks touch sensor threshold |

### How the navigation works

1. Read chassis position and heading from `data.xpos` and `data.xmat`
2. Compute heading error: `desired_heading - current_heading`
3. Read yaw rate from gyroscope sensor for derivative damping
4. PD turn command: `turn = Kp * heading_error - Kd * yaw_rate`
5. Speed scales with distance, gated by heading alignment
6. Differential drive: left wheels get `speed - turn`, right wheels get `speed + turn`

### Running the closed-loop controller

```bash
# Headless mode
uv run --python .venv/bin/python lessons/16-capstone-warehouse-robot/16_capstone_closed_loop.py --headless

# Interactive viewer
.venv/bin/mjpython lessons/16-capstone-warehouse-robot/16_capstone_closed_loop.py
```

### Why the old arm could not pick the box

The robot drove to the shelf, swung its arm, closed the gripper on air, and drove away empty. Three physical causes:

1. **Too short.** The upper arm was 22 cm and the forearm 18 cm. The chassis cannot drive under the shelf, so the turret stopped about 45 cm from the box. The arm could only reach about 27 cm forward.
2. **Elbow bent the wrong way.** The elbow range was `-2.5 … 0.1` rad, so it could only fold backward. To reach a box on a shelf, the elbow has to sit *above* the shelf edge with the forearm sloping down onto the box. A backward-only elbow pushes the forearm up into the underside of the shelf top.
3. **Hand-picked joint angles.** `ARM_REACH = {shoulder 1.2, elbow -0.8}` was a guess. Nothing computed where the gripper would actually end up.

### What changed in the model (`warehouse_robot.xml`)

| Change | Before | After | Why |
|---|---|---|---|
| Upper arm length | 0.22 m | **0.30 m** | reach |
| Forearm length | 0.18 m | **0.26 m** | reach |
| Elbow range | -2.5 … 0.1 rad | **-0.3 … 2.6 rad** | the elbow can now bend forward, over the shelf edge |
| Turret position | chassis centre | **12 cm forward** | starts closer to the shelf |
| Fingers | 5 cm long, open 4 cm | **7 cm long, open 5 cm** | wraps more of the 7 cm box |
| Touch sites | 6 mm sphere | **thin box over the whole inner face** | registers contact anywhere along the finger |
| `grasp_site` + `grasp_pos` sensor | — | **new** | the point IK and RL aim at, midway along the fingers |
| Shoulder / elbow kp | 80 / 60 | **200 / 150** | stiffer servos for the longer arm |
| Gripper kp | 200 | **600** | ~8 N squeeze, so the box no longer slips out while driving |
| `impratio` | 1 | **10** | stiffer friction in the elliptic cone, so a held object slides less |
| `<contact><exclude>` | — | forearm↔wrist/fingers | stops the arm colliding with itself when the wrist bends |
| Box, cylinder, target | 35–40 cm deep on the shelf | **12 cm from the front edge** | where a mobile arm can reach |
| Pylons | in the driving lane | moved to (±2.2, 0.3) | they sat in the robot's path |

### Inverse kinematics instead of guessed joint angles

`warehouse_common.arm_ik(point, pitch_down)` solves analytically for the joint angles that put `grasp_site` at a 3D point in the chassis frame:

1. **Yaw** points the arm plane at the target: `yaw = atan2(y, x)`.
2. **Back off** from the grasp point along the gripper direction by `L_GRIP = 6 cm` to get the wrist joint position.
3. **Two-link law of cosines** gives the elbow angle: `cos(elbow) = (d² − L₁² − L₂²) / (2 L₁ L₂)`. We take the positive (elbow-forward) solution.
4. **Shoulder** = angle to the wrist minus the angle the forearm adds.
5. **Wrist pitch** = desired gripper angle − shoulder − elbow.

All angles are measured from vertical: 0 means the link points straight up, and positive means it tilts forward. The gripper is tilted 0.3 rad (17°) down, which keeps the forearm clear of the shelf edge.

The controller moves the grasp point along **straight lines in world space** (`CartesianMove`) near the shelf, and solves IK every step. A joint-space move between the same two poses would swing the gripper through the shelf.

Two more control details:
- **Gravity compensation.** Every step, `data.qfrc_applied[arm dofs] = data.qfrc_bias[arm dofs]`. The servos then only correct errors and no longer hold up the arm's weight. Real arm firmware does the same.
- **Yaw PD loop.** The turret has a raw torque motor, so `ArmDriver` closes a PD loop on it.

### New phase sequence

`SETTLE → DRIVE_TO_SHELF_A → ALIGN → READY → DOCK → PRE_GRASP → APPROACH → GRASP → LIFT → CARRY → BACK_OFF → DRIVE_TO_SHELF_B → ALIGN → DOCK → PLACE_ABOVE → PLACE_DOWN → RELEASE → RETREAT → BACK_OFF → TUCK`

- **READY** raises the gripper above shelf height *before* docking. The old tucked pose pushed the gripper under the shelf top, and the arm then shoved the robot backwards when it lifted.
- **DOCK** drives straight along the heading until the chassis is 1 cm from a fixed x. Navigation alone stops anywhere within 10 cm.

### Result

```
✓ GRASP   grasped (L=7.8N R=8.0N)
Red box final pos (m): x=1.221  y=-0.997  z=0.595
Distance to target (m, horizontal): 0.003
SUCCESS: True
```

The robustness test randomised the start position by ±15 cm and the box by ±4–5 cm. It succeeded 10/10 times, with every final error ≤ 9 mm.

## Reinforcement learning: learning the grasp

The classical controller now works. RL is still worth learning, because the same approach scales to cases a hand-written IK script cannot handle. Examples are cluttered shelves, unknown object shapes, and noisy perception.

### What RL learns, and what it does not

| Part of the task | Who does it | Why |
|---|---|---|
| Drive, align, dock | classical PD controller | already reliable; learning it would cost millions of samples |
| **Reach, grasp, lift** | **SAC policy** | the hard, contact-rich part |
| Carry, place, release | classical IK | |

Most real mobile manipulators split the work the same way: classical control for the easy parts, learned control for the hard part.

### Files

| File | Role |
|---|---|
| `warehouse_common.py` | shared IK, gravity compensation, sensor helpers |
| `warehouse_grasp_env.py` | `WarehouseGraspEnv`, a Gymnasium environment, plus `GraspPolicyRunner`, which runs a trained policy inside the state machine |
| `train_grasp_rl.py` | trains SAC (stable-baselines3) and evaluates the success rate |
| `export_trajectory.py` | records a real MuJoCo run into the HTML dashboard |

### The environment (`WarehouseGraspEnv`)

- **Reset:** the robot is docked at shelf A with random parking error (±4 cm, ±6 cm, ±4.6°). The box is placed with ±3–4 cm of random error. This **domain randomisation** stops the policy from memorising a single pose.
- **Observation (19 numbers, chassis frame):** grasp point, box position, box − grasp, gripper opening, the two touch forces, lift height, 4 arm joint angles, and the parking error.
- **Action (4 numbers in [-1, 1]):** move the grasp-point target by up to 1.5 cm per step in x/y/z, plus a gripper command. This is a *task-space* action: IK turns it into joint targets. Learning raw joint torques also works, but it needs roughly 10× more data.
- **Reward:** `reach = 1 − tanh(10·dist)`, plus `+0.5` while both fingers touch the box, plus `+2·clip(lift/10cm)` while grasped. Knocking the box away gives −5 and ends the episode. **Success** means the box is held more than 8 cm up. The episode keeps running after success, so the policy must keep holding the box.
- **Rate:** 20 Hz, 25 physics steps per action, 100 steps (5 s) per episode.

### Reward hacking: a real bug from training

The first training run used the **touch sensors** to decide "grasped". SAC found a loophole. A touch sensor reports force from *any* contact inside its site, so the policy pressed the open fingers onto the **shelf top**. It collected the grasp bonus every step and never went near the box. The reward curve plateaued at ~70, which looked like progress.

The fix was to check the actual contact pairs (`finger_L_geom ↔ red_box_geom` and `finger_R_geom ↔ red_box_geom`) in `data.contact`, and to penalise fingers touching the shelf. **Prevention rule:** reward the physical event you want (finger touching box), not a sensor reading that only correlates with it.

The second run found a different loophole. It grasped the box, lifted it **5 cm**, and hovered there for the rest of the episode. Success *ended* the episode with a +10 bonus. Hovering, though, paid ~2.4 per step, and with discount γ = 0.98 that stream is worth ~2.4 / (1 − 0.98) ≈ 120. Finishing was worth only +10, so SAC correctly chose never to cross the 8 cm line. The fix was to stop ending the episode on success and to let the lift reward saturate just past the goal. **Prevention rule:** never make success end the episode if staying alive is worth more than the success bonus.

### Train, evaluate, deploy

```bash
# 1. Install RL libraries (one time)
uv pip install --python .venv/bin/python gymnasium stable-baselines3 tensorboard

# 2. Train (~20 min on an M-series laptop CPU for 200k steps)
uv run --python .venv/bin/python lessons/16-capstone-warehouse-robot/train_grasp_rl.py --steps 200000

# 3. Watch learning curves (optional)
.venv/bin/tensorboard --logdir lessons/16-capstone-warehouse-robot/rl_runs

# 4. Success rate on 50 unseen random episodes
uv run --python .venv/bin/python lessons/16-capstone-warehouse-robot/train_grasp_rl.py \
    --eval lessons/16-capstone-warehouse-robot/models/grasp_sac_best/best_model.zip

# 5. Full warehouse task with the learned grasp
.venv/bin/mjpython lessons/16-capstone-warehouse-robot/16_capstone_closed_loop.py \
    --policy lessons/16-capstone-warehouse-robot/models/grasp_sac_best/best_model.zip
```

A pre-trained policy is saved at `models/grasp_sac_pretrained.zip`, so you can skip step 2.

### Results

| | Classical IK grasp | RL (SAC) grasp |
|---|---|---|
| Grasp env, 30 unseen random episodes | — | **30/30 (100%)** |
| Full task, 10 randomised runs | 10/10 | **10/10** |
| Time for reach → grasp → lift | ~4.5 s (4 phases) | **~1.0 s** (1 phase) |
| Final placement error | ≤ 9 mm | 16–29 mm |
| Training cost | 0 (hand-derived maths) | ~120k steps ≈ 10 min CPU |

The RL policy is about 4× faster because it moves the gripper diagonally straight into the grasp, instead of going through hand-chosen waypoints. It is less precise at placing, because it grips the box off-centre and the classical placement phases assume a centred grip. One way to improve this: add "box offset inside the gripper" to the placement IK, or train a second policy for placing.

### Prediction exercises (RL)

1. **Remove domain randomisation.** Set every `rng.uniform` range in `reset()` to 0 and retrain. Predict: will the training success rate go up or down? What happens when you then run the policy in the full task, where the parking error is real?
2. **Shrink the action step** from 1.5 cm to 0.5 cm. Predict how the episode length needed for success changes, and whether 100 steps is still enough.
3. **Raise the box mass** to 1 kg in the XML *without* retraining. Predict which fails first: the grasp (contact) or the lift (servo strength). Then check `info["lift"]`.

## Interactive dashboard (recorded physics)

The HTML page no longer re-simulates the robot in JavaScript, which is why it used to disagree with MuJoCo. `export_trajectory.py` runs the real controller and embeds every frame (20 Hz) into the page. The page also has a new **arm side view** that shows the shelf, the box, and the arm links. You can watch the elbow come over the shelf edge.

```bash
uv run --python .venv/bin/python lessons/16-capstone-warehouse-robot/export_trajectory.py            # classical run
uv run --python .venv/bin/python lessons/16-capstone-warehouse-robot/export_trajectory.py --policy …  # RL run
```

Note: the original open-loop script `16_capstone_warehouse_robot.py` uses joint angles tuned for the *old* short arm. It still runs, but it will not grasp with the new geometry. Keep it as the "before" example.

## Exercises

1. **Change the object mass.** In `warehouse_robot.xml`, change the red box mass from `0.25` to `2.0` kg. Does the arm still lift it? What happens to the chassis stability?

2. **Increase wheel friction.** Change the wheel class friction from `1.5` to `3.0`. Does the robot drive straighter? Faster?

3. **Remove the tendon equality.** Delete the `<equality>` block. What happens to the gripper fingers when you close them?

4. **Add a third object.** Add another `<body>` with a freejoint and a sphere geom on Shelf A. How do `nq` and `nv` change?

5. **Change a position actuator to a motor.** Replace `shoulder_pos` (position) with a `<motor>` actuator. How does the arm behave differently? (Hint: you now control torque, not target angle.)

6. **Reduce the timestep.** Change from 0.002 to 0.01. Does the simulation stay stable? What is the robot's behavior at 0.02?

## Concepts demonstrated

This model connects to every lesson in the curriculum:

| Lesson | Concept | Where it appears here |
|---|---|---|
| 1 | MjModel, MjData, mj_step | Script loads model, steps through 14,250 timesteps |
| 2 | MJCF body tree, nq/nv counting | 21-body tree with mixed joint types |
| 3 | mj_forward vs mj_step, state arrays | Position actuators need mj_step to servo |
| 4 | mj_name2id, xpos, cameras | Named lookup for all actuators and sensors |
| 5 | Actuator types | All three types: velocity, motor, position |
| 6 | Sensors, sensordata | 12 sensors across 18 sensordata dimensions |
| 7 | Contacts, friction, condim | Wheel-floor and finger-object contacts |
| 8 | Controllers | Waypoint interpolation + position servo gains |
| 9 | Environment loop | Headless step loop with ctrl, step, observe pattern |

## Next steps

With these fundamentals, you can:

- **Add reinforcement learning** with [Gymnasium](https://gymnasium.farama.org/) — wrap this model in a `gym.Env` with observations from sensors and rewards for placing the object
- **Use dm_control** for more complex pre-built robots (humanoids, quadrupeds, manipulators)
- **Try trajectory optimization** with [MuJoCo MPC](https://github.com/google-deepmind/mujoco_mpc) — compute optimal trajectories instead of scripted waypoints
- **Build for sim-to-real** — tune contact parameters to match a real robot, then transfer learned policies
