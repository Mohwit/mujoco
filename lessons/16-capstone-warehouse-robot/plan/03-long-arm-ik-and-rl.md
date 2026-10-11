# Plan: Longer Arm, IK Grasping, and RL Grasp Policy

## Context

After the closed-loop controller (plan 02), the robot navigated well but never picked up the box. It drove to shelf A, closed the gripper on air, and left empty-handed. The user asked for two things:
1. Fix the short arm so the robot can actually pick and place.
2. Add reinforcement learning to the simulation.

## Diagnosis

| Symptom | Physical cause |
|---|---|
| Gripper stops ~20 cm short of the box | Arm was 22 + 18 cm; turret parked ~45 cm from the box |
| Arm hits the shelf underside when reaching | Elbow could only bend backward (range −2.5…0.1) |
| Arm pose wrong even when in range | Joint angles were hand-guessed, with no kinematics |
| Robot shoved backwards during the reach | Tucked gripper sat under the shelf top while docking, then the lift pushed against the shelf |
| Box slipped out while driving | ~3 N grip force; soft friction (impratio 1) |

## Approach

### 1. Model (`warehouse_robot.xml`)
- Upper arm 0.30 m, forearm 0.26 m, and the elbow range flipped to −0.3…2.6 (bends forward).
- Turret moved 12 cm forward on the chassis.
- Longer fingers (7 cm), wider opening (5 cm), box-shaped touch sites, a new `grasp_site` and `grasp_pos` sensor.
- Higher servo gains, gripper kp 600, `impratio=10`.
- `<contact><exclude>` for arm self-collision.
- Objects and target moved to 12 cm from the shelf front edge. Pylons moved out of the lane.

Verified offline: IK solutions for pre-grasp, grasp, and lift were checked for joint limits *and* zero arm–shelf contacts. Parking x was scanned over −0.80…−0.90 and gripper pitch over 0…0.9 rad. Pitch 0.3 rad was the only value collision-free at every parking distance.

### 2. Shared module (`warehouse_common.py`)
- `arm_ik(point_in_chassis, pitch_down)`: analytic yaw + 2-link + wrist solution.
- `ArmDriver`: writes the position targets, runs a PD loop on the yaw torque motor, and applies gravity compensation via `qfrc_applied = qfrc_bias`.

### 3. Classical controller (`16_capstone_closed_loop.py`, rewritten)
- One `PickPlaceController.step()`, shared by headless and viewer modes (the old code duplicated the state machine).
- New phases: READY (raise the gripper before docking), DOCK and BACK_OFF (straight-line position control), PRE_GRASP, APPROACH, LIFT, CARRY, PLACE_ABOVE, PLACE_DOWN, RETREAT (straight Cartesian lines + IK), TUCK (joint-space move).
- A minimum turn rate in `align_step` stops skid-steer stalling near the tolerance.
- Result: 3 mm placement error; 10/10 success with ±15 cm start and ±4 cm box randomisation.

### 4. RL (`warehouse_grasp_env.py`, `train_grasp_rl.py`)
- Gymnasium env for the manipulation subtask only. The classical controller still drives.
- Task-space action (Δ grasp point + gripper), 19-D observation, dense staged reward, domain randomisation.
- SAC from stable-baselines3, 200k steps, with an EvalCallback that keeps the best model.
- `GraspPolicyRunner` plugs the policy into the state machine as a single `RL_GRASP` phase (`--policy PATH`).

### 5. Dashboard (`export_trajectory.py`)
- The HTML replays a recorded MuJoCo run instead of re-simulating in JS (the root cause of the earlier "HTML trajectory is wrong" bugs).
- New arm side view drawn from recorded joint angles.

## Lessons learned
- **Check reachability before tuning controllers.** No gain tuning could fix an arm that was 20 cm too short.
- **Collision-check IK solutions offline** (set qpos → `mj_forward` → read `data.contact`) before running dynamics.
- **Watch for reward hacking.** The first RL run rewarded touch-sensor force, so the policy pressed the fingers on the shelf. Reward the actual finger↔box contact.
- **Don't end the episode on success if staying alive pays more.** Run two lifted the box 5 cm and hovered: ~2.4/step for the rest of the episode beat a one-time +10. Removing success termination fixed it.
- **Touch sensors measure any contact** inside the site volume, not just contact with the object you care about.
- **The `mjOBJ` type matters:** `mj_id2name(m, 1, geom_id)` looks up a *body* name. Use `mjtObj.mjOBJ_GEOM`.
