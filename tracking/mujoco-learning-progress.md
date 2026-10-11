# MuJoCo learning progress

Environment: `.venv`; MuJoCo installed; native viewer uses `mjpython`.

| Topic | Status | Evidence / notes |
|---|---|---|
| Orientation and setup | introduced | Ran `hello_mujoco.py` with `mjpython`. |
| MJCF structure | introduced | Reviewed body, geom, joint, hierarchy, and frames; Lesson 2 checkpoint pending. |
| State and stepping | introduced | Lesson 3 artifacts added; checkpoint pending. |
| Inspection and visualization | not started | Lesson 4 built (two-link arm, names/ids, xpos/xquat/xipos, cameras); not attempted yet. |
| Actuators and control | not started | Lesson 5 built (motor gear 100, position, velocity on the course pendulum); not attempted yet. |
| Sensors and observation | not started | Lesson 6 built (5 sensors, sensordata slices, obs = [cos, sin, qvel]); not attempted yet. |
| Contact and constraints | not started | Lesson 7 built (contype/conaffinity, contact forces, ramp friction, margin); not attempted yet. |
| Controllers and experiments | not started | Lesson 8 built (PD from scratch, gain sweep, gravity comp, timestep stability, CSV log); not attempted yet. |
| Environments and next steps | not started | Lesson 9 built (PendulumHoldEnv on gymnasium.Env, policy returns, where next); not attempted yet. |
| Capstone 1: cart-pole (slot 10) | not started | Built: cart-pole from scratch, full-state (LQR) balance, CartPoleBalanceEnv; not attempted yet. |
| Building bigger models (11) | not started | Built: default classes, keyframes, include, MjSpec; not attempted yet. |
| Kinematics and Jacobians (12) | not started | Built: FK chain, mj_jacSite, analytic and DLS IK on the lesson 4 arm; not attempted yet. |
| Grippers and grasping (13) | not started | Built: tendon + equality fingers, squeeze rule 2 mu N >= m g, touch vs contact pairs; not attempted yet. |
| Mobile bases (14) | not started | Built: free-joint yaw, skid steer (effective track width ~4x), gyro, rangefinder, open vs closed loop; not attempted yet. |
| Learning a policy (15) | not started | Built: SAC on PendulumHoldEnv, learning curve, reward variants, a real reward hack; not attempted yet. |
| Capstone 2: warehouse robot (16) | not started | Moved into index.html: state machine, nav/IK/gripper loops, RL grasp env and both reward hacks; not attempted yet. |

## Learning aids

- 2026-10-11: Added beginner explainers for Lessons 1–3 (`lessons/NN-*/NN-*-explained.html`): plain words → pictures → MuJoCo terms → optional maths → code, with interactive animations. No status change: all three checkpoints are still pending.
- 2026-10-11: Merged the explainer content into `lessons/index.html` (each lesson now reads explainer sections → recap → checkpoint). The standalone `*-explained.html` files are kept unchanged.

- 2026-10-11: Built Lessons 4-9 into `lessons/index.html` (fragments in `lessons/NN-topic/_fragments/`, assembled by `lessons/_build/build.py`; plan in `lessons/_build/CURRICULUM-PLAN.md`). All share the course pendulum and the Lesson 5 motor (gear 100, ctrlrange -1 1). Statuses unchanged until checkpoints are answered.

- 2026-10-11: Added Part 2 to `lessons/index.html`: Capstone 1 (slot 10), lessons 11-15, Capstone 2 (slot 16, the warehouse robot, folder renamed `10-` to `16-capstone-warehouse-robot`). Plan in `lessons/_build/CURRICULUM-PLAN.md`. Statuses unchanged until checkpoints are answered.

## Next lesson

Complete Lesson 3 checkpoint: predict how `mj_forward`, `mj_step`, and `mj_resetData` affect `qpos`, `qvel`, `qacc`, and `time`.
