# MuJoCo Learning Lab: curriculum plan (lessons 4-9)

Lessons 1-3 already exist in `lessons/index.html`. This plan covers lessons 4-9, which take the
learner from "I can step a pendulum" to "I can wrap a controlled, sensed robot in a Gym-style
environment". Part 2 (Capstone 1, lessons 11-15, Capstone 2) is planned at the end of this file.

## One running story

Every lesson reuses one small cast of models, so the learner never has to re-learn a model to
learn a new idea:

| Model | Where it comes from | Used in |
|---|---|---|
| **Course pendulum**: body `pendulum` at `pos="0 0 1"`, hinge `hinge` about `axis="0 1 0"`, capsule `fromto="0 0 0 0 0 -1" size="0.08"`, `timestep="0.002"`, gravity `0 0 -9.81` | Lesson 3 script | 5, 6, 8, 9 (each adds one thing: actuator, then sensors, then a controller, then a Gym wrapper) |
| **Two-link arm**: `upper_arm` and `forearm` on hinges `shoulder` and `elbow`, a small `hand` site, plus a free-floating `box` | New in lesson 4 | 4 (inspection, names, frames, cameras) |
| **Box on a ramp / ball on a floor** | New in lesson 7 | 7 (contacts, friction, contype/conaffinity) |

## Lessons

| # | Folder | Title | One-sentence goal | Big-idea analogy | Script output the page is built on | Sandbox |
|---|---|---|---|---|---|---|
| 4 | `04-inspection-and-visualization` | Inspection and Visualization | Find any part of a model by name, read where it is in the world, and take a picture of it without a GUI. | A phone contact list: you look people up by name, the phone dials a number (the ID). | `mj_name2id` ids for bodies/joints/geoms/sites, `xpos`, `xquat`, `xipos` vs `xpos` for an asymmetric link, `site_xpos` of the hand, camera list; optional PNG via `mujoco.Renderer` | Two-link arm: sliders for shoulder/elbow; click a body to see name, id, `xpos`, `xquat`, `xipos`; camera picker (fixed vs tracking) |
| 5 | `05-actuators-and-control` | Actuators and Control | Add a motor to the course pendulum, set `ctrl`, and predict the torque; compare motor, position and velocity actuators. | Motor = a hand pushing with a fixed strength; position actuator = a spring pulling to a target; velocity actuator = cruise control. | `nu`, `gear`, `ctrlrange`, `actuator_force` for motor/position/velocity at fixed ctrl, holding torque `m g d sin(theta)`, hinge angle after 1 s for each type | Pendulum with actuator-type picker, ctrl slider, gear slider; live torque arrow and `actuator_force` readout |
| 6 | `06-sensors-and-observation` | Sensors and Observation | Declare sensors in MJCF, read them from `sensordata` with the right slice, and build an observation vector. | A car dashboard: each gauge reads one thing, all gauges sit on one panel (`sensordata`). | `nsensor`, `sensor_adr`, `sensor_dim`, per-sensor values (jointpos, jointvel, actuatorfrc, framepos of the tip site, accelerometer), the assembled obs vector `[cos, sin, vel]` | Pendulum with motor; a "dashboard" that lights up the slice of `sensordata` each sensor owns; noise toggle |
| 7 | `07-contact-and-constraints` | Contact and Constraints | Predict which geoms collide, read `data.contact`, and fix sliding or penetration with friction, condim and margin. | Two magnets that only stick when their colour codes match (contype & conaffinity); sandpaper vs ice (friction). | Collision table from `contype`/`conaffinity`, `ncon`, contact `dist`/`pos`/`frame`, `mj_contactForce` normal force vs `m g`, box on ramp sticking below `atan(mu)` and sliding above it | Ramp: angle and friction sliders, box sticks or slides (uses trajectories recorded from MuJoCo or the friction-threshold rule, labelled) |
| 8 | `08-controllers-and-experiments` | Controllers and Experiments | Write a PD controller from scratch, tune its gains by experiment, and see why too large a timestep breaks it. | A shower tap: P turns towards the right temperature, D stops you overshooting. | Step response of the course pendulum to a 1.0 rad target for a few (kp, kd) pairs: rise time, overshoot, steady-state error; timestep sweep showing instability; CSV log written to the lesson folder | kp, kd, target, timestep sliders; live angle-vs-time plot that matches MuJoCo to 4 decimals |
| 9 | `09-environments-and-next-steps` | Environments and Next Steps | Wrap the controlled pendulum in a Gymnasium-style environment with `reset`, `step`, observation and reward, then know where to go next. | A video-game console: reset = new game, step = one button press, observation = the screen, reward = the score. | `PendulumSwingEnv` (pure Python class following the Gymnasium API; `gymnasium` 1.4 is installed so `gymnasium.Env` may be subclassed), episode returns for a random policy vs the lesson 8 PD policy with a fixed seed | A "game loop" diagram the learner drives: reset, step, obs, reward, done; random vs PD policy buttons with live return |

## Shared rules for every lesson

- Teaching ladder: Plain words, Picture, MuJoCo words, Maths (optional), Code, Check yourself, then Recap.
- Every number on the page comes from running the lesson script (MuJoCo 3.15.0, `.venv`).
- Every new term is a `<dfn class="term">` on first use and appears in the glossary.
- No em-dashes or en-dashes, no emoji, sentence-case labels, CSS colour variables only.
- Each lesson ends with "Mark lesson complete" and "Next: ..." (lesson 9's next opens Capstone 1).

## Build

Lessons 4-9 keep their course-site markup as fragments in `NN-topic/_fragments/` and are merged by
`python lessons/_build/build.py` into `lessons/index.html` (base: `lessons/_build/index.base.html`).
Check one lesson headlessly with `lessons/_build/check.cjs`.

# Part 2: Robots that move and grab

The course has three blocks. Every slot is a panel in `lessons/index.html`; the slot number is the
panel id (`lessonN`), the folder prefix and the fragment number.

| Slot | Shown as | Folder | Title |
|---|---|---|---|
| 1-9 | Lesson 01-09 | as above | Part 1: Foundations |
| 10 | Capstone 1 (nav num `C1`) | `10-capstone-cart-pole` | Capstone 1: Balance a Cart-Pole |
| 11 | Lesson 11 | `11-building-bigger-models` | Building Bigger Models |
| 12 | Lesson 12 | `12-kinematics-and-jacobians` | Kinematics and Jacobians |
| 13 | Lesson 13 | `13-grippers-and-grasping` | Grippers and Grasping |
| 14 | Lesson 14 | `14-mobile-bases` | Mobile Bases |
| 15 | Lesson 15 | `15-learning-a-policy` | Learning a Policy |
| 16 | Capstone 2 (nav num `C2`) | `16-capstone-warehouse-robot` (the existing warehouse folder, renamed from `10-`) | Capstone 2: Warehouse Robot |

Why this order: Capstone 1 proves Part 1 with a new model and nothing new to learn. Lessons 11-15
each teach one piece of the warehouse robot on a small model, so Capstone 2 has no unexplained part.

## Shared facts

- Two-link arm (lesson 4): `upper_arm` at `pos="0 0 1"`, hinge `shoulder` axis y, capsule to x=0.5 r 0.04;
  `forearm` at x=0.5, hinge `elbow`, capsule to x=0.4 r 0.03 plus box `weight` (half 0.05) at x=0.4;
  site `hand` at x=0.4. Lessons 11 and 12 reuse it.
- Lesson 9 env: `PendulumHoldEnv` in `09-environments-and-next-steps/09_environments_and_next_steps.py`,
  frame_skip 10, 250 steps, target 1.0 rad, reward `-(err^2 + 0.1 qvel^2 + 0.01 a^2)`. Lesson 15 trains on it.
- Warehouse robot (`16-capstone-warehouse-robot/warehouse_robot.xml`): default classes robot / wheel /
  finger / object / furniture; finger friction 2.0 condim 4; object friction 0.6; red box 0.25 kg;
  wheels velocity actuators; arm position actuators; fixed tendon + equality on the gripper; touch,
  gyro, accelerometer, rangefinder sensors; keyframes; analytic `arm_ik` in `warehouse_common.py`;
  closed-loop controller `16_capstone_closed_loop.py`; RL grasp env `warehouse_grasp_env.py` (SAC).
  Lessons 11-14 take their values from this file where it makes sense, and say so.
- `stable_baselines3` 2.9 and torch 2.14 are installed (CPU).

## Slots

| # | Goal | Big-idea analogy | Script output the page is built on | Sandbox |
|---|---|---|---|---|
| 10 | Build a cart-pole from scratch (slide + hinge, motor on the cart, sensors), balance it with full-state feedback, wrap it as a Gym env. Every step names the Part 1 lesson it uses. | Balancing a broom on your palm: you move your hand under the broom. | Model counts, mass matrix, open-loop fall time, gains, balance from 0.1 rad, recovery from a push, env returns for zero vs controller | Cart-pole with gain sliders, push button, tilt start; in-browser dynamics matching MuJoCo to 4 decimals |
| 11 | Write big models without repeating yourself: default classes, keyframes, `<include>`, and editing models from Python with `MjSpec`. | A style sheet for a document; save points in a game (keyframes). | Attribute resolution per geom, keyframe qpos, MjSpec-built shelf rows (counts), the warehouse model's own classes and keyframes | Default-class resolver (pick a geom, see which level each attribute comes from); keyframe picker posing the arm |
| 12 | Forward kinematics as a chain, the Jacobian (`mj_jacSite`), analytic two-link IK and Jacobian (damped least squares) IK. | Pointing at a cup: your brain picks shoulder and elbow angles for you. | FK vs `site_xpos`, Jacobian matrix at a pose, hand velocity = J qdot, IK solutions (elbow up/down), DLS iterations converging, singular pose | Drag a target; arm solves IK (analytic or DLS step by step); Jacobian readout and velocity arrows |
| 13 | Build a parallel-jaw gripper: slide fingers, tendon + equality coupling, touch sensors; compute and test the squeeze needed to lift a box. | Picking up a glass with two fingers: squeeze harder when hands are slippery. | Contact pairs, touch readings, squeeze sweep vs hold/slip, friction-rule prediction `2 mu N >= m g`, touch-sensor vs contact-pair check | Squeeze, mass and friction sliders; hold or slip, rule vs recorded MuJoCo outcome |
| 14 | Drive a wheeled base: free-joint pose and yaw from the quaternion, wheel velocity actuators, skid-steer kinematics, gyro and rangefinder, open-loop vs closed-loop waypoint driving. | Driving a car with your eyes closed (open loop) vs open (closed loop). | Yaw from qpos quaternion, measured v and turn rate vs wheel speeds, drift of open-loop runs, closed-loop arrival error | Top-down floor: pick a waypoint, compare open vs closed loop (calibrated kinematic model + recorded MuJoCo runs, labelled) |
| 15 | Train SAC on the lesson 9 env, read a learning curve, compare with PD, and see reward design and reward hacking. | Training a dog with treats: it learns what you reward, not what you meant. | Learning curve CSV, returns at checkpoints, SAC vs PD, a reward-design and a reward-hacking experiment with fixed seeds | Learning curve; replay recorded rollouts at checkpoints; reward-term picker |
| 16 | Put it all together: model tour, the pick-and-place task, open loop vs the closed-loop state machine, IK, and where RL takes over grasping. | A kitchen brigade: one chef per job, a head chef calling the order. | The closed-loop run (phases, durations, success), RL env spec, training and eval results, both reward-hacking stories | Top-down replay with phase timeline and sensor dashboard; scripted vs closed-loop runs |
