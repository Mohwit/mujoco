# Lesson 05: Actuators and control

> **Interactive lesson:** open `lessons/index.html` and choose **Lesson 5** (Actuators and Control). It covers the same ideas in plain language with pictures you can drive, a sandbox, and optional maths.

## Goal

Add a motor to the course pendulum, set `ctrl`, and predict the resulting torque. Explain the difference between motor (force), position and velocity actuators.

## Mental model

An actuator is a muscle at a joint. You write one number per actuator into `data.ctrl` before each step. MuJoCo turns it into a torque on the joint.

- `model.nu` is the number of actuators. `data.ctrl` has shape `(nu,)`.
- MuJoCo clips `ctrl` into `ctrlrange` (when `ctrllimited="true"`).
- `data.actuator_force[i] = gain * ctrl + bias0 + bias1 * length + bias2 * velocity` is the actuator's own output, before the gear.
- `data.qfrc_actuator[j] = gear * actuator_force` is the torque on the joint, in N m for a hinge.

| Type | XML | ctrl means | Torque on the hinge | Analogy |
|---|---|---|---|---|
| motor | `<motor gear="100"/>` | strength | `gear * ctrl` | a hand pushing with a set strength |
| position | `<position kp="500"/>` | target angle (rad) | `kp * (ctrl - qpos)` | a spring pulling to a target |
| velocity | `<velocity kv="200"/>` | target speed (rad/s) | `kv * (ctrl - qvel)` | cruise control |

Model is fixed, data is mutable: `gear`, `kp`, `kv`, `ctrlrange` live in `MjModel`. `ctrl`, `actuator_force`, `qfrc_actuator` live in `MjData`. You write `ctrl`; MuJoCo writes the forces.

## Runnable example

```bash
uv run --python .venv/bin/python lessons/05-actuators-and-control/05_actuators_and_control.py
```

The pendulum is the Lesson 3 model exactly. Three versions differ only in the `<actuator>` line:

```xml
<motor    name="motor"  joint="hinge" gear="100" ctrlrange="-1 1"       ctrllimited="true"/>
<position name="servo"  joint="hinge" kp="500"   ctrlrange="-1.57 1.57" ctrllimited="true"/>
<velocity name="cruise" joint="hinge" kv="200"   ctrlrange="-2 2"       ctrllimited="true"/>
```

Why these numbers: gravity's largest torque on this rod is `m g d = 109.14 N m` (rod sideways). Gear 100 with ctrl in -1..1 makes ctrl a throttle with a 100 N m maximum. The position range is a quarter turn each way; the velocity range is 2 rad/s each way.

The helper `05_check_browser_match.py` checks the page's in-browser stepping against `mj_step`.

## Inspectable output

```text
body_mass[1]           (pendulum mass, kg):            22.25085
I = mass matrix [0,0]     (inertia about hinge, kg m^2):  7.87639
m*g*d                     (max gravity torque, N m):      109.14044
data.actuator_force[0]    (actuator output, before gear): 0.500     # ctrl = 0.5
data.qfrc_actuator[0]     (torque on hinge, N m):         50.000    # gear 100 x 0.5
ctrl needed = gravity torque / gear:                      0.52325   # holds 0.5 rad
after 1 s: time=1.000 s  qpos=0.50000 rad  qvel=0.00e+00 rad/s
data.ctrl[0]=+1.50  actuator_force=+1.000  qfrc_actuator=+100.000 N m   # clipped
motor   t=1.000 s  qpos= 0.97690 rad   # ctrl 0.5 = strength
servo   t=1.000 s  qpos= 0.73142 rad   # ctrl 0.5 = target angle
cruise  t=1.000 s  qpos= 0.38133 rad   # ctrl 0.5 = target speed
```

Key facts:

- `actuator_force` is not the joint torque when `gear != 1`. Read `qfrc_actuator` for the torque.
- Holding still at angle `q` needs `gear * ctrl = m g d sin(q)`; at 0.5 rad that is 52.32471 N m, so ctrl = 0.52325.
- A position actuator droops below its target under gravity: with kp 500 and target 0.5 rad it balances at 0.41249 rad.
- MuJoCo's default Euler integrator treats only joint damping implicitly. The kp/kv actuator terms are explicit forces computed from the current state.

## Exercise: predict first

1. A motor has gear 100 and ctrlrange -1..1. You set `ctrl = 0.8`. What are `actuator_force[0]` and `qfrc_actuator[0]`?
2. Same motor, `ctrl = 1.5`. What does `data.ctrl[0]` read, and what torque reaches the hinge?
3. A position actuator has kp 500, ctrl 0.5, qpos 0.2. What torque, and in which direction?
4. Why should a controller write `data.ctrl` and not `model.actuator_gear` or `data.actuator_force`?

Change one line at a time. Print `ctrl`, `actuator_force`, `qfrc_actuator` and `qpos` before and after.

## Checkpoint

Report:

- The torque from ctrl 0.5 with gear 100, and where you read it.
- The ctrl that holds the pendulum at 0.5 rad, and how you checked it.
- One sentence each on motor, position and velocity actuators.
- Which actuator values live in the model and which in the data.

Do not mark this lesson complete until you can predict `qfrc_actuator` from `ctrl` before running the script.
