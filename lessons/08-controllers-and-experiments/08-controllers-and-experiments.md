# Lesson 08: Controllers and experiments

> **The interactive lesson is in `lessons/index.html` (Lesson 8).** It has the step-response explorer, the timestep picture, the predict boxes and the PD sandbox. These notes are the short written version.

## Goal

Write a PD controller from scratch, tune its gains by experiment, explain why a big timestep breaks it, and log a run so anyone can repeat it.

Prerequisites: Lesson 5 (the motor: torque = gear × ctrl) and Lesson 6 (reading the state through sensors).

## Model

The course pendulum (22.2509 kg capsule, 1 m long, hinge `hinge` about +y at z = 1 m, timestep 0.002 s) with the Lesson 5 motor and two joint sensors:

```xml
<actuator>
  <motor name="motor" joint="hinge" gear="100" ctrlrange="-1 1" ctrllimited="true"/>
</actuator>
<sensor>
  <jointpos name="angle" joint="hinge"/>
  <jointvel name="spin"  joint="hinge"/>
</sensor>
```

Numbers read from MuJoCo 3.15.0:

| Quantity | Value | Where it comes from |
|---|---|---|
| turning inertia I | 7.8764 kg m² | `mj_fullM(model, data, M)` (there is no `data.qM` shortcut in 3.15) |
| m g d | 109.1404 N m | `body_mass[1]`, `body_ipos[1]`, gravity |
| strongest torque | 100 N m | gear 100 × ctrl 1 |
| torque to hold 1.0 rad | 91.839 N m (ctrl 0.91839) | m g d sin(1): 92 % of the limit |
| highest angle it can hold | 1.1586 rad | asin(100 / 109.14) |

## Mental model: the control loop

```text
every step:
  1. read      q = data.qpos[0], qvel = data.qvel[0]           (rad, rad/s, joint frame)
  2. compute   tau = kp (target - q) - kd qvel  [+ m g d sin q]  (N m)
               data.ctrl[0] = clip(tau / gear, -1, 1)           (gear = 100)
  3. step      mujoco.mj_step(model, data)
```

- `data.ctrl` is the controller's output. Torque on the hinge = 100 × ctrl, reported in `data.qfrc_actuator[0]` (N m). `data.actuator_force[0]` is the same thing before the gear (ctrl units).
- If you write ctrl = 8.0, MuJoCo keeps 8.0 in `data.ctrl` but uses 1.0: the motor is **saturated**.
- Sensors are computed at the start of `mj_step`, so right after a step `sensordata` still shows the old state (0.000051 rad when `qpos` is already 0.000152 rad). Read `qpos`/`qvel` directly, or call `mj_forward` first.

## Runnable example

```bash
uv run --python .venv/bin/python lessons/08-controllers-and-experiments/08_controllers_and_experiments.py
```

It prints the model facts, three turns of the loop, five experiments, and writes `08_pd_log.csv`. Every run starts hanging still (q = 0) and asks for 1.0 rad.

Check that the web page's copy of the maths matches MuJoCo (needs `node`):

```bash
uv run --python .venv/bin/python lessons/08-controllers-and-experiments/08_check_browser_copy.py
```

## What the experiments show

1. **P only (kd = 0) never settles.** It swings around the balance angle q* forever. kp = 400 and kp = 1600 both swing up to 1.2513 rad: the motor is clipped at 100 N m either way.
2. **kd sweep at kp = 400** (critical kd = 2 √(kp I) = 112.26):

   | kd | damping ratio | rise (s) | overshoot (%) | settle (s) | ss error (rad) |
   |---|---|---|---|---|---|
   | 10 | 0.089 | 0.364 | 47.4 | not settled | 0.1113 |
   | 40 | 0.356 | 0.370 | 25.4 | 1.674 | 0.1968 |
   | 112 | 0.998 | 0.514 | 0.1 | 0.670 | 0.1964 |
   | 400 | 3.563 | 1.892 | 0.0 | not settled | 0.2172 |

3. **Gravity leaves a steady-state error.** At rest kp e = m g d sin(target - e). For kp = 400 the balance equation gives 0.1964 rad and MuJoCo measures 0.1964 rad. Doubling kp to 800 gives 0.1063. Adding m g d sin(q) (gravity compensation) makes it 0.000000.
4. **Timestep vs stability** (kp = kd = 400, gravity comp on): stable while kp h²/I + 2 kd h/I < 4, an edge at h = 0.03864 s (MuJoCo search: 0.03859 s). Past it the motor buzzes between its limits; with the limit switched off the wobble explodes (passes 1 rad at 2.16 s for h = 0.04).
5. **Reproducible log.** `08_pd_log.csv` has columns `time_s, qpos_rad, qvel_rad_s, ctrl, target_rad` (1501 rows) and six `#` header lines with the MuJoCo version, model, timestep, gains and the ctrl rule. Read it with `np.loadtxt(path, delimiter=",")`.

## Exercise: predict first

1. kd = 0: what happens?
2. Double kp from 400 to 800: what happens to the steady-state error?
3. Timestep 10× larger (0.02 s) with kp = kd = 400: stable?
4. Your controller computes tau = 250 N m. What goes into `data.ctrl[0]`, and what torque does the hinge get?

Change one number at a time and compare with the tables above.

## Checkpoint

Report:

- The loop order and why ctrl must be written before `mj_step`.
- Gains (with gravity compensation) that settle within 1 s with under 5 % overshoot.
- The largest stable timestep for kp = kd = 400 and the formula that predicts it.

Next: Lesson 9 wraps this controlled pendulum in a Gymnasium-style environment.
