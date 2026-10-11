# Lesson 13: Grippers and grasping

> **The interactive lesson is in `lessons/index.html` (Lesson 13).** It covers the same ideas with pictures you can drive, predict boxes, and a grasp sandbox that replays numbers recorded from MuJoCo.

## Goal

Build a parallel-jaw gripper (two slide fingers coupled by a tendon and an equality, one position actuator, two touch sensors), then predict and test how hard it must squeeze to lift a box without slipping.

## Mental model

Pick up a glass with two fingers. The squeeze points sideways; the weight points down. What holds the glass up is **friction** along the finger faces, and friction has a limit of mu x N per finger. So:

    holds if   2 mu N >= m g        (N = squeeze of ONE finger)

Squeeze harder, or use grippier pads, when the object is heavier or more slippery.

## The model

The gripper is cut from the warehouse robot (`lessons/16-capstone-warehouse-robot/warehouse_robot.xml`) and put on a simple lift:

| Part | Value | Source |
|---|---|---|
| `hand` on slide joint `lift` (z), position actuator kp 3000 N/m | palm 0.5 kg, never collides | this lesson |
| `grip_L`, `grip_R` slide joints, range 0 to 0.05 m, damping 10 | 0 = closed | warehouse |
| finger class | friction 2.0 0.05 0.01, condim 4 | warehouse |
| box (object class) | 7 cm cube, 0.25 kg, friction 0.6 0.01 0.001, condim 3 | warehouse |
| fixed tendon `grip_L - grip_R` + tendon equality `polycoef="0 0 0 0 0"` | couples the fingers | warehouse |
| `gripper_pos` position actuator on grip_L only | kp 600 N/m, ctrlrange 0 to 0.05 | warehouse |
| `touch_L`, `touch_R` on thin box sites over the inner faces | N | warehouse |
| option | implicit, Newton, elliptic cone, impratio 10, timestep 0.002 | warehouse |

Arrays: `qpos[0]` lift (m), `qpos[1]` grip_L opening (m), `qpos[2]` grip_R opening (m), `qpos[3:10]` box free joint (world position m, quaternion). `nq = 10`, `nv = 9`. `ctrl[1]` is the gripper target opening (m), `actuator_force[1] = kp (ctrl - qpos[1])` (N), `sensordata[0:2]` the touch readings (N), `ten_length[0]` = grip_L - grip_R (m).

## Runnable example

```bash
uv run --python .venv/bin/python lessons/13-grippers-and-grasping/13_grippers_and_grasping.py
```

The sandbox data comes from a helper that runs 250 grasps (10 squeeze settings x 5 masses x 5 finger frictions) and writes `grip_grid.json`:

```bash
uv run --python .venv/bin/python lessons/13-grippers-and-grasping/record_grip_grid.py
```

## What the script shows

**B. Coupling.** A fixed tendon only measures `grip_L - grip_R`; the equality turns "length = 0" into forces. A joint equality `<joint joint1="grip_R" joint2="grip_L" polycoef="0 1 0 0 0"/>` gives identical numbers. Closing the empty gripper to 0.020 m:

```
tendon + equality (warehouse)  after 1 step: grip_L=0.039623 grip_R=0.039691 ten_length=-6.86e-05   after 0.5 s: grip_L=0.020000 grip_R=0.020001 m   ten_length=-8.35e-07 m
no coupling                    after 1 step: grip_L=0.039314 grip_R=0.040000 ten_length=-6.86e-04   after 0.5 s: grip_L=0.020000 grip_R=0.040000 m   ten_length=-2.00e-02 m
```

`ten_length` is computed at the start of a step, so the script calls `mj_forward` before reading it (lesson 3's stale-value rule). The equality is soft, like contacts: while squeezing it gives way, -7.66e-04 m at 0.85 N per finger and -5.59e-03 m at 6.99 N. `solref="0.005 1"` on the equality cuts the latter to -5.60e-04 m.

**C. The squeeze.** A position actuator is a spring: aim its target inside the box and it keeps pushing. The faces touch the box at an opening of 0.030 m, so `ctrl = 0.030 - 2 x setting / 600`. The equality hands half of the actuator force to the other finger, so each finger squeezes half. For setting 1.0 N, held in the air:

```
qpos grip_L=0.029512 grip_R=0.030278 m   equality violation grip_L-grip_R=-7.66e-04 m
actuator_force = kp (ctrl - grip_L) = -1.7070 N  -> half of it per finger 0.8535 N
touch_L=0.8533 N  touch_R=0.8535 N
friction carried by both fingers = 2.4525 N   weight m g = 2.4525 N
```

The measured squeeze (0.8535 N) is below the setting because the finger stops 0.49 mm past 0.030 m (soft contacts, soft equality). On the floor the squeeze is lopsided (1.5629 N vs 0.2607 N) because floor friction holds the box sideways while the right finger lags.

MuJoCo uses the **larger** friction (and condim) of the two geoms by default: finger 0.3 with box 0.6 gives `contact.friction[0] = 0.6`.

**D. Hold or slip.** Close 0.5 s, lift 10 cm in 1.0 s, hold 0.5 s; "held" = slip relative to the hand under 10 mm. Measured squeeze per finger for settings 0.5, 1, 2, 4, 8 N: 0.4083, 0.8535, 1.7307, 3.4735, 6.9918 N. The rule `2 mu N >= m g` agrees with MuJoCo in 30 of 30 script runs and 247 of 250 grid runs; adding the lift's peak acceleration 0.493 m/s^2 (`m (g + a)`) gives 249 of 250. Near the edge (0.25 kg, mu 2.0, needed 0.6131 N static, 0.6440 N with acceleration): 0.5850 N slips, 0.6296 N slips 32.55 mm, 0.6742 N holds.

One variable at a time: impratio 1 instead of 10 lets a held box creep 10x further (0.51 -> 5.18 mm; 1.70 -> 17.30 mm for 1.0 kg). Finger condim 3 instead of 4 changed nothing measurable here.

**E. Touch sensor vs contact pairs.** A touch sensor adds up every contact whose point lies inside its site. With the fingers open around the box and pressed on the floor, each reads 12.052 N (2 of the 4 floor corners are inside the site; the full finger-floor push is 24.104 N), yet no finger touches the box. Check `data.contact` for finger geom <-> box geom pairs instead. This is exactly how the warehouse RL agent reward-hacked its grasp bonus (see the docstring of `warehouse_grasp_env.py`); Capstone 2 tells the full story.

## Checkpoint

1. A 1.0 kg box, contact mu 2.0: smallest squeeze per finger, and which setting first holds it? (2.4525 N; setting 4.0, measured 3.4735 N)
2. Finger friction 0.3, box 0.6: which mu goes into the rule? (0.6)
3. Delete the equality but keep the tendon: do the fingers mirror? (No)
4. A reward pays when both touch sensors exceed 1 N: how can a learner cheat, and how do you stop it?
5. A held box creeps 5.18 mm: which one setting do you change first? (impratio)

## Files

- `13_grippers_and_grasping.py`: the lesson script (parts A to E).
- `record_grip_grid.py`: records the sandbox grid into `grip_grid.json`.
- `_fragments/`: the course-site panel, sandbox and JavaScript merged into `lessons/index.html` by `lessons/_build/build.py`.
