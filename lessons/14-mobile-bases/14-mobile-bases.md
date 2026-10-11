# Lesson 14: Mobile bases

> **The interactive lesson is in `lessons/index.html` (Lesson 14).** It has the recorded MuJoCo runs, the yaw and skid-steer pictures, the rangefinder picture, the predict boxes and the waypoint-driving sandbox. These notes are the short written version.

## Goal

Drive a four-wheeled base: read its pose from the free joint (position + quaternion to yaw), command the wheels with velocity actuators, measure skid-steer kinematics, read a gyro and a rangefinder, and compare open-loop with closed-loop waypoint driving.

Prerequisites: Lesson 4 (free joints, quaternions, `xmat`), Lesson 5 (velocity actuators), Lesson 6 (sensors), Lesson 7 (friction), Lesson 8 (PD control).

## Model

A small copy of the Capstone 2 base from `lessons/16-capstone-warehouse-robot/warehouse_robot.xml`: the same chassis box (8.0 kg + 0.5 kg top plate), the same four cylinder wheels (radius 0.06 m, 0.3 kg, at x = +-0.18 m, y = +-0.17 m), the same `robot` and `wheel` default classes (chassis `contype="0"`, wheels friction 1.5 condim 6, damping 0.1, armature 0.005), the same four velocity actuators (kv 5, forcerange -10 10), the same option line (implicit, Newton, elliptic cone, impratio 10, timestep 0.002) and the same gyro / accelerometer / rangefinder. The arm, shelves and objects are left out. A wall stands at x = 3 m (face at 2.95 m). The robot starts at the origin facing +x.

| Quantity | Value | Where it comes from |
|---|---|---|
| nq / nv / nu | 11 / 10 / 4 | free joint 7 (6) + four wheel hinges |
| total mass | 9.700 kg | `body_subtreemass` |
| resting height | z = 0.1098 m on 8 contacts | after 0.5 s settling |
| sensordata | gyro [0:3], accelerometer [3:6], rangefinder [6] | `sensor_adr`, `sensor_dim` |

## Mental model

```text
pose:     qpos[0:3] = x, y, z (m, world)        qpos[3:7] = quaternion [w, x, y, z]
          yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))  or  atan2(xmat[1,0], xmat[0,0])
wheels:   ctrl = [FL, FR, RL, RR] target spin (rad/s); torque = 5 (ctrl - qvel), clipped to +-10 N m
motion:   v = r (wL + wR) / 2   (works)
          w = r (wR - wL) / B_eff,  B_eff = 1.23 to 1.55 m, NOT the real 0.34 m (wheels scrub sideways)
sensors:  gyro z = turn rate (rad/s, chassis frame); rangefinder = distance along the site's +z (m), -1 = no hit
control:  heading_err = wrap(atan2(dy, dx) - yaw)
          turn  = clip(6 heading_err - 2 gyro_z, -6, 6)
          speed = clip(max(4 dist max(0, 1 - |heading_err|/0.8), 1.5 if |heading_err| < 0.4), 0, 6)
          left = speed - turn, right = speed + turn; stop inside 0.05 m
```

- After `mj_step`, `xmat` (and the sensors) describe the pose at the *start* of the step. Read yaw from `qpos[3:7]`, or call `mj_forward` first.
- Free-joint `qvel[0:3]` is in the world frame, `qvel[3:6]` in the chassis frame. The gyro equals `qvel[3:6]` here.

## Runnable example

```bash
cd /Users/mohwit/Desktop/mujoco && .venv/bin/python lessons/14-mobile-bases/14_mobile_bases.py
```

It prints eight parts (model, pose, actuators, skid-steer, sensors, open vs closed loop, change one thing, calibration) and writes `14_recorded_runs.json` (the poses the web page replays). It takes about 4 s.

Check that the web page's kinematic model is the same maths as its Python copy (needs `node`):

```bash
.venv/bin/python lessons/14-mobile-bases/14_check_browser_copy.py
```

## What the script shows

1. **Yaw both ways.** After 3 s at left -5 / right +5 rad/s and 0.5 s stopped: quaternion [0.7872, 0, 0, 0.6167], yaw 1.329145 rad (76.154 deg) from the quaternion and from `xmat`; difference 4.44e-16 rad. The chassis stayed at (0.0000, 0.0000).
2. **Velocity actuators.** ctrl 5 rad/s gives real wheel speed 4.8974 rad/s and an average torque of 0.5132 N m = 5 x (5 - 4.8974). Spinning on the spot (-5 / +5) the wheels drop to 4.5877 rad/s and need 2.0624 N m: four times the effort.
3. **Skid-steer** (averages 2 to 6 s, real wheel speeds):

   | ctrl L, R | v (m/s) | r(wL+wR)/2 | w (rad/s) | r(wR-wL)/0.34 | B_eff (m) |
   |---|---|---|---|---|---|
   | 5, 5 | 0.2938 | 0.2938 | 0.0000 | 0.0000 | - |
   | -5, 5 | 0.0000 | 0.0000 | 0.4460 | 1.6192 | 1.234 |
   | -2, 2 | -0.0001 | 0.0000 | 0.1541 | 0.5876 | 1.296 |
   | 0, 5 | 0.1459 | 0.1470 | 0.1954 | 0.7596 | 1.322 |
   | 2, 5 | 0.2056 | 0.2058 | 0.1013 | 0.4185 | 1.404 |
   | 4, 6 | 0.2934 | 0.2940 | 0.0554 | 0.2524 | 1.549 |

   The speed formula holds. The turn formula needs an effective track width 3.63 to 4.55 times the real 0.34 m, because the front and rear wheels must slide sideways (scrub) to turn. The spin is a stick-slip chatter: 0.3119 to 0.4815 rad/s, about 21 times a second.
4. **Sensors.** Gyro z matches (yaw change / h) to about 3e-5 rad/s (the gyro uses chassis axes and the chassis rocks a little). The rangefinder matches (2.95 - site x) / cos(yaw) to 4 decimals: 2.6900 m at the start, -1.0 when facing +y. The accelerometer at rest reads [0, 0, 9.81].
5. **Route (0,0) -> W1 (1.5, 0) -> W2 (1.5, 1.0).**

   | Controller | miss W2 by | time |
   |---|---|---|
   | open loop, turn time from B = 0.34 (0.890 s) | 1.0764 m | 10.22 s |
   | open loop, turn time from B_cal = 1.345 m (3.522 s) | 0.0211 m | 12.86 s |
   | closed loop (Capstone 2 gains, 0.05 m arrival radius) | 0.0490 m | 21.99 s |

6. **Change one thing** (same plan, same gains):

   | Change | open loop miss | closed loop miss | closed loop time |
   |---|---|---|---|
   | none | 0.0211 m | 0.0490 m | 21.99 s |
   | floor + wheel friction 1.5 -> 0.3 | 0.9040 m (turned to 142.4 deg) | 0.0485 m | 20.54 s |
   | +5 kg on the chassis | 0.1038 m | 0.0502 m | 24.12 s |
   | right motors 10 % slow | 0.1419 m | 0.0482 m | 24.30 s |

   The slippery floor makes the spin *faster* (0.7139 rad/s, B_eff 0.813 m): less friction fights the scrub, so the timed turn overshoots.
7. **Browser calibration.** Fitting nine steady commands with the commands as wheel speeds gives B_eff = 1.4314 m and a speed factor 0.9776. The same closed-loop controller on that kinematic unicycle (dt 0.01 s) arrives 0.0499 m from W2 in 21.89 s (MuJoCo: 0.0490 m, 21.99 s). The page's JavaScript copy matches the Python copy to 4e-16.

## Exercise: predict first

1. q = [0.7872, 0, 0, 0.6167]: which way is the robot facing? (76.154 deg)
2. left 0, right 5 rad/s: the textbook formula says 0.88 rad/s. What does MuJoCo show, and why? (0.1954 rad/s; scrub)
3. The rangefinder reads -1. What does it mean? (nothing hit)
4. Open loop arrives within 2 cm on the lab floor. Add 5 kg. Which controller do you trust? (closed loop: 0.0502 m vs 0.1038 m, but 24.12 s vs 12.86 s)

Success criterion: in the sandbox, set a goal behind the robot, set the disturbance to -20 %, and explain why open loop misses while closed loop arrives.
